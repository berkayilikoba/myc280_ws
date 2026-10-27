"""LIMO otonom görev katmanı: 'nesneyi al, getir' ROS 2 düğümü.

İş mantığı `gorev_yoneticisi.py` durum makinesinde bulunur.
Bu düğüm yalnızca ROS 2 arayüzleri ile durum makinesi arasında köprüdür:
  - Girişleri dinler ve durum makinesine olay olarak iletir.
  - Durum makinesinden dönen eylemleri uygular.

Arayüzler:
  - Kavrama katmanı:
      /kavrama/hedef (PointStamped, map)
      /kavrama/durum (String, dinleme)
      /kavrama/iptal (Empty)
  - Kol katmanı:
      /kol/kavra (Trigger istemcisi)
      /kol/birak (Trigger istemcisi)
      (kol_sahte=True iken servis yoksa 3 sn bekleyip başarılı sayar)
  - Gezinme:
      navigate_to_pose (NavigateToPose eylemi)
  - Görev girişi ve durumu:
      /gorev/istek (String, dinleme: ör. 'elma' veya 'iptal')
      /gorev/durum (String, yayınlama)
"""

import math
import os
import threading
import time
from typing import Any, List, Optional

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from limo_autonomy.gorev_yoneticisi import (
    Ayar,
    Eylem,
    GorevYoneticisi,
    Poz2D,
    load_nesneler,
)


def yaw_deg_to_quaternion(yaw_deg: float) -> Quaternion:
    """Derece cinsinden yaw açısını geometry_msgs Quaternion formatına dönüştürür."""
    rad = math.radians(yaw_deg)
    half = rad * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class GorevDugumu(Node):
    """LIMO görev yöneticisi ROS 2 düğümü."""

    def __init__(self, node_name: str = 'gorev_dugumu') -> None:
        super().__init__(node_name)

        # Parametreler
        self.declare_parameter('nesneler_dosyasi', '')
        self.declare_parameter('kol_sahte', True)
        self.declare_parameter('kol_zaman_asimi', 30.0)
        self.declare_parameter('kavrama_zaman_asimi', 240.0)
        self.declare_parameter('tasima_zaman_asimi', 180.0)
        self.declare_parameter('timer_periyot', 0.5)

        self._cb_group = ReentrantCallbackGroup()
        self._kilit = threading.RLock()
        self._aktif_nav_gh: Optional[Any] = None

        # Durum makinesini hazırla
        dosya_yolu = str(self.get_parameter('nesneler_dosyasi').value)
        if not dosya_yolu:
            try:
                from ament_index_python.packages import get_package_share_directory
                varsayilan_yol = os.path.join(
                    get_package_share_directory('limo_autonomy'),
                    'config',
                    'nesneler_lab_oda.yaml',
                )
                if os.path.exists(varsayilan_yol):
                    dosya_yolu = varsayilan_yol
            except Exception:
                pass

        if dosya_yolu and os.path.exists(dosya_yolu):
            self.get_logger().info(f"Yapılandırma yükleniyor: {dosya_yolu}")
            cfg = load_nesneler(dosya_yolu)
            ayar = cfg.get('ayar', Ayar())
            ayar.kavrama_zaman_asimi = float(self.get_parameter('kavrama_zaman_asimi').value)
            ayar.tasima_zaman_asimi = float(self.get_parameter('tasima_zaman_asimi').value)
            ayar.kol_zaman_asimi = float(self.get_parameter('kol_zaman_asimi').value)
            self.yonetici = GorevYoneticisi(
                nesneler=cfg['nesneler'],
                ev=cfg['ev'],
                birakma=cfg.get('birakma'),
                ayar=ayar,
            )
        else:
            self.get_logger().warn(f"Yapılandırma dosyası bulunamadı ({dosya_yolu}), varsayılan ile başlandı.")
            self.yonetici = GorevYoneticisi(
                nesneler={},
                ev=Poz2D(0.0, 0.0, 0.0),
                birakma=Poz2D(0.0, 0.0, 0.0),
                ayar=Ayar(
                    kavrama_zaman_asimi=float(self.get_parameter('kavrama_zaman_asimi').value),
                    tasima_zaman_asimi=float(self.get_parameter('tasima_zaman_asimi').value),
                    kol_zaman_asimi=float(self.get_parameter('kol_zaman_asimi').value),
                ),
            )

        # Yayıncılar
        self.pub_kavrama_hedef = self.create_publisher(PointStamped, '/kavrama/hedef', 10)
        self.pub_kavrama_iptal = self.create_publisher(Empty, '/kavrama/iptal', 10)
        self.pub_gorev_durum = self.create_publisher(String, '/gorev/durum', 10)

        # Abonelikler
        self.sub_gorev_istek = self.create_subscription(
            String,
            '/gorev/istek',
            self._gorev_istek_cb,
            10,
            callback_group=self._cb_group,
        )
        self.sub_kavrama_durum = self.create_subscription(
            String,
            '/kavrama/durum',
            self._kavrama_durum_cb,
            10,
            callback_group=self._cb_group,
        )

        # Servis İstemcileri (Kol)
        self.cli_kol_kavra = self.create_client(
            Trigger,
            '/kol/kavra',
            callback_group=self._cb_group,
        )
        self.cli_kol_birak = self.create_client(
            Trigger,
            '/kol/birak',
            callback_group=self._cb_group,
        )

        # Eylem İstemcisi (Nav2)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group,
        )

        # Periyodik denetim zamanlayıcısı (zaman aşımı kontrolleri için)
        timer_periyot = float(self.get_parameter('timer_periyot').value)
        self.timer = self.create_timer(
            timer_periyot,
            self._zaman_dongusu,
            callback_group=self._cb_group,
        )

        self.get_logger().info("GorevDugumu hazır.")

    def _simdi(self) -> float:
        """ROS saati üzerinden saniye cinsinden zaman."""
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _gorev_istek_cb(self, msg: String) -> None:
        """Görev istek konusundan gelen istekleri işler."""
        ad = msg.data.strip()
        self.get_logger().info(f"Görev isteği alındı: '{ad}'")
        t = self._simdi()
        with self._kilit:
            eylemler = self.yonetici.istek(ad, t)
        self._eylemleri_uygula(eylemler)

    def _kavrama_durum_cb(self, msg: String) -> None:
        """Kavrama katmanından gelen durum bildirimlerini işler."""
        durum_str = msg.data.strip()
        t = self._simdi()
        with self._kilit:
            eylemler = self.yonetici.olay('kavrama_durum', durum_str, t)
        self._eylemleri_uygula(eylemler)

    def _zaman_dongusu(self) -> None:
        """Aşama zaman aşımlarını tetiklemek üzere düzenli zaman olayını gönderir."""
        t = self._simdi()
        with self._kilit:
            eylemler = self.yonetici.olay('zaman', None, t)
        self._eylemleri_uygula(eylemler)

    def _eylemleri_uygula(self, eylemler: List[Eylem]) -> None:
        """Durum makinesinin ürettiği eylemleri ROS katmanında yürütür."""
        for eylem in eylemler:
            if eylem.tur == 'durum':
                msg = String(data=str(eylem.veri))
                self.pub_gorev_durum.publish(msg)
                self.get_logger().info(f"[DURUM] {eylem.veri}")

            elif eylem.tur == 'kavrama_hedef':
                pos = eylem.veri
                msg = PointStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x = float(pos.x)
                msg.point.y = float(pos.y)
                msg.point.z = float(getattr(pos, 'z', 0.0))
                self.pub_kavrama_hedef.publish(msg)
                self.get_logger().info(
                    f"Kavrama hedefi gönderildi: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})"
                )

            elif eylem.tur == 'kavrama_iptal':
                self.pub_kavrama_iptal.publish(Empty())
                self.get_logger().info("Kavrama iptal mesajı gönderildi.")

            elif eylem.tur in ('kol_kavra', 'kol_birak'):
                self._kol_eylemi_calistir(eylem.tur)

            elif eylem.tur == 'nav_git':
                self._nav_git_calistir(eylem.veri)

            elif eylem.tur == 'nav_iptal':
                self._nav_iptal_calistir()

    def _kol_eylemi_calistir(self, tur: str) -> None:
        """Kol kavrama veya bırakma servisini asenkron iş parçacığında çalıştırır."""
        def _worker():
            client = self.cli_kol_kavra if tur == 'kol_kavra' else self.cli_kol_birak
            kol_sahte = bool(self.get_parameter('kol_sahte').value)
            kol_zaman_asimi = float(self.get_parameter('kol_zaman_asimi').value)

            servis_var = client.wait_for_service(timeout_sec=0.2)
            if not servis_var and kol_sahte:
                self.get_logger().info(f"'{tur}': servis bulunamadı, kol_sahte=True (3 saniye bekleniyor)...")
                time.sleep(3.0)
                with self._kilit:
                    eylemler = self.yonetici.olay('kol_sonuc', True, self._simdi())
                self._eylemleri_uygula(eylemler)
                return

            if not servis_var:
                self.get_logger().warn(f"'{tur}': servis bekleniyor ({kol_zaman_asimi}s)...")
                servis_var = client.wait_for_service(timeout_sec=kol_zaman_asimi)
                if not servis_var:
                    self.get_logger().error(f"'{tur}': servis bulunamadı, zaman aşımı.")
                    with self._kilit:
                        eylemler = self.yonetici.olay('kol_sonuc', False, self._simdi())
                    self._eylemleri_uygula(eylemler)
                    return

            req = Trigger.Request()
            fut = client.call_async(req)
            start_t = time.time()
            while not fut.done() and (time.time() - start_t < kol_zaman_asimi):
                time.sleep(0.05)

            if not fut.done():
                self.get_logger().error(f"'{tur}': servis yanıtı zaman aşımına uğradı.")
                with self._kilit:
                    eylemler = self.yonetici.olay('kol_sonuc', False, self._simdi())
                self._eylemleri_uygula(eylemler)
                return

            res = fut.result()
            basarili = bool(res.success)
            self.get_logger().info(f"'{tur}' tamamlandı: basari={basarili}, mesaj='{res.message}'")
            with self._kilit:
                eylemler = self.yonetici.olay('kol_sonuc', basarili, self._simdi())
            self._eylemleri_uygula(eylemler)

        th = threading.Thread(target=_worker, daemon=True)
        th.start()

    def _nav_git_calistir(self, poz: Poz2D) -> None:
        """Nav2 navigate_to_pose eylemine hedef gönderir."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 navigate_to_pose eylem sunucusu bulunamadı.")
            with self._kilit:
                eylemler = self.yonetici.olay('nav_sonuc', False, self._simdi())
            self._eylemleri_uygula(eylemler)
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(poz.x)
        goal.pose.pose.position.y = float(poz.y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_deg_to_quaternion(poz.yaw_derece)

        self.get_logger().info(
            f"Nav2 hedefi gönderiliyor: x={poz.x:.2f}, y={poz.y:.2f}, yaw={poz.yaw_derece:.1f}°"
        )
        send_fut = self.nav_client.send_goal_async(goal)
        send_fut.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, fut: Any) -> None:
        """Nav2 hedef kabul/red yanıtını işler."""
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn("Nav2 hedefi reddetti.")
            with self._kilit:
                eylemler = self.yonetici.olay('nav_sonuc', False, self._simdi())
            self._eylemleri_uygula(eylemler)
            return

        self._aktif_nav_gh = gh
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, fut: Any) -> None:
        """Nav2 eylem sonucunu işler."""
        self._aktif_nav_gh = None
        res = fut.result()
        # STATUS_SUCCEEDED = 4
        basarili = (res.status == 4)
        self.get_logger().info(f"Nav2 hedef sonucu alındı: status={res.status} (basarili={basarili})")
        with self._kilit:
            eylemler = self.yonetici.olay('nav_sonuc', basarili, self._simdi())
        self._eylemleri_uygula(eylemler)

    def _nav_iptal_calistir(self) -> None:
        """Aktif Nav2 hedefini iptal eder."""
        if self._aktif_nav_gh is not None:
            self.get_logger().info("Aktif Nav2 görevi iptal ediliyor...")
            try:
                self._aktif_nav_gh.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Nav2 hedef iptali sırasında hata: {e}")
            self._aktif_nav_gh = None


def main(args: Optional[List[str]] = None) -> None:
    """Görev düğümü ana giriş noktası."""
    rclpy.init(args=args)
    dugum = GorevDugumu()
    executor = MultiThreadedExecutor()
    executor.add_node(dugum)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        dugum.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
