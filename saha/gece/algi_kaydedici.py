#!/usr/bin/env python3
"""algi_kaydedici.py — Uçtan uca kavrama testinde algı ve poz doğruluk kaydedicisi/raporlayıcısı.

Kullanım:
  1. Kayıt modu (arka planda çalışır):
     python3 algi_kaydedici.py cikti/algi_verileri.json

  2. Rapor modu (test bitiminde çalıştırılır):
     python3 algi_kaydedici.py --rapor cikti/
"""

import argparse
import json
import math
import os
import re
import signal
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    ROS_VAR = True
except ImportError:
    ROS_VAR = False
    Node = object


def yaw_q(q) -> float:
    """Quaternion -> 2B yaw açısı (radyan)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def aci_sar(aci_rad: float) -> float:
    """Açıyı [-pi, pi] aralığına sarar."""
    return math.atan2(math.sin(aci_rad), math.cos(aci_rad))


def nokta_tf_donustur(t, pt: tuple[float, float, float]) -> tuple[float, float, float]:
    """3B noktayı TransformStamped ile dönüştürür (Euler-Rodrigues)."""
    tx = float(t.transform.translation.x)
    ty = float(t.transform.translation.y)
    tz = float(t.transform.translation.z)
    qx = float(t.transform.rotation.x)
    qy = float(t.transform.rotation.y)
    qz = float(t.transform.rotation.z)
    qw = float(t.transform.rotation.w)

    px, py, pz = float(pt[0]), float(pt[1]), float(pt[2])
    rx = qy * pz - qz * py
    ry = qz * px - qx * pz
    rz = qx * py - qy * px

    cx = qw * px + rx
    cy = qw * py + ry
    cz = qw * pz + rz

    fx = qy * cz - qz * cy
    fy = qz * cx - qx * cz
    fz = qx * cy - qy * cx

    return (tx + px + 2.0 * fx, ty + py + 2.0 * fy, tz + pz + 2.0 * fz)


def robot_cercevesine_donustur(rx: float, ry: float, ryaw: float, nx: float, ny: float) -> tuple[float, float]:
    """Dünya koordinatlarındaki nesneyi robotun base_footprint çerçevesine çevirir."""
    dx = nx - rx
    dy = ny - ry
    c = math.cos(ryaw)
    s = math.sin(ryaw)
    x_r = c * dx + s * dy
    y_r = -s * dx + c * dy
    return float(x_r), float(y_r)


class AlgiKaydediciDugumu(Node):
    """Simülasyon sırasında odom_gercek, nesne_algi ve kavrama durumunu kaydeder."""

    def __init__(self, cikti_dosyasi: str, nesne_gercek_xyz=(5.62, 0.8, 0.04)):
        super().__init__('algi_kaydedici')
        self.cikti_dosyasi = cikti_dosyasi
        self.nesne_gercek = nesne_gercek_xyz  # Gazebo dünya koordinatları

        self.son_odom = None
        self.durum_kayitlari = []
        self.son_yaklasma_aktif = False
        self.calisiyor = True

        self.son_yaklasma_olcumleri = []
        self.son_yaklasma_hatalari_cm = []
        self.tum_olcumler = []
        self.tum_hatalar_cm = []

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Abonelikler
        self.sub_odom = self.create_subscription(
            Odometry, '/odom_gercek', self._odom_cb, 50
        )
        self.sub_durum = self.create_subscription(
            String, '/kavrama/durum', self._durum_cb, 10
        )
        self.sub_algi = self.create_subscription(
            PointStamped, '/kavrama/nesne_algi', self._algi_cb, 10
        )

        self.get_logger().info(f'algi_kaydedici hazır. Çıktı: {self.cikti_dosyasi}')

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.son_odom = {
            't': float(stamp),
            'x': float(p.x),
            'y': float(p.y),
            'z': float(p.z),
            'yaw': float(yaw_q(q)),
        }

    def _durum_cb(self, msg: String) -> None:
        simdi = self.get_clock().now()
        t_sec = simdi.nanoseconds / 1e9
        txt = msg.data.strip()
        kod = txt.split()[0] if txt else ''

        self.durum_kayitlari.append({
            't': float(t_sec),
            'durum': txt,
            'kod': kod,
        })

        if kod == 'SON_YAKLASMA':
            self.son_yaklasma_aktif = True
        elif kod in ('HAZIR', 'HATA', 'KURTARMA', 'PLAN', 'BOSTA'):
            self.son_yaklasma_aktif = False

    def _algi_cb(self, msg: PointStamped) -> None:
        simdi = self.get_clock().now()
        t_sec = simdi.nanoseconds / 1e9
        px, py, pz = float(msg.point.x), float(msg.point.y), float(msg.point.z)
        cerceve = msg.header.frame_id

        # 1. base_footprint çerçevesine dönüştür
        hata_cm = None
        p_bf = None
        try:
            t_bf = self.tf_buffer.lookup_transform(
                'base_footprint',
                cerceve,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.05),
            )
            p_bf = nokta_tf_donustur(t_bf, (px, py, pz))
        except Exception:
            p_bf = None

        # 2. Gazebo gerçeğiyle hata hesapla
        if p_bf is not None and self.son_odom is not None:
            rx, ry, ryaw = self.son_odom['x'], self.son_odom['y'], self.son_odom['yaw']
            x_bf_gercek, y_bf_gercek = robot_cercevesine_donustur(
                rx, ry, ryaw, self.nesne_gercek[0], self.nesne_gercek[1]
            )
            hata_m = math.hypot(p_bf[0] - x_bf_gercek, p_bf[1] - y_bf_gercek)
            hata_cm = float(hata_m * 100.0)
        else:
            # Yedek: Harita TF ile dünya koordinatına çevir
            try:
                t_map = self.tf_buffer.lookup_transform(
                    'map',
                    cerceve,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.05),
                )
                p_map = nokta_tf_donustur(t_map, (px, py, pz))
                hata_m = math.hypot(p_map[0] - self.nesne_gercek[0], p_map[1] - self.nesne_gercek[1])
                hata_cm = float(hata_m * 100.0)
            except Exception:
                hata_cm = None

        kayit = {
            't': float(t_sec),
            'cerceve': cerceve,
            'ham_nokta': [px, py, pz],
            'bf_nokta': list(p_bf) if p_bf else None,
            'son_yaklasma': self.son_yaklasma_aktif,
            'hata_cm': hata_cm,
        }

        self.tum_olcumler.append(kayit)
        if hata_cm is not None:
            self.tum_hatalar_cm.append(hata_cm)

        if self.son_yaklasma_aktif:
            self.son_yaklasma_olcumleri.append(kayit)
            if hata_cm is not None:
                self.son_yaklasma_hatalari_cm.append(hata_cm)

    def kaydet(self) -> None:
        """Toplanan verileri JSON olarak dosyaya yazar."""
        son_yaklasma_ort = (
            float(sum(self.son_yaklasma_hatalari_cm) / len(self.son_yaklasma_hatalari_cm))
            if self.son_yaklasma_hatalari_cm
            else None
        )
        tum_ort = (
            float(sum(self.tum_hatalar_cm) / len(self.tum_hatalar_cm))
            if self.tum_hatalar_cm
            else None
        )

        veri = {
            'nesne_gercek_dunya': list(self.nesne_gercek),
            'son_odom': self.son_odom,
            'son_yaklasma_olcum_sayisi': len(self.son_yaklasma_olcumleri),
            'son_yaklasma_ortalama_hata_cm': son_yaklasma_ort,
            'son_yaklasma_hatalari_cm': self.son_yaklasma_hatalari_cm,
            'tum_olcum_sayisi': len(self.tum_olcumler),
            'tum_ortalama_hata_cm': tum_ort,
            'tum_hatalar_cm': self.tum_hatalar_cm,
            'durum_kayitlari': self.durum_kayitlari,
        }

        os.makedirs(os.path.dirname(os.path.abspath(self.cikti_dosyasi)), exist_ok=True)
        with open(self.cikti_dosyasi, 'w', encoding='utf-8') as f:
            json.dump(veri, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f'Veriler kaydedildi: {self.cikti_dosyasi}')


def rapor_uret(cikti_dizini: str) -> None:
    """Test çıktı dizinindeki verileri ve eylem logunu analiz edip rapor basar."""
    algi_json = os.path.join(cikti_dizini, 'algi_verileri.json')
    eylem_log = os.path.join(cikti_dizini, 'eylem.log')
    durum_log = os.path.join(cikti_dizini, 'durum_kontrol.log')

    veri = {}
    if os.path.isfile(algi_json):
        try:
            with open(algi_json, 'r', encoding='utf-8') as f:
                veri = json.load(f)
        except Exception as e:
            print(f"Uyarı: {algi_json} okunamadı: {e}")

    eylem_metni = ""
    if os.path.isfile(eylem_log):
        try:
            with open(eylem_log, 'r', encoding='utf-8') as f:
                eylem_metni = f.read()
        except Exception as e:
            print(f"Uyarı: {eylem_log} okunamadı: {e}")

    durum_kontrol_metni = ""
    if os.path.isfile(durum_log):
        try:
            with open(durum_log, 'r', encoding='utf-8') as f:
                durum_kontrol_metni = f.read().strip()
        except Exception:
            pass

    # 1. Eylem sonucu ayrıştırma
    eylem_basarili = None
    if 'status: STATUS_SUCCEEDED' in eylem_metni or 'Goal finished with status: SUCCEEDED' in eylem_metni:
        eylem_basarili = True
    elif 'status: STATUS_ABORTED' in eylem_metni or 'status: STATUS_CANCELED' in eylem_metni:
        eylem_basarili = False

    m_basarili = re.search(r'basarili\s*[:=]\s*(true|false)', eylem_metni, re.IGNORECASE)
    if m_basarili:
        eylem_basarili = m_basarili.group(1).lower() == 'true'

    m_neden = re.search(r'neden\s*[:=]\s*([^\n\r,]+)', eylem_metni)
    eylem_neden = m_neden.group(1).strip() if m_neden else 'bilinmiyor'

    m_sure = re.search(r'sure_s\s*[:=]\s*([+-]?\d+(?:\.\d+)?)', eylem_metni)
    eylem_sure = float(m_sure.group(1)) if m_sure else 0.0

    m_tahmini_hata = re.search(r'tahmini_hata_cm\s*[:=]\s*([+-]?\d+(?:\.\d+)?)', eylem_metni)
    tahmini_hata_cm = float(m_tahmini_hata.group(1)) if m_tahmini_hata else 0.0

    # 2. Gazebo gerçeğine göre nesnenin robot çerçevesindeki konumu
    son_odom = veri.get('son_odom')
    nesne_dunya = veri.get('nesne_gercek_dunya', [5.62, 0.8, 0.04])
    nx, ny = float(nesne_dunya[0]), float(nesne_dunya[1])

    if son_odom:
        rx, ry, ryaw = float(son_odom['x']), float(son_odom['y']), float(son_odom['yaw'])
        x_r, y_r = robot_cercevesine_donustur(rx, ry, ryaw, nx, ny)
        d_gercek = math.hypot(x_r, y_r)
        d_kavrama = 0.22  # Hedef kavrama mesafesi
        fark_cm = (d_gercek - d_kavrama) * 100.0
        dx_cm = (x_r - d_kavrama) * 100.0
        dy_cm = y_r * 100.0
        yon_hatasi_deg = math.degrees(math.atan2(y_r, x_r))
    else:
        rx, ry, ryaw = 0.0, 0.0, 0.0
        x_r, y_r, d_gercek = 0.0, 0.0, 0.0
        fark_cm, dx_cm, dy_cm = 0.0, 0.0, 0.0
        yon_hatasi_deg = 0.0

    # 3. Algı ölçüm sayıları
    son_yaklasma_sayisi = int(veri.get('son_yaklasma_olcum_sayisi', 0))
    tum_algi_sayisi = int(veri.get('tum_olcum_sayisi', 0))

    # 4. Algı hatası
    son_yaklasma_ort_hata = veri.get('son_yaklasma_ortalama_hata_cm')
    tum_ort_hata = veri.get('tum_ortalama_hata_cm')

    son_yaklasma_hatalar = veri.get('son_yaklasma_hatalari_cm', [])
    min_hata = min(son_yaklasma_hatalar) if son_yaklasma_hatalar else None
    max_hata = max(son_yaklasma_hatalar) if son_yaklasma_hatalar else None

    # Rapor Metni
    ayrac = "=" * 70
    satirlar = [
        ayrac,
        "UÇTAN UCA KAVRAMA ALGI VE DOĞRULUK RAPORU (renk_algi)",
        ayrac,
        f"Test Çıktı Dizini : {cikti_dizini}",
        f"Başlangıç Durumu  : {durum_kontrol_metni or 'bilinmiyor'}",
        "",
        "1. EYLEM SONUCU (/kavra):",
        f"   Başarılı mı      : {eylem_basarili}",
        f"   Neden Açıklaması : {eylem_neden}",
        f"   Eylem Süresi     : {eylem_sure:.1f} s",
        f"   Tahmini P Hatası : {tahmini_hata_cm:.2f} cm",
        "",
        "2. GAZEBO GERÇEĞİNE GÖRE ROBOT VE NESNE KONUMU:",
        f"   Robot Pozu (/odom_gercek) : x = {rx:+.3f} m, y = {ry:+.3f} m, yaw = {math.degrees(ryaw):+.1f}°",
        f"   Elma Gazebo Pozu (dünya)  : x = {nx:.3f} m, y = {ny:.3f} m, z = {nesne_dunya[2]:.3f} m",
        f"   Nesnenin Robottaki Konumu : x_r = {x_r:+.3f} m (hedef 0.220 m), y_r = {y_r:+.3f} m",
        f"   Kavrama Mesafesi (Öklid)  : {d_gercek:.3f} m",
        f"   d_kavrama (0.22 m) Farkı  : {fark_cm:+.2f} cm (Δx: {dx_cm:+.2f} cm, Δy: {dy_cm:+.2f} cm)",
        f"   Yön Hatası (açısal)       : {yon_hatasi_deg:+.2f}°",
        "",
        "3. ALGI ÖLÇÜM SAYILARI:",
        f"   Son Yaklaşma Sırasında Algı Mesaj Sayısı : {son_yaklasma_sayisi}",
        f"   Tüm Test Boyunca Toplam Algı Mesajı      : {tum_algi_sayisi}",
        "",
        "4. ALGININ GAZEBO GERÇEĞİNE GÖRE HATASI:",
    ]

    if son_yaklasma_ort_hata is not None:
        satirlar.append(f"   Son Yaklaşma Ortalama Algı Hatası : {son_yaklasma_ort_hata:.2f} cm")
        if min_hata is not None and max_hata is not None:
            satirlar.append(f"   En Küçük Hata / En Büyük Hata     : {min_hata:.2f} cm / {max_hata:.2f} cm")
    else:
        satirlar.append("   Son Yaklaşma Ortalama Algı Hatası : Ölçüm yok (0 mesaj)")

    if tum_ort_hata is not None:
        satirlar.append(f"   Tüm Ölçümler Ortalama Hatası      : {tum_ort_hata:.2f} cm")

    satirlar.append(ayrac)
    rapor_metni = "\n".join(satirlar)
    print(rapor_metni)

    # sonuc.json dosyasına yapılandırılmış özet yaz
    sonuc_ozeti = {
        'basarili': bool(eylem_basarili) if eylem_basarili is not None else False,
        'neden': str(eylem_neden),
        'sure_s': float(eylem_sure),
        'robot_odom_gercek': {'x': rx, 'y': ry, 'yaw_rad': ryaw, 'yaw_deg': math.degrees(ryaw)},
        'elma_gazebo_gercek': {'x': nx, 'y': ny, 'z': float(nesne_dunya[2])},
        'nesne_robotta': {'x_r': x_r, 'y_r': y_r, 'd_m': d_gercek},
        'd_kavrama_hedef_m': 0.22,
        'd_kavrama_farki_cm': fark_cm,
        'yon_hatasi_derece': yon_hatasi_deg,
        'son_yaklasma_algi_sayisi': son_yaklasma_sayisi,
        'tum_algi_sayisi': tum_algi_sayisi,
        'son_yaklasma_ortalama_algi_hatasi_cm': son_yaklasma_ort_hata,
        'tum_ortalama_algi_hatasi_cm': tum_ort_hata,
    }

    sonuc_json_yolu = os.path.join(cikti_dizini, 'sonuc.json')
    try:
        with open(sonuc_json_yolu, 'w', encoding='utf-8') as f:
            json.dump(sonuc_ozeti, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"Uyarı: {sonuc_json_yolu} yazılamadı: {e}")


def main() -> None:
    parser = argparse.ArgumentParser(description='Kavrama algı ve doğruluk kaydedicisi/raporlayıcısı.')
    parser.add_argument('cikti', nargs='?', default='', help='Kayıt JSON dosyası veya dizini')
    parser.add_argument('--rapor', action='store_true', help='Rapor modunu çalıştır')
    args = parser.parse_args()

    if args.rapor:
        dizin = args.cikti if args.cikti else '.'
        rapor_uret(dizin)
        return

    if not ROS_VAR:
        print("Hata: ROS 2 modülleri yüklenemedi.")
        sys.exit(1)

    cikti_dosyasi = args.cikti if args.cikti else 'algi_verileri.json'

    rclpy.init()
    dugum = AlgiKaydediciDugumu(cikti_dosyasi)

    def sinyal_yakala(signum, frame):
        dugum.get_logger().info(f'Sinyal {signum} alındı, veriler kaydediliyor...')
        dugum.calisiyor = False

    signal.signal(signal.SIGINT, sinyal_yakala)
    signal.signal(signal.SIGTERM, sinyal_yakala)

    try:
        while rclpy.ok() and dugum.calisiyor:
            rclpy.spin_once(dugum, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        dugum.kaydet()
        dugum.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
