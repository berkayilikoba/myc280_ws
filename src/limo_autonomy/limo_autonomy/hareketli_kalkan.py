"""Hareketli Engel Kalkanı Düğümü (hareketli_kalkan.py).

Neden Var:
----------
Standart ROS 2 Humble collision_monitor düğümü (ör. FootprintApproach), çevredeki engelleri
DURAĞAN kabul eder. Engelleri yalnızca robotun kendi hız vektörüyle robot yönünde öteler;
hareketli engellerin (karşıdan veya yandan gelen insan, başka robot vb.) kendi hız vektörünü
hesaba katamaz. Bu durum özellikle simülasyon ve gerçek dünya testlerinde (ör. dinamik_engel.py
senaryolarında) 0.6 m/s karşıdan veya 0.8 m/s yandan yaklaşan engellerde çarpışma öncesi
yetersiz fren mesafesine (temas anında robotun hala ileri hızda olmasına) yol açar.
Öte yandan DurBolge güvenlik poligonunu ön tarafa doğru büyütmek, kavrama (grasping)
görevlerini imkansız kılar; çünkü kavrama son yaklaşmasında nesne gövde önüne ~6 cm'ye
kadar yaklaşmak zorundadır.

Hız Zincirindeki Yeri:
----------------------
Navigasyon / Hız Komutu Zinciri:
  [nav2 / smoothed / kavrama]
             │  (cmd_vel_giris / varsayılan: cmd_vel_smoothed)
             ▼
  ┌──────────────────────┐
  │   hareketli_kalkan   │  <-- Bu filtre düğümü (bağıl hız ve TTC tabanlı)
  └──────────────────────┘
             │  (cmd_vel_cikis / varsayılan: cmd_vel_kalkan)
             ▼
  ┌──────────────────────┐
  │  collision_monitor   │  <-- Humble collision_monitor (durağan acil fren tamponu)
  └──────────────────────┘
             │  (cmd_vel)
             ▼
     [robot tabanı / base]

Humble collision_monitor'dan Temel Farkları:
---------------------------------------------
1. Lidar noktalarını kümeleyip odom çerçevesinde takip eder (IzTakipci); engellerin mutlak
   hız vektörünü (v_iz) kestirir.
2. Yalnızca HAREKETLİ engelleri (|v_iz| >= min_engel_hizi, vars. 0.20 m/s) tehdit kabul eder.
   Durağan nesneler (kavrama hedefi silindir/kutu, mobilyalar, duvarlar) kalkanı ASLA tetiklemez.
3. Bağıl hız (v_rel = v_iz - v_robot) ve bağıl konumu kullanarak gerçek çarpışmaya kalan süreyi
   (TTC: Time To Collision) analitik olarak çözer.
4. TTC < ttc_dur (vars. 1.5 s) ise ileri hareketi DURDURUR; ancak GERİ hareketi ve dönüşü
   (wz) kısıtlamaz, böylece robot tehlikeden kaçabilir veya yerinde dönebilir.
5. TTC < ttc_yavas (vars. 3.0 s) ise hızı doğrusal olarak yumuşakça yavaşlatır.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Sequence

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    from visualization_msgs.msg import Marker, MarkerArray

    MARKER_VAR = True
except ImportError:
    Marker = None
    MarkerArray = None
    MARKER_VAR = False


# ---------------------------------------------------------------------------
# 1. Saf Fonksiyonlar ve Veri Yapıları (ROS'suz, Pytest ile Test Edilebilir)
# ---------------------------------------------------------------------------


def yaw_q(q: Any) -> float:
    """Quaternion yöneliminden (x, y, z, w) 2B yaw açısını (radyan) hesaplar."""
    qx = float(getattr(q, 'x', q[0] if isinstance(q, (list, tuple)) else 0.0))
    qy = float(getattr(q, 'y', q[1] if isinstance(q, (list, tuple)) else 0.0))
    qz = float(getattr(q, 'z', q[2] if isinstance(q, (list, tuple)) else 0.0))
    qw = float(getattr(q, 'w', q[3] if isinstance(q, (list, tuple)) else 1.0))
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(aci: float) -> float:
    """Açıyı (-pi, pi] aralığına sarar."""
    return math.atan2(math.sin(aci), math.cos(aci))


def poz_aradeger(
    gecmis: Sequence[tuple[float, float, float, float]],
    t: float,
    maks_dt: float = 0.2,
) -> tuple[float, float, float] | None:
    """Verilen t anındaki robot pozunu (x, y, yaw) odom geçmişinden doğrusal aradeğerle hesaplar.

    Parametreler:
        gecmis: Zaman damgasına göre sıralı [(t, x, y, yaw), ...] geçmiş tamponu.
        t: Hedef zaman damgası (saniye).
        maks_dt: En yakın ölçümden izin verilen azami zaman farkı (s, varsayılan 0.2).

    Döndürür:
        (x, y, yaw) pozu. Tampon boşsa veya hedef an en yakın kayıttan maks_dt'den
        daha uzaksa None döner.
    """
    if not gecmis:
        return None

    t_val = float(t)
    t_ilk = float(gecmis[0][0])
    t_son = float(gecmis[-1][0])

    # Hedef zaman tamponun öncesindeyse: en yakını kullan veya None dön
    if t_val < t_ilk:
        if (t_ilk - t_val) > float(maks_dt):
            return None
        return (float(gecmis[0][1]), float(gecmis[0][2]), float(gecmis[0][3]))

    # Hedef zaman tamponun sonrasındaysa: en yakını kullan veya None dön
    if t_val > t_son:
        if (t_val - t_son) > float(maks_dt):
            return None
        return (float(gecmis[-1][1]), float(gecmis[-1][2]), float(gecmis[-1][3]))

    # Hedef zaman tampon aralığında: iki komşu kayıt arasında doğrusal aradeğer
    n = len(gecmis)
    for i in range(n - 1):
        t0 = float(gecmis[i][0])
        t1 = float(gecmis[i + 1][0])
        if t0 <= t_val <= t1:
            if min(t_val - t0, t1 - t_val) > float(maks_dt):
                return None

            dt = t1 - t0
            if dt <= 1e-9:
                return (float(gecmis[i][1]), float(gecmis[i][2]), float(gecmis[i][3]))

            alpha = (t_val - t0) / dt
            x0, y0, yaw0 = float(gecmis[i][1]), float(gecmis[i][2]), float(gecmis[i][3])
            x1, y1, yaw1 = float(gecmis[i + 1][1]), float(gecmis[i + 1][2]), float(gecmis[i + 1][3])

            x = x0 + alpha * (x1 - x0)
            y = y0 + alpha * (y1 - y0)
            dyaw = wrap(yaw1 - yaw0)
            yaw = wrap(yaw0 + alpha * dyaw)
            return (float(x), float(y), float(yaw))

    return (float(gecmis[-1][1]), float(gecmis[-1][2]), float(gecmis[-1][3]))


def yon_tutarliligi(ham_hizlar: Sequence[tuple[float, float]], son_n: int = 3) -> float:
    """Son N ham hız ölçümünün ortalama birim vektörünün uzunluğunu hesaplar.

    Parametreler:
        ham_hizlar: İzin ham hız ölçümleri listesi [(vx, vy), ...].
        son_n: Değerlendirilecek son ölçüm sayısı (varsayılan 3).

    Döndürür:
        Birim vektörlerin ortalamasının büyüklüğü (0.0 - 1.0).
        Ölçümler aynı yöndeyse 1.0'a yakındır; yön rastgele dağılmışsa 0.0'a yakındır.
    """
    if not ham_hizlar:
        return 0.0

    olcumler = ham_hizlar[-son_n:]
    if not olcumler:
        return 0.0

    ux_sum = 0.0
    uy_sum = 0.0
    for vx, vy in olcumler:
        v = math.hypot(vx, vy)
        if v > 1e-6:
            ux_sum += vx / v
            uy_sum += vy / v

    n = len(olcumler)
    return math.hypot(ux_sum / n, uy_sum / n)


def tarama_noktalari(
    ranges: Sequence[float] | np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_menzil: float | None = 5.0,
    lidar_x: float = 0.0,
    lidar_y: float = 0.0,
    lidar_yaw: float = 0.0,
) -> np.ndarray:
    """LaserScan menzil dizisini base_link çerçevesindeki (N, 2) kartezyen noktalara çevirir.

    Parametreler:
        ranges: LaserScan menzil ölçümleri dizisi.
        angle_min: İlk ışının açısı (rad).
        angle_increment: Işınlar arası açı farkı (rad).
        range_min: Asgari geçerli menzil (m).
        range_max: Azami geçerli menzil (m).
        max_menzil: Dikkate alınacak azami menzil eşiği (m, varsayılan 5.0).
        lidar_x: Lidar sensörünün base_link x eksenindeki ofseti (m, varsayılan 0.0).
        lidar_y, lidar_yaw: Lidar'ın base_link'teki y ofseti (m) ve yaw'ı (rad). LIMO'da lidar
            (0.137, 0.015, yaw -0.1686) — yaw yok sayılırsa 2 m'de ~33 cm yanal hata.

    Döndürür:
        (N, 2) boyutunda numpy float dizisi. Tarama sıralamasını korur.
    """
    r = np.asarray(ranges, dtype=float)
    if r.size == 0:
        return np.empty((0, 2), dtype=float)

    ust_menzil = float(range_max)
    if max_menzil is not None:
        ust_menzil = min(ust_menzil, float(max_menzil))

    gecerli = np.isfinite(r) & (r > float(range_min)) & (r < ust_menzil)
    if not np.any(gecerli):
        return np.empty((0, 2), dtype=float)

    indeksler = np.nonzero(gecerli)[0]
    r_secili = r[indeksler]
    acilar = float(angle_min) + indeksler * float(angle_increment)

    acilar = acilar + float(lidar_yaw)
    x = r_secili * np.cos(acilar) + float(lidar_x)
    y = r_secili * np.sin(acilar) + float(lidar_y)
    return np.column_stack((x, y))


def kumele(
    noktalar: np.ndarray | Sequence[tuple[float, float]] | Sequence[Sequence[float]],
    esik: float = 0.10,
    min_nokta: int = 3,
) -> tuple[list[tuple[float, float]], list[float]]:
    """Ardışık tarama noktalarını Öklid mesafesi eşiğine göre tek geçişte kümeler.

    Tarama sıralı olduğu için ardışık noktalar arası mesafe kontrolü yeterlidir.

    Parametreler:
        noktalar: (N, 2) kartezyen tarama noktaları dizisi (sıralı).
        esik: İki ardışık noktanın aynı kümeye ait olması için azami mesafe (m, varsayılan 0.10).
        min_nokta: Bir kümenin geçerli sayılması için gereken asgari nokta sayısı (varsayılan 3).

    Döndürür:
        (merkezler, yaricaplar):
            merkezler: Küme merkezleri listesi [(cx, cy), ...].
            yaricaplar: Her kümenin yarıçapı [r, ...] (merkeze en uzak noktanın mesafesi).
    """
    pts = np.asarray(noktalar, dtype=float)
    if pts.shape[0] < min_nokta:
        return ([], [])

    kumeler: list[np.ndarray] = []
    mevcut_noktalar: list[np.ndarray] = [pts[0]]
    esik_kare = float(esik) * float(esik)

    for i in range(1, pts.shape[0]):
        dx = pts[i, 0] - pts[i - 1, 0]
        dy = pts[i, 1] - pts[i - 1, 1]
        dist_sq = dx * dx + dy * dy

        if dist_sq <= esik_kare:
            mevcut_noktalar.append(pts[i])
        else:
            if len(mevcut_noktalar) >= min_nokta:
                kumeler.append(np.array(mevcut_noktalar))
            mevcut_noktalar = [pts[i]]

    if len(mevcut_noktalar) >= min_nokta:
        kumeler.append(np.array(mevcut_noktalar))

    merkezler: list[tuple[float, float]] = []
    yaricaplar: list[float] = []

    for k in kumeler:
        cx = float(np.mean(k[:, 0]))
        cy = float(np.mean(k[:, 1]))
        merkezler.append((cx, cy))
        dists = np.hypot(k[:, 0] - cx, k[:, 1] - cy)
        r = float(np.max(dists)) if len(dists) > 0 else 0.0
        yaricaplar.append(r)

    return (merkezler, yaricaplar)


def kume_merkezleri_odoma(
    merkezler: Sequence[tuple[float, float]] | np.ndarray,
    robot_pose: tuple[float, float, float] | Sequence[float],
) -> list[tuple[float, float]]:
    """Robot gövdesindeki (base_link) küme merkezlerini odom çerçevesine dönüştürür.

    Parametreler:
        merkezler: base_link çerçevesindeki [(x, y), ...] merkez listesi.
        robot_pose: Robotun odom çerçevesindeki (x, y, yaw) pozu.

    Döndürür:
        Odom çerçevesindeki [(ox, oy), ...] merkez listesi.
    """
    rx = float(robot_pose[0])
    ry = float(robot_pose[1])
    ryaw = float(robot_pose[2]) if len(robot_pose) > 2 else 0.0
    c = math.cos(ryaw)
    s = math.sin(ryaw)

    merkezler_odom: list[tuple[float, float]] = []
    for pt in merkezler:
        x, y = float(pt[0]), float(pt[1])
        ox = rx + c * x - s * y
        oy = ry + s * x + c * y
        merkezler_odom.append((ox, oy))
    return merkezler_odom


@dataclass
class Iz:
    """Odom çerçevesinde takip edilen bir nesne izi."""

    id: int
    x: float
    y: float
    r: float
    vx: float = 0.0
    vy: float = 0.0
    gorulme: int = 1
    yas: int = 1
    t_ilk: float = 0.0
    t_son: float = 0.0
    ham_hizlar: list[tuple[float, float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.ham_hizlar and (self.vx != 0.0 or self.vy != 0.0):
            self.ham_hizlar = [(self.vx, self.vy)]

    @property
    def yaricap(self) -> float:
        return self.r

    @property
    def v(self) -> float:
        """İzin toplam hız büyüklüğü (m/s)."""
        return math.hypot(self.vx, self.vy)

    @property
    def hiz(self) -> float:
        """İzin toplam hız büyüklüğü (m/s)."""
        return math.hypot(self.vx, self.vy)

    @property
    def konum(self) -> tuple[float, float]:
        return (self.x, self.y)

    @property
    def hiz_vektoru(self) -> tuple[float, float]:
        return (self.vx, self.vy)

    @property
    def iz_yasi_s(self) -> float:
        """İzin saniye cinsinden yaşı."""
        # Yalnız gerçek zaman farkı: aynı damgalı tekrar güncellemeler yaşı yapay olarak büyütmesin (R1 B1.6)
        return max(0.0, self.t_son - self.t_ilk)

    def __getitem__(self, key: Any) -> Any:
        if isinstance(key, int):
            return (self.x, self.y, self.vx, self.vy, self.r)[key]
        if hasattr(self, key):
            return getattr(self, key)
        raise KeyError(key)


class IzTakipci:
    """Odom çerçevesindeki küme merkezlerini takip eden ve hız kestiren izleyici.

    Özellikler:
        - Öngörülen konum (x + vx*dt) tabanlı en yakın komşu eşleme (kapı: 0.35 m).
        - Belirsiz eşleşmelerde (>=2 aday küme veya >=2 aday iz) hızı ve görülmeyi sıfırlama.
        - Robot dönüş hızı |wz| > maks_donus_hizi iken hız güncellemesini atlama (konumu güncelle, hızı koru).
        - Üstel yumuşatılmış hız kestirimi (alfa: 0.4) ve ham hız geçmişi takibi.
        - Yaş ve görülme sayısı takibi.
        - 0.5 s boyunca görülmeyen izlerin temizlenmesi.
    """

    def __init__(
        self,
        kapi: float = 0.35,
        alfa: float = 0.4,
        silme_suresi: float = 0.5,
        maks_donus_hizi: float = 0.8,
    ) -> None:
        self.kapi: float = float(kapi)
        self.alfa: float = float(alfa)
        self.silme_suresi: float = float(silme_suresi)
        self.maks_donus_hizi: float = float(maks_donus_hizi)
        self._izler: list[Iz] = []
        self._sonraki_id: int = 1

    def sifirla(self) -> None:
        """Tüm izleri temizler ve sayaçları sıfırlar."""
        self._izler.clear()
        self._sonraki_id = 1

    def guncelle(
        self,
        merkezler_odom: Sequence[tuple[float, float]] | np.ndarray,
        yaricaplar: Sequence[float],
        t: float,
        robot_wz: float = 0.0,
    ) -> None:
        """Yeni küme merkezleriyle izleri günceller.

        Parametreler:
            merkezler_odom: Odom çerçevesindeki [(ox, oy), ...] merkez listesi.
            yaricaplar: Her kümenin yarıçapı [r, ...].
            t: Güncelleme zamanı (saniye).
            robot_wz: Robotun anlık açısal hızı (rad/s, varsayılan 0.0).
        """
        t_val = float(t)
        n_kume = len(merkezler_odom)
        hiz_guncelle_atla = abs(float(robot_wz)) > self.maks_donus_hizi

        if len(self._izler) == 0:
            # Henüz iz yoksa tüm kümeleri yeni iz olarak ekle
            for i in range(n_kume):
                cx, cy = float(merkezler_odom[i][0]), float(merkezler_odom[i][1])
                cr = float(yaricaplar[i]) if i < len(yaricaplar) else 0.20
                yeni_iz = Iz(
                    id=self._sonraki_id,
                    x=cx,
                    y=cy,
                    r=cr,
                    vx=0.0,
                    vy=0.0,
                    gorulme=1,
                    yas=1,
                    t_ilk=t_val,
                    t_son=t_val,
                    ham_hizlar=[],
                )
                self._sonraki_id += 1
                self._izler.append(yeni_iz)
            return

        # 1. Eşleştirmede izin öngörülen konumunu (x + vx*dt) kullan
        kapi_kare = self.kapi * self.kapi
        ciftler: list[tuple[float, int, int]] = []
        iz_adaylari: dict[int, list[int]] = {i: [] for i in range(len(self._izler))}
        kume_adaylari: dict[int, list[int]] = {j: [] for j in range(n_kume)}

        for i, iz in enumerate(self._izler):
            dt_ongoru = max(0.0, t_val - iz.t_son)
            pred_x = iz.x + iz.vx * dt_ongoru
            pred_y = iz.y + iz.vy * dt_ongoru

            for j in range(n_kume):
                cx = float(merkezler_odom[j][0])
                cy = float(merkezler_odom[j][1])
                dx = pred_x - cx
                dy = pred_y - cy
                d2 = dx * dx + dy * dy
                if d2 <= kapi_kare:
                    ciftler.append((d2, i, j))
                    iz_adaylari[i].append(j)
                    kume_adaylari[j].append(i)

        # 2. Belirsiz eşleşme tespiti:
        # Bir iz için kapı içinde >= 2 aday küme varsa veya bir kümeye >= 2 iz adaysa
        belirsiz_izler: set[int] = set()
        for i, k_list in iz_adaylari.items():
            if len(k_list) >= 2:
                belirsiz_izler.add(i)

        for j, i_list in kume_adaylari.items():
            if len(i_list) >= 2:
                for i_idx in i_list:
                    belirsiz_izler.add(i_idx)

        # 3. En kısa mesafeden başlayarak açgözlü (greedy) eşleme
        ciftler.sort(key=lambda item: item[0])

        eslesen_izler: set[int] = set()
        eslesen_kumeler: set[int] = set()

        for d2, i, j in ciftler:
            if i in eslesen_izler or j in eslesen_kumeler:
                continue
            eslesen_izler.add(i)
            eslesen_kumeler.add(j)

            iz = self._izler[i]
            cx = float(merkezler_odom[j][0])
            cy = float(merkezler_odom[j][1])
            cr = float(yaricaplar[j]) if j < len(yaricaplar) else iz.r

            if i in belirsiz_izler:
                # Belirsiz eşleşme: hız kestirimini sıfırla, görülmeyi sıfırla (yeniden kanıtlamalı)
                iz.vx = 0.0
                iz.vy = 0.0
                iz.gorulme = 0
                iz.ham_hizlar.clear()
            elif hiz_guncelle_atla:
                # Dönüş sırasında güvenilmezlik (|wz| > maks_donus_hizi):
                # Hız güncellemesini atla (konumu güncelle, hızı koru)
                pass
            else:
                dt = t_val - iz.t_son
                if dt > 1e-5:
                    v_an_x = (cx - iz.x) / dt
                    v_an_y = (cy - iz.y) / dt
                    iz.ham_hizlar.append((v_an_x, v_an_y))
                    if len(iz.ham_hizlar) > 10:
                        iz.ham_hizlar = iz.ham_hizlar[-10:]

                    if iz.gorulme <= 1:
                        # İlk hareket adımında doğrudan anlık hız atanır (soğuk başlangıç bias'ını önler)
                        iz.vx = v_an_x
                        iz.vy = v_an_y
                    else:
                        iz.vx = self.alfa * v_an_x + (1.0 - self.alfa) * iz.vx
                        iz.vy = self.alfa * v_an_y + (1.0 - self.alfa) * iz.vy

            iz.x = cx
            iz.y = cy
            iz.r = cr
            iz.t_son = t_val
            if i not in belirsiz_izler and not hiz_guncelle_atla:
                iz.gorulme += 1
            iz.yas += 1

        # Eşleşmeyen eski izlerin yaşını artır (ve belirsiz olanların hızını sıfırla)
        for i, iz in enumerate(self._izler):
            if i not in eslesen_izler:
                if i in belirsiz_izler:
                    iz.vx = 0.0
                    iz.vy = 0.0
                    iz.gorulme = 0
                    iz.ham_hizlar.clear()
                iz.yas += 1

        # Eşleşmeyen yeni kümelerden yeni iz oluştur
        for j in range(n_kume):
            if j not in eslesen_kumeler:
                cx = float(merkezler_odom[j][0])
                cy = float(merkezler_odom[j][1])
                cr = float(yaricaplar[j]) if j < len(yaricaplar) else 0.20
                yeni_iz = Iz(
                    id=self._sonraki_id,
                    x=cx,
                    y=cy,
                    r=cr,
                    vx=0.0,
                    vy=0.0,
                    gorulme=1,
                    yas=1,
                    t_ilk=t_val,
                    t_son=t_val,
                    ham_hizlar=[],
                )
                self._sonraki_id += 1
                self._izler.append(yeni_iz)

        # 0.5 s boyunca görülmeyen izleri temizle
        aktif_izler: list[Iz] = []
        for iz in self._izler:
            if (t_val - iz.t_son) <= (self.silme_suresi + 1e-6):
                aktif_izler.append(iz)
        self._izler = aktif_izler

    def izler(self) -> list[Iz]:
        """Aktif izler listesinin kopyasını döndürür."""
        return list(self._izler)


def carpismaya_kalan_sure(
    p_rel: tuple[float, float] | Sequence[float],
    v_rel: tuple[float, float] | Sequence[float],
    r_toplam: float,
) -> float:
    """Robot merkezli bağıl konum ve hızla |p + v t| = r_toplam olan en küçük t >= 0 süresini hesaplar.

    Parametreler:
        p_rel: Engel ile robot arasındaki bağıl konum vektörü (px, py).
        v_rel: Engel ile robot arasındaki bağıl hız vektörü (vx, vy) = v_iz - v_robot.
        r_toplam: İki cismin yarıçapları toplamı (iz_r + robot_r).

    Döndürür:
        En küçük çarpışma süresi (saniye, t >= 0). Gelecekte çarpışma yoksa math.inf.
        Zaten temas/iç içe durumdaysa (|p| <= r_toplam) yaklaşıyorsa 0.0, uzaklaşıyorsa math.inf döner.
    """
    px = float(p_rel[0])
    py = float(p_rel[1])
    vx = float(v_rel[0])
    vy = float(v_rel[1])
    r = float(r_toplam)

    p_sq = px * px + py * py
    r_sq = r * r

    # Zaten temas veya iç içe: yalnız hâlâ yaklaşıyorsa (p . v < 0) 0.0; uzaklaşıyorsa tehdit yok (inf).
    # Aksi halde tepkisiz engel robotun yanından geçip uzaklaşırken bile kalkan "dur"da kalıyordu (sim ölçümü).
    if p_sq <= r_sq:
        return 0.0 if (px * vx + py * vy) < 0.0 else math.inf

    v_sq = vx * vx + vy * vy
    # Bağıl hız sıfır veya ihmal edilebilir; mesafe r'den büyük olduğu için çarpışma olmaz
    if v_sq < 1e-12:
        return math.inf

    # |p + v t|^2 = r^2
    # v_sq * t^2 + 2 (p . v) t + (p_sq - r_sq) = 0
    # A t^2 + 2 b t + C = 0  (A = v_sq, b = p . v, C = p_sq - r_sq > 0)
    b = px * vx + py * vy

    # b >= 0 ise engel robottan uzaklaşıyor (t >= 0 için mesafe sürekli artar)
    if b >= 0.0:
        return math.inf

    c = p_sq - r_sq
    diskriminant = b * b - v_sq * c
    if diskriminant < 0.0:
        # Minimum yaklaşma mesafesi r'den büyük, çarpışma rotasında değil
        return math.inf

    sqrt_disc = math.sqrt(diskriminant)
    # b < 0 ve c > 0 olduğundan sqrt_disc < -b; en küçük pozitif kök:
    t = (-b - sqrt_disc) / v_sq
    return max(0.0, float(t))


class KalkanKarari(tuple):
    """(karar, olcek, gerekce) 3-demetini genişleten, ek öznitelikler sunan karar nesnesi."""

    karar: str
    olcek: float
    gerekce: str
    kritik_iz: Iz | None
    min_ttc: float

    def __new__(
        cls,
        karar: str,
        olcek: float,
        gerekce: str,
        kritik_iz: Iz | None = None,
        min_ttc: float = math.inf,
    ):
        obj = super().__new__(cls, (str(karar), float(olcek), str(gerekce)))
        obj.karar = str(karar)
        obj.olcek = float(olcek)
        obj.gerekce = str(gerekce)
        obj.kritik_iz = kritik_iz
        obj.min_ttc = float(min_ttc)
        return obj


def kalkan_karari(
    izler: Sequence[Iz],
    robot_v_odom: tuple[float, float] | Sequence[float] | float,
    robot_p_odom: tuple[float, float] | Sequence[float] = (0.0, 0.0),
    robot_yaricap: float = 0.20,
    min_engel_hizi: float = 0.20,
    min_gorulme: int = 3,
    min_iz_yasi_s: float = 0.35,
    yon_tutarlilik: float = 0.7,
    ttc_dur: float = 1.5,
    ttc_yavas: float = 3.0,
) -> KalkanKarari:
    """Hareketli izleri inceleyerek kalkan kararını (dur / yavas / serbest) üretir.

    Kural:
        Yalnız HAREKETLİ izler (|v_iz| >= min_engel_hizi vars. 0.20 m/s, gorulme >= min_gorulme vars. 3,
        iz_yasi_s >= min_iz_yasi_s vars. 0.35 s ve yon_tutarliligi >= yon_tutarlilik vars. 0.7, son N=3) dikkate alınır.
        Durağan nesneler (kavrama hedefi, mobilya) ASLA tetiklemez.
        TTC < ttc_dur (vars. 1.5 s) -> 'dur', olcek = 0.0
        TTC < ttc_yavas (vars. 3.0 s) -> 'yavas', olcek doğrusal (ttc_dur'da 0.0, ttc_yavas'ta 1.0)
        Aksi halde -> 'serbest', olcek = 1.0

    Döndürür:
        KalkanKarari: ('serbest'|'yavas'|'dur', ölçek 0..1, gerekçe metni)
    """
    if isinstance(robot_v_odom, (int, float)):
        vrx, vry = float(robot_v_odom), 0.0
    else:
        vrx = float(robot_v_odom[0])
        vry = float(robot_v_odom[1]) if len(robot_v_odom) > 1 else 0.0

    rx = float(robot_p_odom[0])
    ry = float(robot_p_odom[1])

    min_ttc = math.inf
    kritik_iz: Iz | None = None
    kritik_v: float = 0.0

    for iz in izler:
        v_iz = math.hypot(iz.vx, iz.vy)
        # 1. Hız eşiği kontrolü
        if v_iz < float(min_engel_hizi):
            continue

        # 2. Asgari görülme sayısı kontrolü
        if int(iz.gorulme) < int(min_gorulme):
            continue

        # 3. Asgari iz yaşı kontrolü (saniye)
        yas_s = getattr(iz, 'iz_yasi_s', iz.t_son - iz.t_ilk)
        if yas_s < float(min_iz_yasi_s):
            continue

        # 4. Hız yön tutarlılığı kontrolü (son N=3 ham hız)
        ham_hizlar = getattr(iz, 'ham_hizlar', [])
        if yon_tutarliligi(ham_hizlar, son_n=3) < float(yon_tutarlilik):
            continue

        p_rel = (iz.x - rx, iz.y - ry)
        v_rel = (iz.vx - vrx, iz.vy - vry)
        r_toplam = float(iz.r) + float(robot_yaricap)

        ttc = carpismaya_kalan_sure(p_rel, v_rel, r_toplam)
        if ttc < min_ttc:
            min_ttc = ttc
            kritik_iz = iz
            kritik_v = v_iz

    if min_ttc == math.inf or kritik_iz is None:
        return KalkanKarari('serbest', 1.0, 'serbest', kritik_iz=None, min_ttc=math.inf)

    if min_ttc < float(ttc_dur):
        karar = 'dur'
        olcek = 0.0
        gerekce = f"DUR ttc={min_ttc:.2f}s v={kritik_v:.2f}"
    elif min_ttc < float(ttc_yavas):
        karar = 'yavas'
        fark = float(ttc_yavas) - float(ttc_dur)
        olcek = (min_ttc - float(ttc_dur)) / fark if fark > 1e-6 else 0.0
        olcek = max(0.0, min(1.0, float(olcek)))
        gerekce = f"YAVAS ttc={min_ttc:.2f}s v={kritik_v:.2f}"
    else:
        karar = 'serbest'
        olcek = 1.0
        gerekce = 'serbest'

    return KalkanKarari(karar, olcek, gerekce, kritik_iz=kritik_iz, min_ttc=min_ttc)


def komutu_olcekle(
    vx: float,
    wz: float,
    karar: str,
    olcek: float,
    engel_yon_x: float | tuple[float, float] | Sequence[float] = 1.0,
) -> tuple[float, float]:
    """Hız komutunu kalkan kararına göre ölçekler.

    Kural:
        'dur' iken ileri bileşeni engele doğruysa (vx * yon_x > 0) 0 yapılır;
        GERİ (engelden uzaklaşan) hızı ve yerinde dönme (wz) kısılmaz.

    Parametreler:
        vx: İleri/geri çizgisel hız komutu (m/s).
        wz: Açısal hız komutu (rad/s).
        karar: 'serbest', 'yavas' veya 'dur'.
        olcek: Hız ölçeği (0.0 - 1.0).
        engel_yon_x: Engelin robot gövdesindeki x yönü (+1 ön, -1 arka veya vektör).

    Döndürür:
        (vx_cikis, wz_cikis)
    """
    vx_in = float(vx)
    wz_in = float(wz)

    if isinstance(engel_yon_x, (tuple, list, np.ndarray)):
        yon_x = float(engel_yon_x[0])
    else:
        yon_x = float(engel_yon_x)

    wz_out = wz_in

    # vx * yon_x > 0 ise robot engele doğru hareket etmektedir
    if vx_in * yon_x > 0.0:
        if karar == 'dur':
            vx_out = 0.0
        elif karar == 'yavas':
            s = max(0.0, min(1.0, float(olcek)))
            vx_out = vx_in * s
        else:  # 'serbest'
            vx_out = vx_in
    else:
        # Engelden uzaklaşan (geri) veya engele dik hareket kısılmaz
        vx_out = vx_in

    return (vx_out, wz_out)


def odom_zaman_asimi_kontrolu(
    now_s: float,
    robot_pose: tuple[float, float, float] | None,
    son_gecerli_pose_t: float | None,
    odom_gecmisi_bos: bool,
    odom_zaman_asimi: float = 0.5,
) -> bool:
    """Robot pozu aradeğerle bulunamadığında odomun boşalıp/eskidiğini (> odom_zaman_asimi) denetler (B2.2).

    Parametreler:
        now_s: Geçerli zaman (s).
        robot_pose: poz_aradeger sonucu veya None.
        son_gecerli_pose_t: En son geçerli robot pozunun zamanı (s).
        odom_gecmisi_bos: Odom geçmişi tamponunun boş olup olmadığı.
        odom_zaman_asimi: Odom zaman aşımı süresi (s, varsayılan 0.5).

    Döndürür:
        True: Odom geçmişi boş veya > 0.5 s eski; izler silinmeli, karar 'serbest' olmalı (ODOM_YOK).
        False: Poz geçerli veya kesinti henüz eşiği (0.5 s) aşmadı.
    """
    if robot_pose is not None:
        return False

    if odom_gecmisi_bos:
        return True

    if son_gecerli_pose_t is None:
        return True

    return (float(now_s) - float(son_gecerli_pose_t)) > float(odom_zaman_asimi)


def tarama_zaman_asimi_yonetimi(
    now_s: float,
    son_scan_duvar_t: float | None,
    son_karar: str,
    tarama_zaman_asimi: float = 0.5,
    eski_tarama_tutma_s: float = 0.5,
) -> tuple[str, str, float]:
    """Tarama zaman aşımı durumunda kararı ve uygulanacak eylemi belirler (B2.3).

    Kural:
        - Tarama taze ise (dt <= tarama_zaman_asimi): 'normal' çalışma.
        - Tarama eski ve son karar 'dur' ise: eski_tarama_tutma_s boyunca 'dur_tut' (karar 'dur', ölçek 0.0).
        - eski_tarama_tutma_s dolduğunda veya son karar 'dur' değilse: 'serbest_birak' (karar 'serbest', TARAMA_YOK).

    Parametreler:
        now_s: Geçerli zaman damgası (s).
        son_scan_duvar_t: Son geçerli tarama zamanı (s) veya None.
        son_karar: Son kalkan kararı ('dur', 'yavas', 'serbest').
        tarama_zaman_asimi: Taramayı eski sayma eşiği (s, varsayılan 0.5).
        eski_tarama_tutma_s: 'dur' kararının korunma süresi (s, varsayılan 0.5).

    Döndürür:
        (eylem, yeni_karar, yeni_olcek)
        eylem: 'normal' (taze tarama), 'dur_tut' (tutma süresinde dur kararı tutuluyor),
               'serbest_birak' (tutma süresi doldu veya dur değildi; serbest bırakıldı).
    """
    if son_scan_duvar_t is None:
        return ('serbest_birak', 'serbest', 1.0)

    dt = float(now_s) - float(son_scan_duvar_t)
    if dt <= float(tarama_zaman_asimi):
        return ('normal', son_karar, 0.0 if son_karar == 'dur' else 1.0)

    # Tarama zaman aşımına girdi (dt > tarama_zaman_asimi)
    dt_gecen = dt - float(tarama_zaman_asimi)
    if son_karar == 'dur' and dt_gecen < float(eski_tarama_tutma_s):
        return ('dur_tut', 'dur', 0.0)

    return ('serbest_birak', 'serbest', 1.0)


# ---------------------------------------------------------------------------
# 2. ROS 2 Düğümü: HareketliKalkan
# ---------------------------------------------------------------------------


class HareketliKalkan(Node):
    """Hareketli engel kalkanı ROS 2 düğümü.

    Hız zincirinde cmd_vel_giris konusunu dinler, hareketli engellere göre filtreleyip
    cmd_vel_cikis konusuna yayınlar.
    """

    def __init__(self) -> None:
        super().__init__('hareketli_kalkan')
        p = self.declare_parameter

        # Parametreler
        self.etkin = bool(p('etkin', True).value)
        cmd_giris = str(p('cmd_vel_giris', 'cmd_vel_smoothed').value)
        cmd_cikis = str(p('cmd_vel_cikis', 'cmd_vel_kalkan').value)
        scan_konu = str(p('scan_topic', 'scan').value)
        odom_konu = str(p('odom_topic', 'odom').value)

        # Lidar montajı: önce TF (base_link <- tarama çerçevesi, sabit; bir kez okunup saklanır), yoksa bu parametreler
        self.lidar_x = float(p('lidar_x', 0.0).value)
        self.lidar_y = float(p('lidar_y', 0.0).value)
        self.lidar_yaw = float(p('lidar_yaw', 0.0).value)
        self.govde_cercevesi = str(p('govde_cercevesi', 'base_link').value)
        self._lidar_tf_alindi = False
        self.tf_buffer = Buffer()
        self.tf_dinleyici = TransformListener(self.tf_buffer, self)
        self.max_menzil = float(p('max_menzil', 5.0).value)
        self.kume_esik = float(p('kume_esik', 0.10).value)
        self.kume_min_nokta = int(p('kume_min_nokta', 3).value)
        # Duvar/mobilya gibi büyük kümelerin merkezi robot hareket ettikçe kayar (kısmi görünüm) ve sahte hız
        # üretir; insan/kutu boyutundan büyük kümeler izlenmez.
        self.max_iz_yaricap = float(p('max_iz_yaricap', 0.35).value)

        eslesme_kapi = float(p('eslesme_kapi', 0.35).value)
        hiz_alfa = float(p('hiz_alfa', 0.4).value)
        iz_silme_suresi = float(p('iz_silme_suresi', 0.5).value)
        self.maks_donus_hizi = float(p('maks_donus_hizi', 0.8).value)

        self.robot_yaricap = float(p('robot_yaricap', 0.20).value)
        self.min_engel_hizi = float(p('min_engel_hizi', 0.20).value)
        self.min_gorulme = int(p('min_gorulme', 3).value)
        self.min_iz_yasi_s = float(p('min_iz_yasi_s', 0.35).value)
        self.yon_tutarlilik = float(p('yon_tutarlilik', 0.7).value)
        self.ttc_dur = float(p('ttc_dur', 1.5).value)
        self.ttc_yavas = float(p('ttc_yavas', 3.0).value)
        self.tarama_zaman_asimi = float(p('tarama_zaman_asimi', 0.5).value)
        self.eski_tarama_tutma_s = float(p('eski_tarama_tutma_s', 0.5).value)
        self.odom_zaman_asimi = float(p('odom_zaman_asimi', 0.5).value)

        # İz Takipçisi
        self.takipci = IzTakipci(
            kapi=eslesme_kapi,
            alfa=hiz_alfa,
            silme_suresi=iz_silme_suresi,
            maks_donus_hizi=self.maks_donus_hizi,
        )

        # Durum Değişkenleri
        self._robot_pose_odom: tuple[float, float, float] | None = None
        self._robot_v_odom: tuple[float, float] = (0.0, 0.0)
        self._robot_wz: float = 0.0
        self._odom_gecmisi: list[tuple[float, float, float, float]] = []
        self._son_scan_t: float | None = None
        self._son_scan_duvar_t: float | None = None
        self._son_gecerli_pose_t: float | None = None
        self._son_karar: str = 'serbest'
        self._son_olcek: float = 1.0
        self._son_gerekce: str = 'serbest'
        self._onceki_gerekce: str | None = None
        self._son_kritik_iz: Iz | None = None
        self._zaman_asimi_uyarildi: bool = False
        self._tarama_yok_uyarildi: bool = False
        self._odom_yok_uyarildi: bool = False

        # Yayıncılar
        self.p_cmd = self.create_publisher(Twist, cmd_cikis, 10)
        self.p_durum = self.create_publisher(String, '/hareketli_kalkan/durum', 10)

        self._marker_yayinla = bool(MARKER_VAR and MarkerArray is not None)
        if self._marker_yayinla:
            self.p_marker = self.create_publisher(MarkerArray, '/hareketli_kalkan/izler', 10)

        # Abonelikler
        self.create_subscription(
            LaserScan,
            scan_konu,
            self._scan_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            odom_konu,
            self._odom_cb,
            50,
        )
        self.create_subscription(
            Twist,
            cmd_giris,
            self._cmd_cb,
            10,
        )

        self.get_logger().info(
            f"HareketliKalkan başlatıldı: {cmd_giris} -> {cmd_cikis} (etkin={self.etkin})"
        )

    def _lidar_tf_oku(self, cerceve: str) -> None:
        """base_link <- lidar sabit dönüşümünü bir kez okur; bulunamazsa parametre değerleri kalır."""
        try:
            t = self.tf_buffer.lookup_transform(self.govde_cercevesi, cerceve, rclpy.time.Time())
        except Exception:
            return
        q = t.transform.rotation
        self.lidar_x = float(t.transform.translation.x)
        self.lidar_y = float(t.transform.translation.y)
        self.lidar_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._lidar_tf_alindi = True
        self.get_logger().info(
            f'Lidar montajı TF\'ten: x={self.lidar_x:.3f} y={self.lidar_y:.3f} yaw={math.degrees(self.lidar_yaw):.1f}°')

    def _simdi_s(self) -> float:
        """Düğüm saatine göre geçerli saniyeyi döndürür."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg: Odometry) -> None:
        t_msg = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        now_s = self._simdi_s()
        t_odom = t_msg if t_msg > 0.0 else now_s

        pos = msg.pose.pose.position
        yaw = yaw_q(msg.pose.pose.orientation)
        self._robot_pose_odom = (float(pos.x), float(pos.y), float(yaw))
        self._robot_wz = float(msg.twist.twist.angular.z)

        # Gövde hızını odom çerçevesine çevir
        vx_r = float(msg.twist.twist.linear.x)
        vy_r = float(msg.twist.twist.linear.y)
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_odom = vx_r * c - vy_r * s
        vy_odom = vx_r * s + vy_r * c
        self._robot_v_odom = (vx_odom, vy_odom)

        # Simülasyon sıfırlaması (zamanın geriye gitmesi) koruması (B1.2)
        if self._odom_gecmisi and t_odom < self._odom_gecmisi[-1][0]:
            self._odom_gecmisi.clear()
            self.takipci.sifirla()
            self._son_gecerli_pose_t = None

        # Odom geçmişi tamponunu güncelle (son 1 saniye)
        self._odom_gecmisi.append((t_odom, float(pos.x), float(pos.y), float(yaw)))
        esik_t = t_odom - 1.0
        self._odom_gecmisi = [kayit for kayit in self._odom_gecmisi if kayit[0] >= esik_t]

    def _scan_cb(self, msg: LaserScan) -> None:
        t_msg = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        now_s = self._simdi_s()
        t_scan = t_msg if t_msg > 0.0 else now_s

        self._son_scan_t = t_scan

        if not self._lidar_tf_alindi and msg.header.frame_id:
            self._lidar_tf_oku(msg.header.frame_id)

        # Taramanın header.stamp anındaki robot pozunu odom geçmişinden doğrusal aradeğerle bul
        robot_pose = poz_aradeger(self._odom_gecmisi, t_scan)
        if robot_pose is None:
            # B2.2: Odom geçmişi boşaldı/eskidi (poz_aradeger None, > 0.5 s) ise
            # son karar KİLİTLİ kalmasın: izleri temizle, karar 'serbest',
            # /hareketli_kalkan/durum'a "ODOM_YOK" bir kez yayınla, uyar.
            if odom_zaman_asimi_kontrolu(
                now_s=now_s,
                robot_pose=robot_pose,
                son_gecerli_pose_t=self._son_gecerli_pose_t,
                odom_gecmisi_bos=(len(self._odom_gecmisi) == 0),
                odom_zaman_asimi=self.odom_zaman_asimi,
            ):
                self.takipci.sifirla()
                self._son_karar = 'serbest'
                self._son_olcek = 1.0
                self._son_gerekce = 'ODOM_YOK'
                self._son_kritik_iz = None

                if not self._odom_yok_uyarildi:
                    self._odom_yok_uyarildi = True
                    self.get_logger().warn(
                        "Odom geçmişi boş veya eski (>0.5s); izler temizlendi, karar 'serbest' (ODOM_YOK)."
                    )
                    durum_msg = String()
                    durum_msg.data = "ODOM_YOK"
                    self.p_durum.publish(durum_msg)
            return

        self._son_gecerli_pose_t = now_s
        self._son_scan_duvar_t = now_s
        self._odom_yok_uyarildi = False

        pts = tarama_noktalari(
            ranges=msg.ranges,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            max_menzil=self.max_menzil,
            lidar_x=self.lidar_x,
            lidar_y=self.lidar_y,
            lidar_yaw=self.lidar_yaw,
        )

        if pts.shape[0] == 0:
            merkezler_base: list[tuple[float, float]] = []
            yaricaplar: list[float] = []
        else:
            merkezler_base, yaricaplar = kumele(
                pts,
                esik=self.kume_esik,
                min_nokta=self.kume_min_nokta,
            )
            secili = [i for i, r in enumerate(yaricaplar) if r <= self.max_iz_yaricap]
            merkezler_base = [merkezler_base[i] for i in secili]
            yaricaplar = [yaricaplar[i] for i in secili]

        merkezler_odom = kume_merkezleri_odoma(merkezler_base, robot_pose)
        self.takipci.guncelle(
            merkezler_odom,
            yaricaplar,
            t_scan,
            robot_wz=self._robot_wz,
        )

        karar_sonuc = kalkan_karari(
            izler=self.takipci.izler(),
            robot_v_odom=self._robot_v_odom,
            robot_p_odom=robot_pose[:2],
            robot_yaricap=self.robot_yaricap,
            min_engel_hizi=self.min_engel_hizi,
            min_gorulme=self.min_gorulme,
            min_iz_yasi_s=self.min_iz_yasi_s,
            yon_tutarlilik=self.yon_tutarlilik,
            ttc_dur=self.ttc_dur,
            ttc_yavas=self.ttc_yavas,
        )

        self._son_karar = karar_sonuc.karar
        self._son_olcek = karar_sonuc.olcek
        self._son_gerekce = karar_sonuc.gerekce
        self._son_kritik_iz = karar_sonuc.kritik_iz

        if self._son_gerekce != self._onceki_gerekce:
            self._onceki_gerekce = self._son_gerekce
            durum_msg = String()
            durum_msg.data = self._son_gerekce
            self.p_durum.publish(durum_msg)

            if self._son_karar == 'dur':
                self.get_logger().warn(f"Kalkan TETİKLENDİ: {self._son_gerekce}")
            elif self._son_karar == 'yavas':
                self.get_logger().info(f"Kalkan YAVAŞLATIYOR: {self._son_gerekce}")

        if self._marker_yayinla and hasattr(self, 'p_marker'):
            self._yayinla_markerlar(msg.header.stamp)

    def _yayinla_markerlar(self, stamp: Any) -> None:
        if not MARKER_VAR or Marker is None or MarkerArray is None:
            return
        try:
            ma = MarkerArray()
            for iz in self.takipci.izler():
                m = Marker()
                m.header.frame_id = 'odom'
                m.header.stamp = stamp
                m.ns = 'hareketli_izler'
                m.id = int(iz.id)
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = float(iz.x)
                m.pose.position.y = float(iz.y)
                m.pose.position.z = 0.2
                m.scale.x = max(0.1, float(iz.r) * 2.0)
                m.scale.y = max(0.1, float(iz.r) * 2.0)
                m.scale.z = 0.4

                v = math.hypot(iz.vx, iz.vy)
                if v >= self.min_engel_hizi and iz.gorulme >= self.min_gorulme:
                    m.color.r = 1.0
                    m.color.g = 0.1
                    m.color.b = 0.1
                    m.color.a = 0.8
                else:
                    m.color.r = 0.2
                    m.color.g = 0.6
                    m.color.b = 1.0
                    m.color.a = 0.4
                m.lifetime.sec = 1
                ma.markers.append(m)
            self.p_marker.publish(ma)
        except Exception:
            pass

    def _cmd_cb(self, msg: Twist) -> None:
        # 1. Kalkan devre dışıysa doğrudan geçir
        if not self.etkin:
            self.p_cmd.publish(msg)
            return

        # 2. Tarama zaman aşımı denetimi (B2.3)
        now_s = self._simdi_s()
        eylem, yeni_karar, yeni_olcek = tarama_zaman_asimi_yonetimi(
            now_s=now_s,
            son_scan_duvar_t=self._son_scan_duvar_t,
            son_karar=self._son_karar,
            tarama_zaman_asimi=self.tarama_zaman_asimi,
            eski_tarama_tutma_s=self.eski_tarama_tutma_s,
        )

        if eylem == 'dur_tut':
            # B2.3: son karar 'dur' ise eski_tarama_tutma_s (vars. 0.5) boyunca dur'u tut
            engel_yon_x = 1.0
            if self._son_kritik_iz is not None and self._robot_pose_odom is not None:
                rx, ry, ryaw = self._robot_pose_odom
                dx = self._son_kritik_iz.x - rx
                dy = self._son_kritik_iz.y - ry
                c = math.cos(ryaw)
                s = math.sin(ryaw)
                x_base = c * dx + s * dy
                engel_yon_x = 1.0 if x_base >= 0.0 else -1.0

            vx_cikis, wz_cikis = komutu_olcekle(
                vx=msg.linear.x,
                wz=msg.angular.z,
                karar='dur',
                olcek=0.0,
                engel_yon_x=engel_yon_x,
            )
            out = Twist()
            out.linear.x = vx_cikis
            out.linear.y = 0.0
            out.linear.z = msg.linear.z
            out.angular.x = msg.angular.x
            out.angular.y = msg.angular.y
            out.angular.z = wz_cikis
            self.p_cmd.publish(out)
            return

        if eylem == 'serbest_birak':
            # eski_tarama_tutma_s doldu veya son karar dur değildi: serbest + "TARAMA_YOK"
            self.takipci.sifirla()
            self._son_karar = 'serbest'
            self._son_olcek = 1.0
            self._son_gerekce = 'TARAMA_YOK'
            self._son_kritik_iz = None

            if not self._tarama_yok_uyarildi:
                self.get_logger().warn(
                    f"Tarama verisi eski (>{self.tarama_zaman_asimi}s) veya yok, "
                    "hareketli kalkan serbest bırakılıyor (TARAMA_YOK)."
                )
                self._tarama_yok_uyarildi = True
                durum_msg = String()
                durum_msg.data = "TARAMA_YOK"
                self.p_durum.publish(durum_msg)

            self.p_cmd.publish(msg)
            return

        self._tarama_yok_uyarildi = False

        # 3. Engelin robot çerçevesindeki yönünü hesapla
        engel_yon_x = 1.0
        if self._son_kritik_iz is not None and self._robot_pose_odom is not None:
            rx, ry, ryaw = self._robot_pose_odom
            dx = self._son_kritik_iz.x - rx
            dy = self._son_kritik_iz.y - ry
            c = math.cos(ryaw)
            s = math.sin(ryaw)
            # base_link x ekseninde engelin konumu
            x_base = c * dx + s * dy
            engel_yon_x = 1.0 if x_base >= 0.0 else -1.0

        # 4. Hız komutunu ölçekle
        vx_cikis, wz_cikis = komutu_olcekle(
            vx=msg.linear.x,
            wz=msg.angular.z,
            karar=self._son_karar,
            olcek=self._son_olcek,
            engel_yon_x=engel_yon_x,
        )

        out = Twist()
        out.linear.x = vx_cikis
        out.linear.y = msg.linear.y * (self._son_olcek if self._son_karar != 'serbest' else 1.0)
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = wz_cikis
        self.p_cmd.publish(out)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = HareketliKalkan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zincirin ortasındayız: kapanırken son komut asılı kalmasın diye sıfır hız (R1 B2.1; launch respawn eder)
        try:
            if rclpy.ok():
                node.p_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
