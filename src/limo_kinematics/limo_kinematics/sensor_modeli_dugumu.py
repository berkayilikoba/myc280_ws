"""Simülasyon düğümü: Gazebo'nun kusursuz sensörlerini gerçek LIMO'nun ölçülmüş kusurlarıyla yeniden yayınlar.

Değerler gerçek kayıtlardan (saha/analiz/sim_param_cikar.py -> saha/config/sim_gercek_param.yaml, 27.10.2026, 5 kayıt):
  odom   /odom_gercek (planar_move, Gazebo gerçek konumu) -> /odom + TF odom->base_footprint. Gerçek tabanın odom'u
         dönüşü %0.46 fazla (k_w = Δyaw_gerçek/Δyaw_odom = 0.9954), mesafeyi %1.9 eksik (k_v = 1.0188) sayıyor.
         Bir kapsama turunda (31.5 rad dönüş) gerçekte -8..-10° yön kayması = bu ölçek hatası. Rastgele kısım küçük:
         ölçekten sonra tur sonu farkı ±2° / ~30 rad -> σ 0.006 rad/√rad (1 s pencerelerdeki 0.027'nin çoğu lidar
         pozunun kendi gürültüsü).
  imu    /imu_ham -> /imu: ivme ölçeği 1.024 + x/y sapması, hıza bağlı titreşim gürültüsü (gerçek gövde sürüşte
         titriyor), jiroskop 0.2°/s (0.00349 rad/s) adımlarla nicemlenir; durağan Gazebo gürültüsü (σ) URDF'te.
  lidar  /scan_ham -> /scan: rastgele %3.3 ölçüm kaybı (gerçek T-mini geçerli oranı %96.7).
  derinlik /camera/depth_sim/depth/image_raw (32FC1, m) -> /camera/depth/image_raw (16UC1, mm) + camera_info:
         σ(z) = a·z² (a = 0.00176 1/m, D435 848x480 sabit sahne ölçümü), geçerli aralık [z_min, z_max], derinlik
         süreksizliklerinde gölge şeridi (gerçek D435'te nesne kenarında geçersiz bant), zeminde yüksek kayıp
         (gerçekte %16-40 geçerli) ve leke halinde rastgele boşluk.
  renk   /camera/color_sim/image_raw (rgb8 veya bgr8) -> /camera/color/image_raw + camera_info:
         gerçek D435 renk bozulma modeli (hareket bulanıklığı, radyal vinyet, beyaz denge kazancı ve Poisson-Gauss gürültüsü).
odom_hatasi:=false ile /odom = gerçek konum (eski davranış; Nav2 ayar düzeneği için).
bolum: 'odom_imu', 'derinlik', 'renk' ya da 'hepsi'.
"""

import array
import math

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_ros import TransformBroadcaster

try:
    from limo_kinematics import renk_bozu
    from limo_kinematics.renk_bozu import renk_boz
except ImportError:
    import renk_bozu
    from renk_bozu import renk_boz

VARSAYILAN_RENK_PARAM = getattr(
    renk_bozu, "VARSAYILAN_PARAM", getattr(renk_bozu, "VARSAYILAN_PARAMETRELER", {})
)


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


PROFIL_ONAYAR = {
    'teker': {
        'k_w': 0.9954,
        'k_v': 1.0188,
        'sigma_w': 0.006,
        'vio_olcek_kayma': 0.0,
        'vio_yaw_kayma_deg_m': 0.0,
        'vio_konum_ry': 0.0,
        'vio_sicrama_hz': 0.0,
        'vio_sicrama_konum': 0.05,
        'vio_sicrama_yaw_deg': 2.0,
    },
    'vio': {
        'k_w': 1.0,
        'k_v': 1.0,
        'sigma_w': 0.0,
        'vio_olcek_kayma': 0.002,
        'vio_yaw_kayma_deg_m': 0.3,
        'vio_konum_ry': 0.005,
        'vio_sicrama_hz': 0.0,
        'vio_sicrama_konum': 0.05,
        'vio_sicrama_yaw_deg': 2.0,
    },
    'vio_kotu': {
        'k_w': 1.0,
        'k_v': 1.0,
        'sigma_w': 0.0,
        'vio_olcek_kayma': 0.005,
        'vio_yaw_kayma_deg_m': 0.8,
        'vio_konum_ry': 0.01,
        'vio_sicrama_hz': 0.02,
        'vio_sicrama_konum': 0.05,
        'vio_sicrama_yaw_deg': 2.0,
    },
}


class OdomHatasi:
    """Gerçek konum artışlarını ölçülmüş ölçek hatası + gürültü / VIO hata profiliyle tümleştirir (saf, test edilebilir)."""

    def __init__(
        self,
        k_w=None,
        k_v=None,
        sigma_w=None,
        tohum=0,
        profil='teker',
        vio_olcek_kayma=None,
        vio_yaw_kayma_deg_m=None,
        vio_konum_ry=None,
        vio_sicrama_hz=None,
        vio_sicrama_konum=None,
        vio_sicrama_yaw_deg=None,
        vio_tohum=None,
    ):
        self.profil = str(profil).strip().lower()
        onayar = PROFIL_ONAYAR.get(self.profil, PROFIL_ONAYAR['teker'])

        self.k_w = float(k_w) if k_w is not None else float(onayar['k_w'])
        self.k_v = float(k_v) if k_v is not None else float(onayar['k_v'])
        self.sigma_w = float(sigma_w) if sigma_w is not None else float(onayar['sigma_w'])

        self.vio_olcek_kayma = (
            float(vio_olcek_kayma) if vio_olcek_kayma is not None else float(onayar['vio_olcek_kayma'])
        )
        self.vio_yaw_kayma_deg_m = (
            float(vio_yaw_kayma_deg_m) if vio_yaw_kayma_deg_m is not None else float(onayar['vio_yaw_kayma_deg_m'])
        )
        self.vio_konum_ry = (
            float(vio_konum_ry) if vio_konum_ry is not None else float(onayar['vio_konum_ry'])
        )
        self.vio_sicrama_hz = (
            float(vio_sicrama_hz) if vio_sicrama_hz is not None else float(onayar['vio_sicrama_hz'])
        )
        self.vio_sicrama_konum = (
            float(vio_sicrama_konum) if vio_sicrama_konum is not None else float(onayar['vio_sicrama_konum'])
        )
        self.vio_sicrama_yaw_deg = (
            float(vio_sicrama_yaw_deg) if vio_sicrama_yaw_deg is not None else float(onayar['vio_sicrama_yaw_deg'])
        )

        self.tohum = int(vio_tohum) if vio_tohum is not None else int(tohum)
        self.rng = np.random.default_rng(self.tohum)

        self.onceki = None
        self._onceki_t = None
        self.x = self.y = self.yaw = 0.0
        self.olcek_hata = 0.0
        self.sicrama_sayisi = 0

    def hiz_olcekle(self, vx, wz):
        """Twist (hız) alanına ölçek hatasını uygular; sıçramayı hıza yansıtmaz."""
        olcek_carpani = (1.0 + self.olcek_hata) / self.k_v
        return vx * olcek_carpani, wz / self.k_w

    def adim(self, x, y, yaw, dt=None, t=None):
        if self.onceki is None:
            self.onceki = (x, y, yaw)
            if t is not None:
                self._onceki_t = float(t)
            return self.x, self.y, self.yaw
        px, py, pyaw = self.onceki
        self.onceki = (x, y, yaw)
        dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
        c, s = math.cos(pyaw), math.sin(pyaw)
        ileri = c * (x - px) + s * (y - py)
        yanal = -s * (x - px) + c * (y - py)

        # 'teker' profilinde ve VIO parametreleri kapalıyken eski çıktı ve RNG birebir aynı
        vio_aktif = (self.profil != 'teker') or (
            self.vio_olcek_kayma > 0
            or self.vio_yaw_kayma_deg_m > 0
            or self.vio_konum_ry > 0
            or self.vio_sicrama_hz > 0
        )

        if not vio_aktif:
            dyaw_o = dyaw / self.k_w + self.rng.normal(0.0, self.sigma_w * math.sqrt(abs(dyaw))) if dyaw else 0.0
            ileri, yanal = ileri / self.k_v, yanal / self.k_v
            orta = self.yaw + dyaw_o / 2
            self.x += math.cos(orta) * ileri - math.sin(orta) * yanal
            self.y += math.sin(orta) * ileri + math.cos(orta) * yanal
            self.yaw = math.atan2(math.sin(self.yaw + dyaw_o), math.cos(self.yaw + dyaw_o))
            return self.x, self.y, self.yaw

        ds = math.hypot(ileri, yanal)
        dtheta = abs(dyaw)

        # 1. Ölçek kayması rastgele yürüyüşü (mesafeyle σ)
        if ds > 0 and self.vio_olcek_kayma > 0:
            self.olcek_hata += self.rng.normal(0.0, self.vio_olcek_kayma * math.sqrt(ds))

        olcek_carpani = (1.0 + self.olcek_hata) / self.k_v
        ileri_o = ileri * olcek_carpani
        yanal_o = yanal * olcek_carpani

        # 2. Konum rastgele yürüyüşü (m/√m)
        if ds > 0 and self.vio_konum_ry > 0:
            sg_ry = self.vio_konum_ry * math.sqrt(ds)
            ileri_o += self.rng.normal(0.0, sg_ry)
            yanal_o += self.rng.normal(0.0, sg_ry)

        # 3. Yaw artışı ve kayması
        dyaw_o = dyaw / self.k_w
        if dyaw and self.sigma_w > 0:
            dyaw_o += self.rng.normal(0.0, self.sigma_w * math.sqrt(dtheta))
        if ds > 0 and self.vio_yaw_kayma_deg_m > 0:
            sg_yaw = math.radians(self.vio_yaw_kayma_deg_m) * math.sqrt(ds)
            dyaw_o += self.rng.normal(0.0, sg_yaw)

        # 4. Poz entegrasyonu
        orta = self.yaw + dyaw_o / 2
        self.x += math.cos(orta) * ileri_o - math.sin(orta) * yanal_o
        self.y += math.sin(orta) * ileri_o + math.cos(orta) * yanal_o
        self.yaw = math.atan2(math.sin(self.yaw + dyaw_o), math.cos(self.yaw + dyaw_o))

        # 5. Sıçrama (dt süresine göre tek seferlik kayma — Poisson süreci)
        dt_val = 0.0
        if dt is not None:
            dt_val = float(dt)
        elif t is not None:
            t_f = float(t)
            if self._onceki_t is not None and t_f > self._onceki_t:
                dt_val = t_f - self._onceki_t
            self._onceki_t = t_f

        if self.vio_sicrama_hz > 0 and dt_val > 0:
            p_sicrama = 1.0 - math.exp(-self.vio_sicrama_hz * dt_val)
            if self.rng.random() < p_sicrama:
                jx = self.rng.normal(0.0, self.vio_sicrama_konum)
                jy = self.rng.normal(0.0, self.vio_sicrama_konum)
                jyaw = self.rng.normal(0.0, math.radians(self.vio_sicrama_yaw_deg))
                self.x += jx
                self.y += jy
                self.yaw = math.atan2(math.sin(self.yaw + jyaw), math.cos(self.yaw + jyaw))
                self.sicrama_sayisi += 1

        return self.x, self.y, self.yaw


class VioHatasi(OdomHatasi):
    """VIO hata profilini OdomHatasi üzerine uygulayan sarmalayıcı sınıf."""

    def __init__(self, *args, profil='vio', **kwargs):
        super().__init__(*args, profil=profil, **kwargs)


def normaller(z, K):
    """derinlik görüntüsünden birim yüzey normalleri (kamera optik çerçevesi, (H, W, 3)) ve 3B noktalar"""
    H, W = z.shape
    u = (np.arange(W, dtype=np.float32) - K[2]) / K[0]
    v = (np.arange(H, dtype=np.float32) - K[5]) / K[4]
    P = np.stack([z * u[None, :], z * v[:, None], z], -1)
    du, dv = np.zeros_like(P), np.zeros_like(P)
    du[:, 1:-1] = P[:, 2:] - P[:, :-2]
    dv[1:-1] = P[2:] - P[:-2]
    n = np.cross(du, dv)
    return n / np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), 1e-12), P


def gelis_acisi(z, K):
    """yüzey normali ile görüş ışını arasındaki açı (derece; 0 = dik bakış, 90 = sıyırma)"""
    n, P = normaller(z, K)
    c = np.abs(np.sum(n * P, -1)) / np.maximum(np.linalg.norm(P, axis=-1), 1e-12)
    return np.degrees(np.arccos(np.clip(c, 0, 1)))


# Gerçek D435 zemin geçerliliği (v2a sahnesi, 27.10.2026): zemin 0-0.5 m %40, 0.5-4 m %16-25. Kayıp açıdan değil
# zeminin kendisinden (sıyırma açısındaki yan duvar gerçekte geçerli) -> yalnız zemin (normali düşey) pikseller.
ZEMIN_P = ((0.0, 0.40), (0.5, 0.40), (0.75, 0.21), (8.0, 0.20))
ZEMIN_COS = 0.9          # |normal · düşey| > bu -> yatay yüzey (kamera eğimi ~1°)


def derinlik_bozu(z, a=0.00176, z_min=0.28, z_max=10.0, golge_esik=0.08, bosluk=0.02, rng=None, K=None,
                  leke=8):
    """z: (H, W) float32 metre (NaN/inf = yok). Dönüş: uint16 mm, gerçek D435 benzeri gürültü ve boşluklarla.
    K (camera_info k, 9) verilirse zemin (yatay yüzey) boşluğu (ZEMIN_P); boşluklar `leke` px'lik lekeler halinde
    (gerçekte tek piksel değil bölge olarak kaybolur)."""
    rng = rng or np.random.default_rng()
    z = np.where(np.isfinite(z), z, 0.0).astype(np.float32)
    gec = (z >= z_min) & (z <= z_max)
    H, W = z.shape
    if leke > 1:   # düşük çözünürlüklü düzgün rastgele alan -> büyüt: komşu pikseller aynı kaderi paylaşır
        alan = rng.random((H // leke + 2, W // leke + 2)).astype(np.float32)
        alan = np.repeat(np.repeat(alan, leke, 0), leke, 1)[:H, :W]
    else:
        alan = rng.random(z.shape).astype(np.float32)
    p = np.full(z.shape, 1.0 - bosluk, np.float32)
    if K is not None:   # yatay yüzeyler (zemin; optik çerçevede düşey = y ekseni)
        n, _ = normaller(z, K)
        yatay = np.abs(n[..., 1]) > ZEMIN_COS
        pz = np.interp(z, [q[0] for q in ZEMIN_P], [q[1] for q in ZEMIN_P]).astype(np.float32)
        p = np.where(yatay, np.minimum(p, pz), p)
    gec &= alan < p
    zn = z + rng.normal(0.0, 1.0, z.shape).astype(np.float32) * (a * z * z)
    # gölge: yatay komşuyla bağıl fark > golge_esik olan kenarın uzak tarafı (D435 sol IR - projektör paralaksı)
    if golge_esik > 0:
        fark = np.zeros_like(z)
        fark[:, 1:] = (z[:, 1:] - z[:, :-1]) / np.maximum(z[:, 1:], 1e-3)
        kenar = fark > golge_esik                                   # sağdaki piksel daha uzak
        golge = kenar.copy()
        for k in range(1, 4):                                       # ~3 piksellik bant
            golge[:, k:] |= kenar[:, :-k]
        gec &= ~golge
    return np.where(gec, np.clip(np.round(zn * 1000.0), 0, 65535), 0).astype(np.uint16)


# Gerçek IMU (27.10.2026, 3 kayıt): durağan ivme ortalaması (-0.13, -0.09, 10.045) -> ölçek 1.024 (|a| 10.045 / 9.81)
# + x/y sapması; hareketteki ek gürültü (1 s ortalamadan sapma σ, durağan gürültünün üstüne):
#   düz 0.15 m/s : jiro (0.008, 0.008, 0.010) rad/s, ivme (0.23, 0.11, 0.11) m/s²
#   dönüş 0.45 rad/s: jiro (0.007, 0.006, 0.065), ivme (0.22, 0.27, 0.075)
IVME_OLCEK, IVME_SAPMA = 1.024, (-0.13, -0.09, 0.0)
TITRESIM = {'duz': (np.array([0.008, 0.008, 0.010]), np.array([0.23, 0.11, 0.11]), 0.15),
            'donus': (np.array([0.007, 0.006, 0.065]), np.array([0.22, 0.27, 0.075]), 0.45)}


def imu_titresim_sigma(v, w):
    """hız/dönüşe göre ek gürültü σ (jiro, ivme): her durum katkısı hızla orantılı (doyumlu), kareler toplamı"""
    fv, fw = min(abs(v) / TITRESIM['duz'][2], 1.0), min(abs(w) / TITRESIM['donus'][2], 1.0)
    g = np.sqrt((fv * TITRESIM['duz'][0]) ** 2 + (fw * TITRESIM['donus'][0]) ** 2)
    a = np.sqrt((fv * TITRESIM['duz'][1]) ** 2 + (fw * TITRESIM['donus'][1]) ** 2)
    return g, a


def renk_mesaj_boz(
    veri: np.ndarray,
    encoding: str,
    param=None,
    w_z: float = 0.0,
    v_x: float = 0.0,
    rng=None,
) -> np.ndarray:
    """ROS bağımsız renk bozulma yardımcı fonksiyonu (RGB/BGR bağımsız)."""
    if not isinstance(veri, np.ndarray):
        veri = np.asarray(veri)
    if veri.size == 0:
        return veri.astype(np.uint8, copy=True)
    if veri.dtype != np.uint8:
        veri = np.clip(np.round(veri), 0, 255).astype(np.uint8)
    if not veri.flags['C_CONTIGUOUS']:
        veri = np.ascontiguousarray(veri)

    w_z_val = float(w_z) if w_z is not None else 0.0
    v_x_val = float(v_x) if v_x is not None else 0.0

    enc = str(encoding).strip().lower()
    if enc in ('rgb8', 'rgb') and veri.ndim == 3 and veri.shape[2] == 3:
        bgr = cv2.cvtColor(veri, cv2.COLOR_RGB2BGR)
        bozuk_bgr = renk_boz(bgr, param=param, w_z=w_z_val, v_x=v_x_val, rng=rng)
        return cv2.cvtColor(bozuk_bgr, cv2.COLOR_BGR2RGB)
    else:
        return renk_boz(veri, param=param, w_z=w_z_val, v_x=v_x_val, rng=rng)


class SensorModeliDugumu(Node):

    def __init__(self):
        super().__init__('sensor_modeli')
        p = self.declare_parameter
        bolum = p('bolum', 'hepsi').value
        oi = bolum in ('hepsi', 'odom_imu')
        dr = bolum in ('hepsi', 'derinlik')
        rk = bolum in ('hepsi', 'renk')
        self.odom_hatasi = p('odom_hatasi', True).value
        self.odom_profil = str(p('odom_profil', 'teker').value).strip().lower()
        if self.odom_profil not in PROFIL_ONAYAR:
            self.odom_profil = 'teker'
        onayar = PROFIL_ONAYAR[self.odom_profil]

        k_w = p('k_w', onayar['k_w']).value
        k_v = p('k_v', onayar['k_v']).value
        sigma_w = p('sigma_w', onayar['sigma_w']).value
        vio_olcek_kayma = p('vio_olcek_kayma', onayar['vio_olcek_kayma']).value
        vio_yaw_kayma_deg_m = p('vio_yaw_kayma_deg_m', onayar['vio_yaw_kayma_deg_m']).value
        vio_konum_ry = p('vio_konum_ry', onayar['vio_konum_ry']).value
        vio_sicrama_hz = p('vio_sicrama_hz', onayar['vio_sicrama_hz']).value
        vio_sicrama_konum = p('vio_sicrama_konum', onayar['vio_sicrama_konum']).value
        vio_sicrama_yaw_deg = p('vio_sicrama_yaw_deg', onayar['vio_sicrama_yaw_deg']).value
        vio_tohum = p('vio_tohum', 0).value

        self.oh = OdomHatasi(
            k_w=k_w,
            k_v=k_v,
            sigma_w=sigma_w,
            tohum=vio_tohum,
            profil=self.odom_profil,
            vio_olcek_kayma=vio_olcek_kayma,
            vio_yaw_kayma_deg_m=vio_yaw_kayma_deg_m,
            vio_konum_ry=vio_konum_ry,
            vio_sicrama_hz=vio_sicrama_hz,
            vio_sicrama_konum=vio_sicrama_konum,
            vio_sicrama_yaw_deg=vio_sicrama_yaw_deg,
            vio_tohum=vio_tohum,
        )
        self._son_odom_t = None
        self.nicem = p('jiroskop_nicem', 0.00349).value
        self.titresim = p('imu_titresim', True).value
        self.lidar_kayip = p('lidar_kayip', 0.033).value      # gerçek T-mini: geçerli ölçüm %96.7 (sim_real_fark)
        self.hiz = (0.0, 0.0)                                  # gerçek gövde v, w (titreşim için)
        self.dK = None
        self.dp = dict(a=p('derinlik_a', 0.00176).value, z_min=p('derinlik_z_min', 0.28).value,
                       z_max=p('derinlik_z_max', 10.0).value, golge_esik=p('derinlik_golge_esik', 0.08).value,
                       bosluk=p('derinlik_bosluk', 0.02).value)
        self.rng = np.random.default_rng(1)

        self.renk_bozu_aktif = p('renk_bozu', True).value
        self.renk_param_dosyasi = p('renk_param_dosyasi', '').value
        self.renk_tohum = p('renk_tohum', 2).value
        self.renk_rng = np.random.default_rng(self.renk_tohum)
        self.renk_param = dict(VARSAYILAN_RENK_PARAM)
        if self.renk_param_dosyasi:
            try:
                with open(self.renk_param_dosyasi, 'r', encoding='utf-8') as f:
                    yuklenen = yaml.safe_load(f)
                    if isinstance(yuklenen, dict):
                        if 'renk' in yuklenen and isinstance(yuklenen['renk'], dict):
                            self.renk_param.update(yuklenen['renk'])
                        else:
                            self.renk_param.update(yuklenen)
            except Exception as e:
                self.get_logger().error(f"Renk parametre dosyası okunamadı ({self.renk_param_dosyasi}): {e}")

        if oi:
            self.tf = TransformBroadcaster(self)
            self.p_odom = self.create_publisher(Odometry, 'odom', 20)
            self.p_imu = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom_gercek', self._odom, 50)
            self.create_subscription(Imu, 'imu_ham', self._imu, qos_profile_sensor_data)
            self.p_scan = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
            self.create_subscription(LaserScan, 'scan_ham', self._scan, qos_profile_sensor_data)
            self.get_logger().info(
                f'odom/IMU/lidar modeli: profil={self.oh.profil} odom_hatasi={self.odom_hatasi} '
                f'k_w={self.oh.k_w} k_v={self.oh.k_v} σw={self.oh.sigma_w}; '
                f'vio_olcek_kayma={self.oh.vio_olcek_kayma} vio_yaw_kayma_deg_m={self.oh.vio_yaw_kayma_deg_m} '
                f'vio_konum_ry={self.oh.vio_konum_ry} vio_sicrama_hz={self.oh.vio_sicrama_hz} '
                f'vio_sicrama_konum={self.oh.vio_sicrama_konum} vio_sicrama_yaw_deg={self.oh.vio_sicrama_yaw_deg}; '
                f'jiroskop nicem {self.nicem}; IMU titreşim {self.titresim}, ivme ölçek {IVME_OLCEK}; lidar kayıp {self.lidar_kayip}'
            )
        elif rk:
            self.create_subscription(Odometry, '/odom_gercek', self._odom_hiz, 50)

        if dr:
            self.p_d = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
            self.p_di = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
            self.create_subscription(Image, '/camera/depth_sim/depth/image_raw', self._derinlik, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '/camera/depth_sim/depth/camera_info', self._dinfo,
                                     qos_profile_sensor_data)
            self.get_logger().info(f'derinlik modeli: {self.dp}')

        if rk:
            self.p_c = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
            self.p_ci = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data)
            self.create_subscription(Image, '/camera/color_sim/image_raw', self._renk, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '/camera/color_sim/camera_info', self._cinfo,
                                     qos_profile_sensor_data)
            self.get_logger().info(f'renk modeli: renk_bozu={self.renk_bozu_aktif}, tohum={self.renk_tohum}, '
                                   f'dosya={self.renk_param_dosyasi or "varsayilan"}, param={self.renk_param}')

    def _odom(self, m):
        self.hiz = (m.twist.twist.linear.x, m.twist.twist.angular.z)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if self.odom_hatasi:
            sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            dt = 0.0
            if self._son_odom_t is not None:
                fark = sim_t - self._son_odom_t
                if 0.0 < fark < 1.0:
                    dt = fark
            self._son_odom_t = sim_t

            x, y, yaw = self.oh.adim(p.x, p.y, yaw_q(q), dt=dt)
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.pose.orientation.x = m.pose.pose.orientation.y = 0.0
            vx, wz = self.oh.hiz_olcekle(m.twist.twist.linear.x, m.twist.twist.angular.z)
            m.twist.twist.linear.x = vx
            m.twist.twist.angular.z = wz
        self.p_odom.publish(m)
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = m.child_frame_id
        t.transform.translation.x, t.transform.translation.y = m.pose.pose.position.x, m.pose.pose.position.y
        t.transform.translation.z = m.pose.pose.position.z
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)

    def _imu(self, m):
        la, av = m.linear_acceleration, m.angular_velocity
        if self.titresim:
            sg, sa = imu_titresim_sigma(*self.hiz)
            dg, da = self.rng.normal(0.0, 1.0, 3) * sg, self.rng.normal(0.0, 1.0, 3) * sa
            av.x, av.y, av.z = av.x + dg[0], av.y + dg[1], av.z + dg[2]
            la.x, la.y, la.z = la.x + da[0], la.y + da[1], la.z + da[2]
        la.x, la.y, la.z = (IVME_OLCEK * la.x + IVME_SAPMA[0], IVME_OLCEK * la.y + IVME_SAPMA[1],
                            IVME_OLCEK * la.z + IVME_SAPMA[2])
        if self.nicem > 0:
            av = m.angular_velocity
            av.x, av.y, av.z = (round(v / self.nicem) * self.nicem for v in (av.x, av.y, av.z))
        self.p_imu.publish(m)

    def _scan(self, m):
        if self.lidar_kayip > 0:
            r = np.array(m.ranges, np.float32)
            r[self.rng.random(len(r)) < self.lidar_kayip] = np.inf
            m.ranges = r.tolist()
        self.p_scan.publish(m)

    def _derinlik(self, m):
        if m.encoding != '32FC1':
            return
        z = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        out = Image()
        out.header, out.height, out.width = m.header, m.height, m.width
        out.encoding, out.is_bigendian, out.step = '16UC1', 0, m.width * 2
        # array.array: rclpy bytes atamasını bayt bayt doğruluyor (2.7 MB: 146 ms -> 2 ms)
        out.data = array.array('B', derinlik_bozu(z, rng=self.rng, K=self.dK, **self.dp).tobytes())
        self.p_d.publish(out)

    def _dinfo(self, m):
        self.dK = np.array(m.k, np.float32)
        self.p_di.publish(m)

    def _odom_hiz(self, m):
        self.hiz = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def _renk(self, m):
        if not self.renk_bozu_aktif:
            self.p_c.publish(m)
            return

        if m.encoding not in ('rgb8', 'bgr8'):
            self.p_c.publish(m)
            return

        try:
            if m.step == 0 or m.step == m.width * 3:
                veri = np.frombuffer(m.data, dtype=np.uint8).reshape((m.height, m.width, 3))
            else:
                veri = np.frombuffer(m.data, dtype=np.uint8).reshape((m.height, m.step))[:, :m.width * 3].reshape((m.height, m.width, 3))
        except Exception as e:
            self.get_logger().warn(f"Renk görüntüsü çözümlenemedi: {e}")
            self.p_c.publish(m)
            return

        v_x, w_z = self.hiz
        bozuk = renk_mesaj_boz(veri, m.encoding, self.renk_param, w_z=w_z, v_x=v_x, rng=self.renk_rng)

        out = Image()
        out.header = m.header
        out.height = m.height
        out.width = m.width
        out.encoding = m.encoding
        out.is_bigendian = m.is_bigendian
        out.step = m.width * 3
        out.data = array.array('B', bozuk.tobytes())
        self.p_c.publish(out)

    def _cinfo(self, m):
        self.p_ci.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = SensorModeliDugumu()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
