"""Kavrama için yaklaşma pozu: nesnenin haritadaki konumundan, myCobot'un erişebileceği bir noktaya ve belirli açıyla
varış (ROS'suz; testli). "elmayı al" -> nesne konumu (VLMaps sorgusu / algı) -> bu modül -> Nav2 + düz son yaklaşma.

Geometri (base_footprint, x ileri): kol nesneyi robotun önünde, gövde ekseninde (yanal ofset `yanal`) `d_kavrama`
uzaklıkta kavrar. kol_erisim.py (URDF FK, 27.10.2026): kavrama noktası robot merkezinin 0.18-0.30 m önünde (zemin-0.6 m
yan, 0.45 m'ye kadar üstten); varsayılan 0.22. Yaklaşma yönü θ: robotun varışta baktığı yön (nesneye doğru).
  P(θ)  kavrama pozu      = nesne - d_kavrama·u(θ) - yanal·n(θ)     (u = (cos θ, sin θ), n = sol normal)
  Q(θ)  kavrama öncesi    = P - d_on·u(θ)                            Nav2 hedefi (aynı θ)
  Q -> P düz, yavaş son yaklaşma (diff: yerinde dönüp hizalanır, sonra düz) -> varış açısı Nav2 toleransından bağımsız
Bir θ geçerli: Q'da ve Q->P boyunca ayak izi (pay ile) engelsiz; P'de engel yalnız nesnenin kendi dairesi (r_nesne)
içinde olabilir (nesneye yaklaşmak zorundayız). Puan: P'deki engel açıklığı (fazlası iyi, doyumlu) - istenen yöne
sapma (--yon verilirse) - robotun mevcut konumundan uzaklık (küçük ağırlık).
"""

import math
from dataclasses import dataclass, field

import numpy as np

# LIMO ayak izi (diff profili, Nav2 ile aynı): x ±0.161, y ±0.11
AYAK_IZI = (-0.161, 0.161, -0.11, 0.11)


@dataclass
class Harita:
    dolu: np.ndarray            # bool (H, W), satır 0 = üst (y büyük); bilinmeyen dolu sayılır
    cozunurluk: float
    orijin: tuple               # (x, y) sol alt köşe

    @classmethod
    def yaml_oku(cls, yol):
        import os
        import cv2
        import yaml
        h = yaml.safe_load(open(yol))
        img = cv2.imread(os.path.join(os.path.dirname(os.path.abspath(yol)), h['image']), cv2.IMREAD_GRAYSCALE)
        p = (255 - img.astype(float)) / 255.0 if not h.get('negate', 0) else img / 255.0
        dolu = (p > h.get('free_thresh', 0.25))                     # dolu + bilinmeyen
        return cls(dolu, float(h['resolution']), (float(h['origin'][0]), float(h['origin'][1])))

    def hucre(self, x, y):
        u = np.floor((np.asarray(x) - self.orijin[0]) / self.cozunurluk).astype(int)
        v = self.dolu.shape[0] - 1 - np.floor((np.asarray(y) - self.orijin[1]) / self.cozunurluk).astype(int)
        return v, u

    def dolu_mu(self, x, y):
        v, u = self.hucre(x, y)
        ic = (v >= 0) & (v < self.dolu.shape[0]) & (u >= 0) & (u < self.dolu.shape[1])
        out = np.ones(np.shape(x), bool)                            # harita dışı = dolu
        out[ic] = self.dolu[v[ic], u[ic]]
        return out

    def dolu_noktalar(self):
        v, u = np.nonzero(self.dolu)
        return np.c_[self.orijin[0] + (u + 0.5) * self.cozunurluk,
                     self.orijin[1] + (self.dolu.shape[0] - v - 0.5) * self.cozunurluk]


@dataclass
class Ayar:
    d_kavrama: float = 0.22     # robot merkezi -> kavrama noktası (m), kol_erisim.py medyanı
    yanal: float = 0.0          # kavrama noktasının robot eksenine yanal ofseti (m, + sol)
    d_on: float = 0.35          # kavrama öncesi geri çekilme (m)
    pay: float = 0.05           # ayak izi güvenlik payı (m)
    r_nesne: float = 0.12       # nesne/destek dairesi: P'de bu daire içindeki engel yok sayılır (m)
    r_temas: float = 0.05       # kol-nesne görüş çizgisinde yok sayılan nesne çevresi (m); duvar dibi nesnede
                                # duvarın arkasından "duvar içinden" kavrama seçilmesin
    aci_adim: float = 5.0       # θ tarama adımı (derece)
    yon: float | None = None    # istenen yaklaşma yönü (rad, harita); None = serbest
    yon_tol: float = math.radians(45)   # istenen yönden en çok sapma
    acik_doyum: float = 0.30    # açıklık puanı bu değerde doyar (m)


@dataclass
class Yaklasim:
    theta: float
    kavrama: tuple              # P (x, y, θ)
    oncesi: tuple               # Q (x, y, θ)
    aciklik: float              # P'de ayak izinin nesne dışı en yakın engele uzaklığı (m)
    puan: float
    nesne_robotta: tuple = field(default=(0.0, 0.0))   # P'de nesnenin base_footprint'teki (x, y)'si


def ayak_izi_noktalari(x, y, th, pay, adim=0.02):
    """ayak izi dikdörtgeninin (pay ile büyütülmüş) içini örnekleyen noktalar (harita)"""
    x0, x1, y0, y1 = AYAK_IZI
    gx, gy = np.meshgrid(np.arange(x0 - pay, x1 + pay + 1e-9, adim), np.arange(y0 - pay, y1 + pay + 1e-9, adim))
    c, s = math.cos(th), math.sin(th)
    return x + c * gx.ravel() - s * gy.ravel(), y + s * gx.ravel() + c * gy.ravel()


def aday_degerlendir(h, engeller, agac, nesne, th, a, robot=None):
    ox, oy = nesne
    u, n = np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])
    P = np.array([ox, oy]) - a.d_kavrama * u - a.yanal * n
    Q = P - a.d_on * u
    # Q ve Q->P yolu: ayak izi tamamen engelsiz (pay ile); nesne dairesi dahil (son yaklaşmada nesneye çarpmasın)
    for t in np.linspace(0, 1, max(2, int(a.d_on / 0.03) + 1)):
        X = Q + t * (P - Q)
        if t == 1.0:
            break
        xs, ys = ayak_izi_noktalari(X[0], X[1], th, a.pay)
        if h.dolu_mu(xs, ys).any():
            return None
    # P: nesne dairesi dışındaki engeller ayak izine (pay ile) girmemeli
    xs, ys = ayak_izi_noktalari(P[0], P[1], th, a.pay)
    dis = np.hypot(xs - ox, ys - oy) > a.r_nesne
    if h.dolu_mu(xs[dis], ys[dis]).any():
        return None
    # kol -> nesne görüş çizgisi: gövde önünden nesneye engel yok (nesnenin r_temas çevresi hariç)
    on = P + AYAK_IZI[1] * u + a.yanal * n
    L = np.linalg.norm(np.array([ox, oy]) - on)
    ts = np.arange(0.0, max(L - a.r_temas, 0.0), h.cozunurluk / 2)
    if len(ts) and h.dolu_mu(on[0] + ts * (ox - on[0]) / max(L, 1e-9), on[1] + ts * (oy - on[1]) / max(L, 1e-9)).any():
        return None
    # açıklık: P ayak izi kenarından nesne dairesi dışındaki en yakın engele
    if agac is not None:
        x0, x1, y0, y1 = AYAK_IZI
        k = agac.query_ball_point(P, 1.0)
        E = engeller[k] if len(k) else np.zeros((0, 2))
        E = E[np.hypot(E[:, 0] - ox, E[:, 1] - oy) > a.r_nesne]
        if len(E):
            c, s = math.cos(th), math.sin(th)
            lx, ly = c * (E[:, 0] - P[0]) + s * (E[:, 1] - P[1]), -s * (E[:, 0] - P[0]) + c * (E[:, 1] - P[1])
            dx, dy = np.maximum(np.maximum(x0 - lx, lx - x1), 0), np.maximum(np.maximum(y0 - ly, ly - y1), 0)
            acik = float(np.hypot(dx, dy).min())
        else:
            acik = 1.0
    else:
        acik = 1.0
    puan = min(acik, a.acik_doyum) / a.acik_doyum
    if a.yon is not None:
        sap = abs(math.atan2(math.sin(th - a.yon), math.cos(th - a.yon)))
        if sap > a.yon_tol:
            return None
        puan -= 0.5 * sap / a.yon_tol
    if robot is not None:
        puan -= 0.05 * math.hypot(Q[0] - robot[0], Q[1] - robot[1])
    return Yaklasim(th, (float(P[0]), float(P[1]), th), (float(Q[0]), float(Q[1]), th), acik, puan,
                    (a.d_kavrama, a.yanal))


def erisilebilir_bolge(h, robot, yaricap=0.14):
    """robotun (x, y) bulunduğu, robot yarıçapı kadar daraltılmış serbest bölge (bool ızgara); robot hücresi doluysa
    None (kontrol yapılmaz)"""
    import cv2
    k = max(1, int(round(yaricap / h.cozunurluk)))
    serbest = cv2.erode((~h.dolu).astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
    _, etiket = cv2.connectedComponents(serbest, connectivity=8)
    v, u = h.hucre(robot[0], robot[1])
    if not (0 <= v < etiket.shape[0] and 0 <= u < etiket.shape[1]) or etiket[v, u] == 0:
        return None
    return etiket == etiket[v, u]


def yaklasim_bul(h, nesne, a=None, robot=None, hepsi=False):
    """nesne (x, y) harita çerçevesinde -> en iyi Yaklasim (ya da puana göre sıralı liste). Geçerli yoksa None/[]."""
    a = a or Ayar()
    try:
        from scipy.spatial import cKDTree
        engeller = h.dolu_noktalar()
        agac = cKDTree(engeller) if len(engeller) else None
    except ImportError:
        engeller, agac = None, None
    bolge = erisilebilir_bolge(h, robot) if robot is not None else None
    adaylar = []
    for th in np.radians(np.arange(-180, 180, a.aci_adim)):
        y = aday_degerlendir(h, engeller, agac, nesne, float(th), a, robot)
        if y and bolge is not None:
            v, u = h.hucre(y.oncesi[0], y.oncesi[1])
            if not (0 <= v < bolge.shape[0] and 0 <= u < bolge.shape[1] and bolge[v, u]):
                continue                                    # robotun bulunduğu serbest bölgeden ulaşılamıyor
        if y:
            adaylar.append(y)
    adaylar.sort(key=lambda y: -y.puan)
    if hepsi:
        return adaylar
    return adaylar[0] if adaylar else None
