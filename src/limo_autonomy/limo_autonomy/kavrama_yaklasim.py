"""Kavrama için yaklaşma pozu: nesnenin haritadaki konumundan, myCobot'un erişebileceği bir noktaya ve belirli açıyla
varış (ROS'suz; testli). "elmayı al" -> nesne konumu (VLMaps sorgusu / algı) -> bu modül -> Nav2 + düz son yaklaşma.

Geometri (base_footprint, x ileri): kol nesneyi robotun önünde, gövde ekseninde (yanal ofset `yanal`) `d_kavrama`
uzaklıkta kavrar. kol_erisim.py (URDF FK, 27.10.2026): kavrama noktası robot merkezinin 0.18-0.30 m önünde (zemin-0.6 m
yan, 0.45 m'ye kadar üstten); varsayılan 0.22. Yaklaşma yönü θ: robotun varışta baktığı yön (nesneye doğru).
  P(θ)  kavrama pozu      = nesne - d_kavrama·u(θ) - yanal·n(θ)     (u = (cos θ, sin θ), n = sol normal)
  Q(θ)  kavrama öncesi    = P - d_on·u(θ)                            Nav2 hedefi (aynı θ)
  Q -> P düz, yavaş son yaklaşma (diff: yerinde dönüp hizalanır, sonra düz) -> varış açısı Nav2 toleransından bağımsız
Bir θ geçerli: Q'da ayak izi (pay + konum_belirsizligi) ve Q->P koridoru boyunca pay (pay + konum_belirsizligi -> pay_p + belirsizlik_p)
azalarak engelsiz; P'de engel (pay_p + belirsizlik_p ile) yalnız nesnenin kendi dairesi (r_nesne) içinde olabilir (nesneye yaklaşmak zorundayız). Puan: P ve Q açıklığı + yüzey normali uyumu
(PCA kenar tercihi) - istenen yöne sapma (--yon verilirse) - robotun mevcut konumundan uzaklık (küçük ağırlık).
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
    d_on: float | None = None   # None ise [d_on_min, d_on_max] aralığında dinamik; sayı verilirse sabit tutulur
    d_on_min: float = 0.30      # en küçük yaklaşma öncesi mesafe (m)
    d_on_max: float = 0.60      # en büyük yaklaşma öncesi mesafe (m)
    d_on_adim: float = 0.05     # d_on arama adımı (m)
    pay: float = 0.05           # ayak izi güvenlik payı (m)
    pay_p: float = 0.03         # P'deki ayak izi güvenlik payı (m); gerekçe: P'ye son 3 cm 3 cm/s sürünmeyle,
                                # nesneye göreli kapalı çevrimle ve lidar güvenlik durdurmasıyla varılıyor.
    konum_belirsizligi: float = 0.06  # koridor ek güvenlik payı (m)
    belirsizlik_p: float = 0.015 # P'deki belirsizlik payı (m)
    r_nesne: float = 0.12       # nesne/destek dairesi: P'de bu daire içindeki engel yok sayılır (m)
    r_temas: float = 0.05       # kol-nesne görüş çizgisinde yok sayılan nesne çevresi (m); duvar dibi nesnede
                                # duvarın arkasından "duvar içinden" kavrama seçilmesin
    aci_adim: float = 5.0       # θ tarama adımı (derece)
    yon: float | None = None    # istenen yaklaşma yönü (rad, harita); None = serbest
    yon_tol: float = math.radians(45)   # istenen yönden en çok sapma
    acik_doyum: float = 0.30    # açıklık puanı bu değerde doyar (m)
    yuzey_agirlik: float = 1.0  # kenar normali uyum ağırlığı
    aci_ayrik: float = math.radians(20)  # hepsi=True durumunda sonuçlar arası en küçük açı farkı (rad)


@dataclass
class Yaklasim:
    theta: float
    kavrama: tuple              # P (x, y, θ)
    oncesi: tuple               # Q (x, y, θ)
    aciklik: float              # P'de ayak izinin nesne dışı en yakın engele uzaklığı (m)
    puan: float
    nesne_robotta: tuple = field(default=(0.0, 0.0))   # P'de nesnenin base_footprint'teki (x, y)'si
    d_on: float = 0.35          # seçilen yaklaşma öncesi mesafe (m)


def ayak_izi_noktalari(x, y, th, pay, adim=0.02):
    """ayak izi dikdörtgeninin (pay ile büyütülmüş) içini örnekleyen noktalar (harita)"""
    x0, x1, y0, y1 = AYAK_IZI
    gx, gy = np.meshgrid(np.arange(x0 - pay, x1 + pay + 1e-9, adim), np.arange(y0 - pay, y1 + pay + 1e-9, adim))
    c, s = math.cos(th), math.sin(th)
    return x + c * gx.ravel() - s * gy.ravel(), y + s * gx.ravel() + c * gy.ravel()


def ayak_izi_aciklik(agac, engeller, pos, th, ox, oy, r_nesne):
    """Ayak izinin nesne dairesi dışındaki en yakın engele uzaklığı (m)."""
    if agac is None or engeller is None or len(engeller) == 0:
        return 1.0
    x0, x1, y0, y1 = AYAK_IZI
    k = agac.query_ball_point(pos, 1.0)
    if not len(k):
        return 1.0
    E = engeller[k]
    E = E[np.hypot(E[:, 0] - ox, E[:, 1] - oy) > r_nesne]
    if not len(E):
        return 1.0
    c, s = math.cos(th), math.sin(th)
    lx = c * (E[:, 0] - pos[0]) + s * (E[:, 1] - pos[1])
    ly = -s * (E[:, 0] - pos[0]) + c * (E[:, 1] - pos[1])
    dx = np.maximum(np.maximum(x0 - lx, lx - x1), 0)
    dy = np.maximum(np.maximum(y0 - ly, ly - y1), 0)
    return float(np.hypot(dx, dy).min())


def yuzey_normali_bul(engeller, agac, nesne, r_arama=0.30, r_nesne=0.12):
    """Nesne çevresindeki (r <= r_arama) dolu hücrelerden PCA ile baskın kenar doğrultusunu
    ve dış taraftan kenara dik yaklaşma açısını (rad) bulur. Kenar yoksa None döner."""
    if agac is None or engeller is None or len(engeller) == 0:
        return None
    ox, oy = nesne
    k = agac.query_ball_point((ox, oy), r_arama)
    if not len(k):
        return None
    E = engeller[k]
    d = np.hypot(E[:, 0] - ox, E[:, 1] - oy)
    noktalar = E[(d > r_nesne) & (d <= r_arama)]
    if len(noktalar) < 5:
        return None
    C = noktalar.mean(axis=0)
    X = noktalar - C
    U, s, Vt = np.linalg.svd(X)
    # Baskın kenar kontrolü: s[0] belirgin şekilde s[1]'den büyük olmalı
    if len(s) < 2 or s[0] < 1.5 * s[1] + 1e-5:
        return None
    v_norm = Vt[1]
    v_duvar = C - np.array([ox, oy])
    if np.dot(v_norm, v_duvar) < 0:
        v_norm = -v_norm
    return math.atan2(v_norm[1], v_norm[0])


def aday_incele(h, engeller, agac, nesne, th, a, robot=None, th_yuzey=None):
    """Tek bir aday açıyı değerlendirir; (Yaklasim veya None, elenme_nedeni, aciklik) döndürür."""
    ox, oy = nesne
    u = np.array([math.cos(th), math.sin(th)])
    n = np.array([-math.sin(th), math.cos(th)])
    P = np.array([ox, oy]) - a.d_kavrama * u - a.yanal * n

    # Açıklık: P'de ayak izinin nesne dışı en yakın engele uzaklığı
    acik = ayak_izi_aciklik(agac, engeller, P, th, ox, oy, a.r_nesne)

    # 1. P ayak izi kontrolü: nesne dairesi dışındaki engeller ayak izine (pay_p + belirsizlik_p) girmemeli.
    # Gerekçe: P'ye son 3 cm 3 cm/s sürünmeyle, nesneye göreli kapalı çevrimle ve lidar güvenlik durdurmasıyla varılır.
    esik_p = a.pay_p + a.belirsizlik_p
    xs, ys = ayak_izi_noktalari(P[0], P[1], th, esik_p)
    dis = np.hypot(xs - ox, ys - oy) > a.r_nesne
    if h.dolu_mu(xs[dis], ys[dis]).any():
        return None, 'P_carpisma', acik

    # Geçerlilik eşiği: P'de en yakın engele açıklık en az pay_p + belirsizlik_p olmalıdır
    if agac is not None and acik < esik_p - 1e-4:
        return None, 'P_carpisma', acik

    # 2. Kol -> nesne görüş çizgisi: gövde önünden nesneye engel yok (nesnenin r_temas çevresi hariç)
    on = P + AYAK_IZI[1] * u + a.yanal * n
    L = np.linalg.norm(np.array([ox, oy]) - on)
    ts = np.arange(0.0, max(L - a.r_temas, 0.0), h.cozunurluk / 2)
    if len(ts) and h.dolu_mu(on[0] + ts * (ox - on[0]) / max(L, 1e-9), on[1] + ts * (oy - on[1]) / max(L, 1e-9)).any():
        return None, 'erisim', acik

    # 3. İstenen yön sapma kontrolü
    if a.yon is not None:
        sap = abs(math.atan2(math.sin(th - a.yon), math.cos(th - a.yon)))
        if sap > a.yon_tol:
            return None, 'yon', acik

    # 4. Uyarlanabilir d_on ve Düz Koridor Koşulu:
    # d_on taraması [d_on_max -> d_on_min] sırasında koridor VE Q ayak izi (pay + konum_belirsizligi)
    # birlikte sağlanan EN BÜYÜK d_on seçilir. Q'dan P'ye ilerledikçe toplam pay:
    # (pay + konum_belirsizligi) -> (pay_p + belirsizlik_p) doğrusal geçiş yapar.
    if a.d_on is not None:
        d_on_adaylar = [a.d_on]
    else:
        adim = a.d_on_adim
        n_adim = int(round((a.d_on_max - a.d_on_min) / adim))
        d_on_adaylar = [round(a.d_on_max - i * adim, 4) for i in range(n_adim + 1)]

    secilen_d_on = None
    secilen_Q = None
    q_temiz_var = False

    pay_q = a.pay + a.konum_belirsizligi

    for d_val in d_on_adaylar:
        Q_val = P - d_val * u
        n_ad = max(2, int(d_val / 0.03) + 1)

        # Q noktasında ayak izi (pay + konum_belirsizligi) kontrolü
        xs_q, ys_q = ayak_izi_noktalari(Q_val[0], Q_val[1], th, pay_q)
        dis_q = np.hypot(xs_q - ox, ys_q - oy) > a.r_nesne
        if h.dolu_mu(xs_q[dis_q], ys_q[dis_q]).any():
            continue  # Bu d_val için Q pozunda çarpışma var

        q_temiz_var = True

        # Koridor boyunca diğer adımları denetle (t > 0'dan t = 1'e)
        koridor_temiz = True
        for t in np.linspace(0, 1, n_ad)[1:]:
            X = Q_val + t * (P - Q_val)
            koridor_pay_t = (1.0 - t) * pay_q + t * esik_p
            xs_k, ys_k = ayak_izi_noktalari(X[0], X[1], th, koridor_pay_t)
            dis_k = np.hypot(xs_k - ox, ys_k - oy) > a.r_nesne
            if h.dolu_mu(xs_k[dis_k], ys_k[dis_k]).any():
                koridor_temiz = False
                break
        if koridor_temiz:
            secilen_d_on = d_val
            secilen_Q = Q_val
            break

    if secilen_d_on is None:
        if not q_temiz_var:
            return None, 'Q_carpisma', acik
        return None, 'koridor', acik

    Q = secilen_Q

    # Açıklık: P ve Q'da ayak izinin nesne dışı en yakın engele uzaklığı (doyumlu)
    # P'de en yakın engele açıklık (aciklik) puanı aynı kalır
    acik_q = ayak_izi_aciklik(agac, engeller, Q, th, ox, oy, a.r_nesne)
    puan = min(acik, a.acik_doyum) / a.acik_doyum + min(acik_q, a.acik_doyum) / a.acik_doyum

    # Yüzey normali tercihi: kenara dik ve dış taraftan yaklaşma
    if th_yuzey is not None and a.yuzey_agirlik > 0:
        fark = math.atan2(math.sin(th - th_yuzey), math.cos(th - th_yuzey))
        puan += a.yuzey_agirlik * math.cos(fark)

    # İstenen yön sapma cezası
    if a.yon is not None:
        sap = abs(math.atan2(math.sin(th - a.yon), math.cos(th - a.yon)))
        puan -= 0.5 * sap / a.yon_tol

    # Robot konumundan uzaklık cezası
    if robot is not None:
        puan -= 0.05 * math.hypot(Q[0] - robot[0], Q[1] - robot[1])

    y = Yaklasim(th, (float(P[0]), float(P[1]), th), (float(Q[0]), float(Q[1]), th),
                 acik, puan, (a.d_kavrama, a.yanal), secilen_d_on)
    return y, None, acik


def aday_degerlendir(h, engeller, agac, nesne, th, a, robot=None, th_yuzey=None):
    y, _, _ = aday_incele(h, engeller, agac, nesne, th, a, robot=robot, th_yuzey=th_yuzey)
    return y


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
    th_yuzey = yuzey_normali_bul(engeller, agac, nesne, r_arama=0.30, r_nesne=a.r_nesne)
    bolge = erisilebilir_bolge(h, robot) if robot is not None else None
    adaylar = []
    for th in np.radians(np.arange(-180, 180, a.aci_adim)):
        y = aday_degerlendir(h, engeller, agac, nesne, float(th), a, robot, th_yuzey=th_yuzey)
        if y and bolge is not None:
            v, u = h.hucre(y.oncesi[0], y.oncesi[1])
            if not (0 <= v < bolge.shape[0] and 0 <= u < bolge.shape[1] and bolge[v, u]):
                continue                                    # robotun bulunduğu serbest bölgeden ulaşılamıyor
        if y:
            adaylar.append(y)
    adaylar.sort(key=lambda y: -y.puan)
    # Birbirine çok yakın açıları (< a.aci_ayrik, vars. 20°) tekrar etme (NMS)
    secilenler = []
    for y in adaylar:
        if any(abs(math.atan2(math.sin(y.theta - s.theta), math.cos(y.theta - s.theta))) < a.aci_ayrik for s in secilenler):
            continue
        secilenler.append(y)
    if hepsi:
        return secilenler
    return secilenler[0] if secilenler else None


def yaklasim_tani(h, nesne, a=None, robot=None) -> dict:
    """Aday açıların değerlendirme sonuçlarını ve elenme nedenlerini raporlar.

    yaklasim_bul None döndürdüğünde kavrama düğümünün hata ayıklama ve günlüğe
    yazma amacıyla kullanabileceği tanı sözlüğü üretir.

    Döndürür:
        dict:
          - 'P_carpisma': int (P'de ayak izi çarpışması sayısı)
          - 'koridor': int (Q->P koridor çarpışması sayısı)
          - 'Q_carpisma': int (Q pozu çarpışması sayısı)
          - 'erisim': int (kol görüş çizgisi veya robot serbest bölge erişilemezliği sayısı)
          - 'yuzey': int (yüzey normali uyumsuzluğu sayısı)
          - 'yon': int (istenen yönden sapma sayısı)
          - 'toplam_aday': int (değerlendirilen toplam açı sayısı)
          - 'gecerli_sayisi': int (geçerli bulunan yaklaşma pozu sayısı)
          - 'en_iyi_elenen_acikliklar': list[float] (en yüksek açıklığa sahip elenen 3 adayın açıklıkları)
          - 'en_kucuk_acikliklar': list[float] (alias)
          - 'acikliklar': list[float] (alias)
          - 'en_iyi_elenenler': list[dict] (en iyi 3 adayın açı, neden ve açıklık ayrıntıları)
          - 'nedenler': dict (nedenlerin sayıları)
    """
    a = a or Ayar()
    try:
        from scipy.spatial import cKDTree
        engeller = h.dolu_noktalar()
        agac = cKDTree(engeller) if len(engeller) else None
    except ImportError:
        engeller, agac = None, None

    th_yuzey = yuzey_normali_bul(engeller, agac, nesne, r_arama=0.30, r_nesne=a.r_nesne)
    bolge = erisilebilir_bolge(h, robot) if robot is not None else None

    sayilar = {
        'P_carpisma': 0,
        'koridor': 0,
        'Q_carpisma': 0,
        'erisim': 0,
        'yuzey': 0,
        'yon': 0,
    }
    elenen_adaylar = []
    gecerli_sayisi = 0
    toplam_aday = 0

    for th in np.radians(np.arange(-180, 180, a.aci_adim)):
        toplam_aday += 1
        y, neden, acik = aday_incele(h, engeller, agac, nesne, float(th), a, robot=robot, th_yuzey=th_yuzey)

        if y and bolge is not None:
            v, u = h.hucre(y.oncesi[0], y.oncesi[1])
            if not (0 <= v < bolge.shape[0] and 0 <= u < bolge.shape[1] and bolge[v, u]):
                y = None
                neden = 'erisim'

        if y is not None:
            gecerli_sayisi += 1
        else:
            if neden in sayilar:
                sayilar[neden] += 1
            else:
                sayilar[neden] = 1
            elenen_adaylar.append({
                'theta': float(th),
                'theta_derece': round(float(math.degrees(th)), 1),
                'neden': neden,
                'aciklik': round(float(acik), 4),
            })

    # Elenen en iyi 3 adayın açıklıkları (açıklığı en yüksek olanlar)
    elenen_adaylar.sort(key=lambda item: -item['aciklik'])
    en_iyi_3 = elenen_adaylar[:3]
    en_iyi_acikliklar = [c['aciklik'] for c in en_iyi_3]

    return {
        'P_carpisma': sayilar['P_carpisma'],
        'koridor': sayilar['koridor'],
        'Q_carpisma': sayilar['Q_carpisma'],
        'erisim': sayilar['erisim'],
        'yuzey': sayilar['yuzey'],
        'yon': sayilar['yon'],
        'toplam_aday': toplam_aday,
        'gecerli_sayisi': gecerli_sayisi,
        'en_iyi_elenen_acikliklar': en_iyi_acikliklar,
        'en_kucuk_acikliklar': en_iyi_acikliklar,
        'acikliklar': en_iyi_acikliklar,
        'en_iyi_elenenler': en_iyi_3,
        'nedenler': dict(sayilar),
    }


def icp2d(P, agac, M, T0, esikler=(0.3, 0.15, 0.08, 0.04), it=10):
    """2B noktadan noktaya ICP (saha/analiz/lidar_poz.py ile aynı): P (N,2) robot çerçevesinde tarama, M harita
    noktaları + cKDTree, T0 3x3 başlangıç -> (T, uyum: <4 cm eşleşen oran)"""
    T = T0.copy()
    for e in esikler:
        for _ in range(it):
            Q = P @ T[:2, :2].T + T[:2, 2]
            d, j = agac.query(Q, distance_upper_bound=e)
            k = np.isfinite(d)
            if k.sum() < 20:
                return T, 0.0
            A, B = Q[k], M[j[k]]
            ca, cb = A.mean(0), B.mean(0)
            U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
            R = (U @ Vt).T
            if np.linalg.det(R) < 0:
                Vt[1] *= -1
                R = (U @ Vt).T
            dT = np.eye(3)
            dT[:2, :2], dT[:2, 2] = R, cb - R @ ca
            T = dT @ T
            if abs(math.atan2(R[1, 0], R[0, 0])) < 1e-5 and np.linalg.norm(dT[:2, 2]) < 1e-5:
                break
    d, _ = agac.query(P @ T[:2, :2].T + T[:2, 2], distance_upper_bound=0.04)
    return T, float(np.isfinite(d).mean())
