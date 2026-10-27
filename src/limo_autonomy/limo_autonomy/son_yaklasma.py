"""Nesneye göreli kapalı çevrim son yaklaşma denetleyicisi (ROS'suz, saf Python).

Robot kavrama öncesi poz Q'dayken (nesneye ~0.35–0.6 m), nesnenin robot çerçevesindeki
ölçümüne göre P'ye (nesne robotun (d_kavrama, yanal)'ında, robot yönü istenen yaklaşma
yönünde) hassas varmayı sağlar.

Bant dışında Astolfi kutupsal durum geribeslemeli denetleyicisi diferansiyel sürüş
kinematiğine uygun kapalı çevrim sürüş üretir. Son banda girildiğinde (rho < son_donus_mesafe)
hedef çerçevesinde ayrık hizalama (Aşama A) ve sürünme (Aşama B) mantığı salınımı önler.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap(aci: float) -> float:
    """Açıyı [-pi, pi) aralığına sarar."""
    return math.atan2(math.sin(aci), math.cos(aci))


@dataclass
class SonYaklasmaAyar:
    """Son yaklaşma algoritması yapılandırma parametreleri."""

    d_kavrama: float = 0.22      # robot merkezi -> kavrama noktası (m)
    yanal: float = 0.0           # kavrama noktasının robot eksenine yanal ofseti (m, + sol)
    v_max: float = 0.10          # en yüksek doğrusal hız (m/s)
    v_min: float = 0.015         # en düşük doğrusal hız eşiği (m/s)
    w_max: float = 0.6           # en yüksek açısal hız (rad/s)
    a_max: float = 0.3           # en yüksek doğrusal ivme (m/s²)
    k_rho: float = 0.6           # Astolfi mesafe kazancı (k_rho > 0)
    k_alpha: float = 1.8         # Astolfi yönelme açısı kazancı (k_alpha - k_rho > 0)
    k_beta: float = -0.5         # Astolfi hedef açı kazancı (k_beta < 0)
    tol_xy: float = 0.01         # hedef konum toleransı (m)
    tol_yaw: float = math.radians(2.0)  # hedef yönelim toleransı (rad)
    son_donus_mesafe: float = 0.03      # bu mesafe içinde son yaklaşma bandı (m)
    olcum_zaman_asimi: float = 1.0      # ölçüm gelmezse zaman aşımı (s)
    sapma_esik: float = 0.15     # tahmin edilen hedefin ani sıçrama eşiği (m)
    geri_izin: bool = True       # hedef arkada kalırsa kısa geri manevraya izin ver
    max_sure: float = 40.0       # görev için azami süre (s)

    # Son bant (Aşama A & B) parametreleri
    k_yaw: float = 1.5           # son bant açısal hizalama kazancı
    w_min_donus: float = 0.05    # son bant en düşük dönüş hızı (rad/s, ölü bant)
    k_ileri: float = 1.0         # son bant sürünme ileri kazancı
    v_surunme: float = 0.03      # son bant en yüksek sürünme hızı (m/s)
    tol_yanal: float = 0.015     # son bant yanal hata toleransı (m)
    yanal_hata_esik: float = 0.03  # son bant kabul edilemez yanal hata eşiği (m)
    yanal_kabul_genis: bool = True  # tol_yanal <= |e_yanal| < yanal_hata_esik iken TAMAM+uyarı izni

    # Görüş koruma parametreleri
    gorus_yari_aci: float = math.radians(35.0)  # kamera yatay görüş yarı açısı (rad, varsayılan 35°)
    gorus_payi: float = math.radians(10.0)      # görüş koruma emniyet payı (rad, varsayılan 10°)
    olcumsuz_mesafe: float = 0.25               # bu mesafenin altında ölçüm kesilse de odomla devam et (m)

    def __post_init__(self) -> None:
        """Astolfi kutupsal denetleyici kazançlarının asimptotik kararlılık koşullarını doğrular."""
        if not (self.k_rho > 0 and self.k_beta < 0 and (self.k_alpha - self.k_rho) > 0):
            raise ValueError(
                f"Geçersiz Astolfi kazançları: k_rho={self.k_rho} (>0), "
                f"k_beta={self.k_beta} (<0), k_alpha - k_rho={self.k_alpha - self.k_rho} (>0) olmalı."
            )
        if abs(self.gorus_yari_aci) > math.pi:
            self.gorus_yari_aci = math.radians(self.gorus_yari_aci)
        if abs(self.gorus_payi) > math.pi:
            self.gorus_payi = math.radians(self.gorus_payi)


def hedef_pozu(
    nesne_xy_odom: tuple[float, float],
    yaklasma_yaw_odom: float,
    ayar: SonYaklasmaAyar,
) -> tuple[float, float, float]:
    """Nesne ve yaklaşma yönünden robotun varması gereken P pozunu (odom çerçevesinde) hesaplar.

    P = nesne - (d_kavrama * [cos, sin]) + yanal * [-sin, cos], yaw = yaklasma_yaw.
    """
    ox, oy = float(nesne_xy_odom[0]), float(nesne_xy_odom[1])
    c = math.cos(yaklasma_yaw_odom)
    s = math.sin(yaklasma_yaw_odom)
    px = ox - ayar.d_kavrama * c + ayar.yanal * (-s)
    py = oy - ayar.d_kavrama * s + ayar.yanal * c
    return (float(px), float(py), float(yaklasma_yaw_odom))


def nesne_robottan_odoma(
    nesne_robot_xy: tuple[float, float],
    robot_odom_pose: tuple[float, float, float],
) -> tuple[float, float]:
    """Robot gövde çerçevesindeki (base_footprint) nesne ölçümünü odom çerçevesine dönüştürür."""
    lx, ly = float(nesne_robot_xy[0]), float(nesne_robot_xy[1])
    rx, ry, ryaw = float(robot_odom_pose[0]), float(robot_odom_pose[1]), float(robot_odom_pose[2])
    c = math.cos(ryaw)
    s = math.sin(ryaw)
    ox = rx + c * lx - s * ly
    oy = ry + s * lx + c * ly
    return (float(ox), float(oy))


def nesne_robotta_son(
    robot_odom_pose: tuple[float, float, float],
    nesne_odom: tuple[float, float],
) -> tuple[float, float]:
    """Odom çerçevesindeki nesne konumunu robotun o anki base_footprint çerçevesine dönüştürür."""
    rx, ry, ryaw = float(robot_odom_pose[0]), float(robot_odom_pose[1]), float(robot_odom_pose[2])
    ox, oy = float(nesne_odom[0]), float(nesne_odom[1])
    dx = ox - rx
    dy = oy - ry
    c = math.cos(ryaw)
    s = math.sin(ryaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return (float(lx), float(ly))


class NesneTahmini:
    """Odom çerçevesinde nesne konumunun üstel ortalamayla takibi ve aykırı ölçüm filtrelemesi."""

    def __init__(self, alfa: float = 0.4, esik_aykiri: float = 0.10) -> None:
        self.alfa = alfa
        self.esik_aykiri = esik_aykiri
        self.x: float | None = None
        self.y: float | None = None
        self.son_zaman: float | None = None
        self.aykiri_sayac: int = 0

    def guncelle(self, olcum_xy_odom: tuple[float, float], t: float) -> bool:
        """Yeni bir odom çerçeveli ölçümle tahmini günceller.

        İlk ölçüm doğrudan atanır. >0.10 m sıçrayan aykırı ölçümler 3 ardışık
        gelmedikçe reddedilir.
        """
        mx, my = float(olcum_xy_odom[0]), float(olcum_xy_odom[1])
        if self.x is None or self.y is None:
            self.x = mx
            self.y = my
            self.son_zaman = t
            self.aykiri_sayac = 0
            return True

        fark = math.hypot(mx - self.x, my - self.y)
        if fark > self.esik_aykiri:
            self.aykiri_sayac += 1
            if self.aykiri_sayac >= 3:
                self.x = mx
                self.y = my
                self.son_zaman = t
                self.aykiri_sayac = 0
                return True
            return False

        self.aykiri_sayac = 0
        self.x = (1.0 - self.alfa) * self.x + self.alfa * mx
        self.y = (1.0 - self.alfa) * self.y + self.alfa * my
        self.son_zaman = t
        return True

    def yas(self, t: float) -> float:
        """Son geçerli ölçümden bu yana geçen süreyi (saniye) döner."""
        if self.son_zaman is None:
            return float('inf')
        return max(0.0, t - self.son_zaman)

    def gecerli(self, t: float, zaman_asimi: float = 1.0) -> bool:
        """Tahminin zaman aşımına uğramadığını doğrular."""
        return self.son_zaman is not None and self.yas(t) <= zaman_asimi

    @property
    def konum(self) -> tuple[float, float] | None:
        if self.x is None or self.y is None:
            return None
        return (self.x, self.y)


class SonYaklasma:
    """Kapalı çevrim nesneye göreli son yaklaşma denetleyicisi."""

    def __init__(
        self,
        ayar: SonYaklasmaAyar | None = None,
        yaklasma_yaw_odom: float | None = None,
    ) -> None:
        self.ayar = ayar if ayar is not None else SonYaklasmaAyar()
        self.yaklasma_yaw_odom = yaklasma_yaw_odom
        self.nesne_tahmin = NesneTahmini(alfa=0.4, esik_aykiri=0.10)

        self.t_baslangic: float | None = None
        self.onceki_t: float | None = None
        self.onceki_v: float = 0.0
        self.onceki_hedef: tuple[float, float, float] | None = None
        self._son_bantta: bool = False
        self._son_uyari: str | None = None
        self._tamamlandi: bool = False

    def adim(
        self,
        robot_odom_pose: tuple[float, float, float],
        t: float,
        olcum_robot_xy: tuple[float, float] | None = None,
        guvenlik_mesafe: float | None = None,
    ) -> tuple[float, float, str, dict]:
        """Tek bir kontrol çevrimi adımı yürütür.

        Döndürür:
            (v, w, durum, bilgi)
            durum ∈ {'YAKLASIYOR', 'DONUYOR', 'TAMAM', 'HATA:<neden>'}
            bilgi: rho, alpha, beta, e_yaw, hedef, nesne_odom, olcum_yasi, e_ileri, e_yanal
        """
        rx, ry, ryaw = (
            float(robot_odom_pose[0]),
            float(robot_odom_pose[1]),
            float(robot_odom_pose[2]),
        )
        if self.t_baslangic is None:
            self.t_baslangic = t

        dt = (t - self.onceki_t) if self.onceki_t is not None else 0.0
        self.onceki_t = t

        # Güvenlik engeli kontrolü
        if guvenlik_mesafe is not None and guvenlik_mesafe < 0.05:
            self.onceki_v = 0.0
            return (0.0, 0.0, 'HATA:engel', self._bos_bilgi(robot_odom_pose, t))

        # Görev azami süre kontrolü
        if (t - self.t_baslangic) > self.ayar.max_sure:
            self.onceki_v = 0.0
            return (0.0, 0.0, 'HATA:max_sure', self._bos_bilgi(robot_odom_pose, t))

        # Yeni ölçüm geldiyse nesne tahminini güncelle
        if olcum_robot_xy is not None:
            olcum_odom = nesne_robottan_odoma(olcum_robot_xy, (rx, ry, ryaw))
            onceki_konum = self.nesne_tahmin.konum
            kabul = self.nesne_tahmin.guncelle(olcum_odom, t)
            if kabul and onceki_konum is not None:
                sicrama = math.hypot(
                    self.nesne_tahmin.x - onceki_konum[0],  # type: ignore[operator]
                    self.nesne_tahmin.y - onceki_konum[1],  # type: ignore[operator]
                )
                if sicrama > self.ayar.sapma_esik:
                    self.onceki_v = 0.0
                    return (0.0, 0.0, 'HATA:sapma', self._bos_bilgi(robot_odom_pose, t))

            # Yaklaşma yönü verilmemişse ilk ölçümde robot->nesne doğrultusu sabitlenir
            if self.yaklasma_yaw_odom is None and self.nesne_tahmin.konum is not None:
                ox, oy = self.nesne_tahmin.konum
                self.yaklasma_yaw_odom = math.atan2(oy - ry, ox - rx)

        # Nesne tahmini henüz hiç oluşmadıysa bekleme / zaman aşımı kontrolü
        if self.nesne_tahmin.konum is None or self.yaklasma_yaw_odom is None:
            if (t - self.t_baslangic) > self.ayar.olcum_zaman_asimi:
                self.onceki_v = 0.0
                return (0.0, 0.0, 'HATA:zaman_asimi', self._bos_bilgi(robot_odom_pose, t))
            self.onceki_v = 0.0
            return (0.0, 0.0, 'YAKLASIYOR', self._bos_bilgi(robot_odom_pose, t))

        ox, oy = self.nesne_tahmin.konum
        hedef = hedef_pozu((ox, oy), self.yaklasma_yaw_odom, self.ayar)
        px, py, th_g = hedef

        # Hedef konumda ani sıçrama kontrolü (sapma_esik)
        if self.onceki_hedef is not None:
            hedef_sicrama = math.hypot(px - self.onceki_hedef[0], py - self.onceki_hedef[1])
            if hedef_sicrama > self.ayar.sapma_esik:
                self.onceki_v = 0.0
                return (0.0, 0.0, 'HATA:sapma', self._bos_bilgi(robot_odom_pose, t))
        self.onceki_hedef = hedef

        # Hata hesapları:
        # e_ileri: hedef doğrultusunda robot -> P mesafesi
        # e_yanal: hedef eksenine dik robot -> P ofseti
        # e_yaw: hedef yönelim ile robot yönelimi farkı
        dx = px - rx
        dy = py - ry
        rho = math.hypot(dx, dy)
        e_yaw = wrap(th_g - ryaw)

        c_g = math.cos(th_g)
        s_g = math.sin(th_g)
        e_ileri = dx * c_g + dy * s_g
        e_yanal = -dx * s_g + dy * c_g

        # Ölçüm kaybı ve zaman aşımı kontrolü:
        # olcumsuz_mesafe içinde (varsayılan 0.25 m) ölçüm kaybı tolere edilir (kamera kör noktası; odomla tamamlanır).
        olcum_yasi = self.nesne_tahmin.yas(t)
        if olcum_yasi > self.ayar.olcum_zaman_asimi and rho > self.ayar.olcumsuz_mesafe:
            self.onceki_v = 0.0
            bilgi = self._bilgi(
                rho=rho,
                alpha=0.0,
                beta=0.0,
                e_yaw=e_yaw,
                hedef=hedef,
                nesne_odom=(ox, oy),
                olcum_yasi=olcum_yasi,
                e_ileri=e_ileri,
                e_yanal=e_yanal,
            )
            return (0.0, 0.0, 'HATA:zaman_asimi', bilgi)

        # Hedefe zaten ulaşıldıysa TAMAM durumunu koru
        if self._tamamlandi:
            bilgi = self._bilgi(
                rho=rho,
                alpha=0.0,
                beta=0.0,
                e_yaw=e_yaw,
                hedef=hedef,
                nesne_odom=(ox, oy),
                olcum_yasi=olcum_yasi,
                e_ileri=e_ileri,
                e_yanal=e_yanal,
                uyari=self._son_uyari,
            )
            return (0.0, 0.0, 'TAMAM', bilgi)

        # Son bant histerezis kontrolü:
        # rho < son_donus_mesafe ile banda girilir, girildikten sonra rho < 2*son_donus_mesafe boyunca kalınır.
        if self._son_bantta:
            if rho >= 2.0 * self.ayar.son_donus_mesafe:
                self._son_bantta = False
        else:
            if rho < self.ayar.son_donus_mesafe:
                self._son_bantta = True

        # =========================================================================
        # 1. Son Bant Mantığı (Aşama A: Hizala, Aşama B: Sürün)
        # =========================================================================
        if self._son_bantta:
            # Kabul edilemez yanal hata eşiği kontrolü
            if abs(e_yanal) >= self.ayar.yanal_hata_esik:
                self.onceki_v = 0.0
                bilgi = self._bilgi(
                    rho=rho,
                    alpha=0.0,
                    beta=0.0,
                    e_yaw=e_yaw,
                    hedef=hedef,
                    nesne_odom=(ox, oy),
                    olcum_yasi=olcum_yasi,
                    e_ileri=e_ileri,
                    e_yanal=e_yanal,
                )
                return (0.0, 0.0, 'HATA:yanal', bilgi)

            ileri_tamam = abs(e_ileri) < self.ayar.tol_xy
            yaw_tamam = abs(e_yaw) < self.ayar.tol_yaw
            yanal_tamam = abs(e_yanal) < self.ayar.tol_yanal
            yanal_uyari = (
                self.ayar.yanal_kabul_genis
                and (self.ayar.tol_yanal <= abs(e_yanal) < self.ayar.yanal_hata_esik)
            )

            # TAMAM koşulu
            if ileri_tamam and yaw_tamam and (yanal_tamam or yanal_uyari):
                self._tamamlandi = True
                self.onceki_v = 0.0
                uyari = 'yanal' if (not yanal_tamam and yanal_uyari) else None
                self._son_uyari = uyari
                bilgi = self._bilgi(
                    rho=rho,
                    alpha=0.0,
                    beta=0.0,
                    e_yaw=e_yaw,
                    hedef=hedef,
                    nesne_odom=(ox, oy),
                    olcum_yasi=olcum_yasi,
                    e_ileri=e_ileri,
                    e_yanal=e_yanal,
                    uyari=uyari,
                )
                return (0.0, 0.0, 'TAMAM', bilgi)

            # İleri ve yönelim tamamlandı ancak yanal hata tolerans dışı ve geniş kabule izin verilmemiş
            if ileri_tamam and yaw_tamam and not (yanal_tamam or yanal_uyari):
                self.onceki_v = 0.0
                bilgi = self._bilgi(
                    rho=rho,
                    alpha=0.0,
                    beta=0.0,
                    e_yaw=e_yaw,
                    hedef=hedef,
                    nesne_odom=(ox, oy),
                    olcum_yasi=olcum_yasi,
                    e_ileri=e_ileri,
                    e_yanal=e_yanal,
                )
                return (0.0, 0.0, 'HATA:yanal', bilgi)

            if abs(e_yaw) > self.ayar.tol_yaw:
                # Aşama A (HIZALA): yerinde dönerek yönelimi hizala
                durum = 'DONUYOR'
                v = 0.0
                self.onceki_v = 0.0
                w_val = self.ayar.k_yaw * e_yaw
                w = max(-self.ayar.w_max, min(self.ayar.w_max, w_val))
                if abs(w) < self.ayar.w_min_donus:
                    w = math.copysign(self.ayar.w_min_donus, e_yaw if e_yaw != 0.0 else 1.0)
            else:
                # Aşama B (SURUN): küçük hızla ileri/geri sürünerek e_ileri hatasını kapat, yönü tut
                durum = 'YAKLASIYOR'
                v_val = self.ayar.k_ileri * e_ileri
                v = max(-self.ayar.v_surunme, min(self.ayar.v_surunme, v_val))
                if abs(e_ileri) > self.ayar.tol_xy:
                    if abs(v) < self.ayar.v_min:
                        v = math.copysign(self.ayar.v_min, e_ileri if e_ileri != 0.0 else 1.0)
                w_val = self.ayar.k_yaw * e_yaw
                w = max(-self.ayar.w_max, min(self.ayar.w_max, w_val))
                self.onceki_v = v

            bilgi = self._bilgi(
                rho=rho,
                alpha=0.0,
                beta=0.0,
                e_yaw=e_yaw,
                hedef=hedef,
                nesne_odom=(ox, oy),
                olcum_yasi=olcum_yasi,
                e_ileri=e_ileri,
                e_yanal=e_yanal,
            )
            return (v, w, durum, bilgi)

        # =========================================================================
        # 2. Bant Dışı: Astolfi Kutupsal Durum Geribeslemeli Denetleyici
        # =========================================================================
        delta = math.atan2(dy, dx)
        alpha = wrap(delta - ryaw)
        beta = wrap(th_g - delta)

        durum = 'YAKLASIYOR'
        if abs(alpha) > math.pi / 2 and self.ayar.geri_izin:
            # Hedef arkada kaldıysa ters yön formülasyonu (kısa geri manevra)
            if alpha > math.pi / 2:
                alpha_rev = alpha - math.pi
                beta_rev = wrap(beta - math.pi)
            else:
                alpha_rev = alpha + math.pi
                beta_rev = wrap(beta + math.pi)
            v_ham = -self.ayar.k_rho * rho
            w_ham = self.ayar.k_alpha * alpha_rev + self.ayar.k_beta * beta_rev
        else:
            v_ham = self.ayar.k_rho * rho
            w_ham = self.ayar.k_alpha * alpha + self.ayar.k_beta * beta

        # Açısal hız sınırlandırması
        w = max(-self.ayar.w_max, min(self.ayar.w_max, w_ham))

        # =====================================================================
        # Görüş koruma kısıtı (Bant dışı; son bantta Aşama A/B uygulanmaz)
        # =====================================================================
        th_nesne = math.atan2(oy - ry, ox - rx)
        b = wrap(th_nesne - ryaw)
        d_nesne = math.hypot(ox - rx, oy - ry)
        b_sinir = max(1e-3, self.ayar.gorus_yari_aci - self.ayar.gorus_payi)

        # Doğrusal hız: kerteriz büyüdükçe azalt (b -> sınır iken v -> v_min)
        oran_b = min(1.0, abs(b) / b_sinir)
        v_maks_b = self.ayar.v_min + (1.0 - oran_b) * (self.ayar.v_max - self.ayar.v_min)

        if v_ham > 0:
            v_hedef = min(v_maks_b, max(self.ayar.v_min, v_ham))
        elif v_ham < 0:
            v_hedef = -min(v_maks_b, max(self.ayar.v_min, -v_ham))
        else:
            v_hedef = 0.0

        if dt > 0:
            max_dv = self.ayar.a_max * dt
            v = max(self.onceki_v - max_dv, min(self.onceki_v + max_dv, v_hedef))
        else:
            v = v_hedef

        if abs(b) >= b_sinir and abs(v) > self.ayar.v_min:
            v = math.copysign(self.ayar.v_min, v if v != 0.0 else 1.0)

        # Açısal hız: bir sonraki adımın tahmini dahil |b| > b_sinir olacaksa w'yi
        # kerterizi azaltacak yöne kırp
        dt_est = dt if dt > 1e-4 else 0.02
        ryaw_s = wrap(ryaw + w * dt_est)
        rx_s = rx + v * math.cos(ryaw) * dt_est
        ry_s = ry + v * math.sin(ryaw) * dt_est
        b_sonraki = wrap(math.atan2(oy - ry_s, ox - rx_s) - ryaw_s)

        if b_sonraki > b_sinir or b > b_sinir:
            # Nesne solda: kerterizi azaltmak için w pozitif (sola) olmalı
            w_alt = (b - b_sinir) / dt_est - (v * math.sin(b) / max(0.01, d_nesne))
            w = max(w, max(0.0, w_alt))
            for _ in range(10):
                ryaw_s = wrap(ryaw + w * dt_est)
                rx_s = rx + v * math.cos(ryaw) * dt_est
                ry_s = ry + v * math.sin(ryaw) * dt_est
                bs = wrap(math.atan2(oy - ry_s, ox - rx_s) - ryaw_s)
                if bs <= b_sinir or w >= self.ayar.w_max:
                    break
                w = min(self.ayar.w_max, w + 0.05)
        elif b_sonraki < -b_sinir or b < -b_sinir:
            # Nesne sağda: kerterizi azaltmak (|b| küçültmek) için w negatif (sağa) olmalı
            w_ust = (b + b_sinir) / dt_est - (v * math.sin(b) / max(0.01, d_nesne))
            w = min(w, min(0.0, w_ust))
            for _ in range(10):
                ryaw_s = wrap(ryaw + w * dt_est)
                rx_s = rx + v * math.cos(ryaw) * dt_est
                ry_s = ry + v * math.sin(ryaw) * dt_est
                bs = wrap(math.atan2(oy - ry_s, ox - rx_s) - ryaw_s)
                if -bs <= b_sinir or w <= -self.ayar.w_max:
                    break
                w = max(-self.ayar.w_max, w - 0.05)

        w = max(-self.ayar.w_max, min(self.ayar.w_max, w))
        self.onceki_v = v

        bilgi = self._bilgi(
            rho=rho,
            alpha=alpha,
            beta=beta,
            e_yaw=e_yaw,
            hedef=hedef,
            nesne_odom=(ox, oy),
            olcum_yasi=olcum_yasi,
            e_ileri=e_ileri,
            e_yanal=e_yanal,
        )
        return (v, w, durum, bilgi)

    def _bilgi(
        self,
        rho: float,
        alpha: float,
        beta: float,
        e_yaw: float,
        hedef: tuple[float, float, float] | None,
        nesne_odom: tuple[float, float] | None,
        olcum_yasi: float,
        e_ileri: float = 0.0,
        e_yanal: float = 0.0,
        uyari: str | None = None,
    ) -> dict:
        bilgi_sozlugu = {
            'rho': float(rho),
            'alpha': float(alpha),
            'beta': float(beta),
            'e_yaw': float(e_yaw),
            'hedef': hedef,
            'nesne_odom': nesne_odom,
            'olcum_yasi': float(olcum_yasi),
            'e_ileri': float(e_ileri),
            'e_yanal': float(e_yanal),
        }
        if uyari is not None:
            bilgi_sozlugu['uyari'] = str(uyari)
        return bilgi_sozlugu

    def _bos_bilgi(self, robot_odom_pose: tuple[float, float, float], t: float) -> dict:
        ox, oy = (self.nesne_tahmin.konum if self.nesne_tahmin.konum is not None else (None, None))
        hedef = None
        if ox is not None and oy is not None and self.yaklasma_yaw_odom is not None:
            hedef = hedef_pozu((ox, oy), self.yaklasma_yaw_odom, self.ayar)
        olcum_yasi = self.nesne_tahmin.yas(t)
        return self._bilgi(
            rho=float('nan'),
            alpha=0.0,
            beta=0.0,
            e_yaw=0.0,
            hedef=hedef,
            nesne_odom=(ox, oy) if ox is not None else None,
            olcum_yasi=olcum_yasi,
            e_ileri=0.0,
            e_yanal=0.0,
        )
