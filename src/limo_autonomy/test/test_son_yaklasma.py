"""son_yaklasma: kapalı çevrim nesneye göreli son yaklaşma testleri (ROS'suz saf Python)."""

import math
import random

import pytest

from limo_autonomy.son_yaklasma import (
    SonYaklasma,
    SonYaklasmaAyar,
    hedef_pozu,
    nesne_robotta_son,
    nesne_robottan_odoma,
    NesneTahmini,
    wrap,
)


def kapali_cevrim_simule(
    robot_poz_0: tuple[float, float, float] = (0.0, 0.08, math.radians(15.0)),
    nesne_gercek: tuple[float, float] = (0.45, 0.0),
    yaklasma_yaw: float = 0.0,
    ayar: SonYaklasmaAyar | None = None,
    dt: float = 0.02,
    max_adim: int = 1500,
    gurultu_sigma: float = 0.0,
    olcum_hz: float = 50.0,
    odom_carpani: float = 1.0,
    olcum_kesilme_rho: float | None = None,
    erken_kesilme_rho: float | None = None,
    rng_seed: int = 42,
    adim_cb=None,
) -> tuple[str, tuple[float, float, float], tuple[float, float, float], int]:
    """Basit diferansiyel tekerlek kinematiği ile kapalı çevrim yaklaşma simülasyonu.

    Döndürür:
        (son_durum, son_gercek_robot_pozu, son_odom_robot_pozu, toplam_adim)
    """
    rng = random.Random(rng_seed)
    ayar = ayar if ayar is not None else SonYaklasmaAyar()
    denetleyici = SonYaklasma(ayar=ayar, yaklasma_yaw_odom=yaklasma_yaw)

    rx, ry, ryaw = robot_poz_0
    ox, oy, oyaw = robot_poz_0

    hedef_p = hedef_pozu(nesne_gercek, yaklasma_yaw, ayar)

    son_olcum_t = -1e9
    olcum_araligi = 1.0 / olcum_hz

    durum = 'BASLADI'
    adim_sayisi = 0

    for k in range(max_adim):
        adim_sayisi += 1
        t = k * dt

        # Hedefe gerçek mesafe
        rho_gercek = math.hypot(hedef_p[0] - rx, hedef_p[1] - ry)

        # Ölçüm üretimi
        olcum_robot = None
        kesilme_var = False
        if olcum_kesilme_rho is not None and rho_gercek <= olcum_kesilme_rho:
            kesilme_var = True
        if erken_kesilme_rho is not None and rho_gercek <= erken_kesilme_rho:
            kesilme_var = True

        if not kesilme_var and (t - son_olcum_t) >= (olcum_araligi - 1e-6):
            son_olcum_t = t
            # Gerçek robot çerçevesinde nesne konumu
            dx = nesne_gercek[0] - rx
            dy = nesne_gercek[1] - ry
            c, s = math.cos(ryaw), math.sin(ryaw)
            lx = c * dx + s * dy
            ly = -s * dx + c * dy

            if gurultu_sigma > 0.0:
                lx += rng.gauss(0.0, gurultu_sigma)
                ly += rng.gauss(0.0, gurultu_sigma)
            olcum_robot = (lx, ly)

        v, w, durum, _ = denetleyici.adim(
            robot_odom_pose=(ox, oy, oyaw),
            t=t,
            olcum_robot_xy=olcum_robot,
        )

        if adim_cb is not None:
            adim_cb(k, t, (rx, ry, ryaw), (ox, oy, oyaw), v, w, durum, denetleyici)

        if durum == 'TAMAM' or durum.startswith('HATA'):
            break

        # Odom kinematiği (komut üzerinden bütünleme)
        ox += v * math.cos(oyaw) * dt
        oy += v * math.sin(oyaw) * dt
        oyaw = wrap(oyaw + w * dt)

        # Gerçek hareket (fiziksel robot: odom_carpani kadar kayma/fark içerebilir)
        v_gercek = v * odom_carpani
        w_gercek = w * odom_carpani
        rx += v_gercek * math.cos(ryaw) * dt
        ry += v_gercek * math.sin(ryaw) * dt
        ryaw = wrap(ryaw + w_gercek * dt)

    return durum, (rx, ry, ryaw), (ox, oy, oyaw), adim_sayisi


def test_temel_yaklasma():
    """(a) başlangıç Q = nesneye 0.45 m, yanal 0.08 m kayık, 15° yön hatası

    -> TAMAM, son |e_xy|<1.5 cm, |e_yaw|<2.5°.
    """
    ayar = SonYaklasmaAyar()
    durum, gercek_poz, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.08, math.radians(15.0)),
        nesne_gercek=(0.45, 0.0),
        yaklasma_yaw=0.0,
        ayar=ayar,
        dt=0.02,
    )
    assert durum == 'TAMAM'
    hedef = hedef_pozu((0.45, 0.0), 0.0, ayar)
    e_xy = math.hypot(hedef[0] - gercek_poz[0], hedef[1] - gercek_poz[1])
    e_yaw = abs(wrap(hedef[2] - gercek_poz[2]))

    assert e_xy < 0.015, f"Konum hatası çok büyük: {e_xy * 100:.2f} cm (beklenen < 1.5 cm)"
    assert e_yaw < math.radians(2.5), f"Yönelim hatası çok büyük: {math.degrees(e_yaw):.2f}° (beklenen < 2.5°)"


def test_gurultulu_ve_seyrek_olcum():
    """(b) ölçümlere σ=1 cm gürültü + 5 Hz -> yine TAMAM, hatalar < 2 cm / 3°."""
    ayar = SonYaklasmaAyar()
    durum, gercek_poz, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.08, math.radians(15.0)),
        nesne_gercek=(0.45, 0.0),
        yaklasma_yaw=0.0,
        ayar=ayar,
        dt=0.02,
        gurultu_sigma=0.01,  # 1 cm standart sapma
        olcum_hz=5.0,        # 5 Hz seyrek ölçüm
        rng_seed=123,
    )
    assert durum == 'TAMAM'
    hedef = hedef_pozu((0.45, 0.0), 0.0, ayar)
    e_xy = math.hypot(hedef[0] - gercek_poz[0], hedef[1] - gercek_poz[1])
    e_yaw = abs(wrap(hedef[2] - gercek_poz[2]))

    assert e_xy < 0.02, f"Gürültülü konum hatası çok büyük: {e_xy * 100:.2f} cm (beklenen < 2 cm)"
    assert e_yaw < math.radians(3.0), f"Gürültülü açı hatası çok büyük: {math.degrees(e_yaw):.2f}° (beklenen < 3°)"


def test_odom_kaymasi():
    """(c) odom kayması: robotun 'gerçek' hareketi komutun %2 fazlası

    (ölçüm gerçek göreli konumdan) -> kapalı çevrim telafi eder, TAMAM ve
    gerçek göreli hata < 2 cm.
    """
    ayar = SonYaklasmaAyar()
    nesne_gercek = (0.45, 0.0)
    durum, gercek_poz, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.08, math.radians(15.0)),
        nesne_gercek=nesne_gercek,
        yaklasma_yaw=0.0,
        ayar=ayar,
        dt=0.02,
        odom_carpani=1.02,   # %2 fazla hareket
    )
    assert durum == 'TAMAM'

    # Nesnenin gerçek robot gövdesindeki (base_footprint) konumu
    lx, ly = nesne_robotta_son(gercek_poz, nesne_gercek)
    # Hedeflenen göreli konum: (d_kavrama, yanal) = (0.22, 0.0)
    goreli_hata = math.hypot(lx - ayar.d_kavrama, ly - ayar.yanal)
    assert goreli_hata < 0.02, f"Gerçek göreli hata çok büyük: {goreli_hata * 100:.2f} cm (beklenen < 2 cm)"


def test_olcum_kesilmesi_son_8cm_ve_30cm():
    """(d) son 8 cm'de ölçüm kesilir -> yine TAMAM; 45 cm'de kesilir -> 'HATA' (zaman aşımı)."""
    ayar = SonYaklasmaAyar(olcum_zaman_asimi=1.0)

    # 1. Son 8 cm'de kesilme: kamera yakın kör noktada nesneyi kaybetse bile TAMAM'a varmalı
    durum_8cm, gercek_poz_8cm, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.08, math.radians(15.0)),
        nesne_gercek=(0.45, 0.0),
        yaklasma_yaw=0.0,
        ayar=ayar,
        dt=0.02,
        olcum_kesilme_rho=0.08,  # rho <= 8 cm iken ölçüm kesilir
    )
    assert durum_8cm == 'TAMAM'
    hedef = hedef_pozu((0.45, 0.0), 0.0, ayar)
    e_xy = math.hypot(hedef[0] - gercek_poz_8cm[0], hedef[1] - gercek_poz_8cm[1])
    assert e_xy < 0.02

    # 2. 30 cm'de kesilme: hedeften uzakta (> 10 cm) ölçüm kesilirse zaman aşımı hatası vermeli
    durum_30cm, _, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.0, 0.0),
        nesne_gercek=(0.60, 0.0),
        yaklasma_yaw=0.0,
        ayar=ayar,
        dt=0.02,
        erken_kesilme_rho=0.45,  # N13: olcumsuz_mesafe 0.25; 30 cm'de kesilince zaman aşımına kadar <25 cm'ye girip TAMAM olur
    )
    assert durum_30cm.startswith('HATA')
    assert 'zaman_asimi' in durum_30cm


def test_guvenlik_engeli():
    """(e) guvenlik_mesafe 0.03 -> 'HATA:engel' ve v=w=0."""
    denetleyici = SonYaklasma()
    v, w, durum, bilgi = denetleyici.adim(
        robot_odom_pose=(0.0, 0.0, 0.0),
        t=0.0,
        olcum_robot_xy=(0.35, 0.0),
        guvenlik_mesafe=0.03,
    )
    assert durum == 'HATA:engel'
    assert v == 0.0
    assert w == 0.0


def test_astolfi_kazanc_dogrulamasi():
    """(f) Astolfi kazanç doğrulaması geçersiz değerde ValueError."""
    # k_rho <= 0
    with pytest.raises(ValueError):
        SonYaklasmaAyar(k_rho=0.0)
    with pytest.raises(ValueError):
        SonYaklasmaAyar(k_rho=-0.5)

    # k_beta >= 0
    with pytest.raises(ValueError):
        SonYaklasmaAyar(k_beta=0.0)
    with pytest.raises(ValueError):
        SonYaklasmaAyar(k_beta=0.3)

    # k_alpha - k_rho <= 0
    with pytest.raises(ValueError):
        SonYaklasmaAyar(k_rho=0.6, k_alpha=0.6)
    with pytest.raises(ValueError):
        SonYaklasmaAyar(k_rho=0.6, k_alpha=0.5)

    # Geçerli kazançlar kabul edilmeli
    ayar = SonYaklasmaAyar(k_rho=0.6, k_alpha=1.8, k_beta=-0.5)
    assert ayar.k_rho == 0.6
    assert ayar.k_alpha == 1.8
    assert ayar.k_beta == -0.5


def test_yardimci_fonksiyonlar_ve_filtre():
    """Yardımcı koordinat dönüşümleri ve NesneTahmini filtre birim testleri."""
    ayar = SonYaklasmaAyar(d_kavrama=0.22, yanal=0.0)
    hedef = hedef_pozu((1.0, 2.0), math.radians(90.0), ayar)
    # yaw = 90 deg: cos=0, sin=1 -> px = 1 - 0.22*0 = 1, py = 2 - 0.22*1 = 1.78
    assert hedef[0] == pytest.approx(1.0, abs=1e-6)
    assert hedef[1] == pytest.approx(1.78, abs=1e-6)
    assert hedef[2] == pytest.approx(math.radians(90.0), abs=1e-6)

    # NesneTahmini testleri
    tahmin = NesneTahmini(alfa=0.4, esik_aykiri=0.10)
    assert tahmin.konum is None
    assert tahmin.yas(1.0) == float('inf')

    # İlk ölçüm doğrudan atanır
    assert tahmin.guncelle((1.0, 1.0), t=0.0) is True
    assert tahmin.konum == (1.0, 1.0)
    assert tahmin.yas(0.5) == pytest.approx(0.5)

    # Tekil aykırı ölçüm (> 10 cm sıçrama) reddedilir
    assert tahmin.guncelle((1.20, 1.0), t=0.1) is False
    assert tahmin.konum == (1.0, 1.0)

    # 2. ardışık aykırı ölçüm de reddedilir
    assert tahmin.guncelle((1.20, 1.0), t=0.2) is False
    assert tahmin.konum == (1.0, 1.0)

    # 3. ardışık aykırı ölçüm kabul edilir
    assert tahmin.guncelle((1.20, 1.0), t=0.3) is True
    assert tahmin.konum == (1.20, 1.0)

    # Normal ölçüm üstel ortalamayla güncellenir: 0.6 * 1.20 + 0.4 * 1.25 = 0.72 + 0.50 = 1.22
    assert tahmin.guncelle((1.25, 1.0), t=0.4) is True
    assert tahmin.konum[0] == pytest.approx(1.22, abs=1e-6)


def test_son_bant_yanal_hata_genis_kabul_2cm():
    """Banda 2 cm yanal hatayla girilirse 'HATA:yanal' değil TAMAM+uyarı (yanal_kabul_genis=True),
    yanal_kabul_genis=False durumunda ise 'HATA:yanal' olmalı.
    """
    ayar_genis = SonYaklasmaAyar(yanal_kabul_genis=True, son_donus_mesafe=0.03)
    denetleyici_genis = SonYaklasma(ayar=ayar_genis, yaklasma_yaw_odom=0.0)

    # Hedef: nesne=(0.45, 0.0), yaklasma_yaw=0.0 -> P = (0.23, 0.0, 0.0)
    # Robot poz: (0.23, 0.02, 0.0) -> e_ileri = 0.0, e_yanal = -0.02 (2 cm), e_yaw = 0.0
    # rho = 0.02 < son_donus_mesafe (0.03) -> robot doğrudan son bantta
    v, w, durum, bilgi = denetleyici_genis.adim(
        robot_odom_pose=(0.23, 0.02, 0.0),
        t=0.0,
        olcum_robot_xy=(0.22, -0.02),
    )
    assert durum == 'TAMAM'
    assert bilgi.get('uyari') == 'yanal'
    assert abs(bilgi['e_yanal']) == pytest.approx(0.02, abs=1e-4)

    # yanal_kabul_genis = False iken 2 cm hata kabul edilmemeli -> 'HATA:yanal'
    ayar_dar = SonYaklasmaAyar(yanal_kabul_genis=False, son_donus_mesafe=0.03)
    denetleyici_dar = SonYaklasma(ayar=ayar_dar, yaklasma_yaw_odom=0.0)
    _, _, durum_dar, bilgi_dar = denetleyici_dar.adim(
        robot_odom_pose=(0.23, 0.02, 0.0),
        t=0.0,
        olcum_robot_xy=(0.22, -0.02),
    )
    assert durum_dar == 'HATA:yanal'
    assert abs(bilgi_dar['e_yanal']) == pytest.approx(0.02, abs=1e-4)


def test_son_bant_yanal_hata_esik_asimi_4cm():
    """Banda 4 cm yanal hatayla girilirse (yanal_hata_esik=0.03 aşımı) -> 'HATA:yanal'."""
    ayar = SonYaklasmaAyar(son_donus_mesafe=0.05, yanal_hata_esik=0.03)
    denetleyici = SonYaklasma(ayar=ayar, yaklasma_yaw_odom=0.0)

    # Robot poz: (0.23, 0.04, 0.0) -> e_yanal = -0.04 (4 cm)
    # rho = 0.04 < son_donus_mesafe (0.05) -> son banda girildiğinde 4 cm >= 0.03 eşiği aşılır
    v, w, durum, bilgi = denetleyici.adim(
        robot_odom_pose=(0.23, 0.04, 0.0),
        t=0.0,
        olcum_robot_xy=(0.22, -0.04),
    )
    assert durum == 'HATA:yanal'
    assert v == 0.0
    assert w == 0.0
    assert abs(bilgi['e_yanal']) == pytest.approx(0.04, abs=1e-4)


def test_kapali_cevrim_son_bant_yanal_hata():
    """Kapalı çevrim simülasyonda son bant yanal hata senaryoları."""
    # 1. 2 cm yanal hata ile yaklaşma -> TAMAM
    ayar_genis = SonYaklasmaAyar(yanal_kabul_genis=True)
    durum_2cm, gercek_poz_2cm, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.21, 0.02, math.radians(3.0)),
        nesne_gercek=(0.45, 0.0),
        yaklasma_yaw=0.0,
        ayar=ayar_genis,
        dt=0.02,
    )
    assert durum_2cm == 'TAMAM'

    # 2. 4 cm yanal hata ile yaklaşma -> HATA:yanal
    ayar_esik = SonYaklasmaAyar(son_donus_mesafe=0.05, yanal_hata_esik=0.03)
    durum_4cm, _, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.21, 0.04, 0.0),
        nesne_gercek=(0.45, 0.0),
        yaklasma_yaw=0.0,
        ayar=ayar_esik,
        dt=0.02,
    )
    assert durum_4cm == 'HATA:yanal'


def test_gorus_koruma_kerteriz_25_dereceyi_asmaz_ve_tamam():
    """Başlangıçta nesne yaklaşma yönüne göre 30° yandayken kerteriz hiçbir adımda

    25°'yi (gorus_yari_aci - gorus_payi) aşmaz ve son yaklaşma TAMAM ile sonuçlanır.
    """
    ayar = SonYaklasmaAyar(
        gorus_yari_aci=math.radians(35.0),
        gorus_payi=math.radians(10.0),
    )
    b_sinir = ayar.gorus_yari_aci - ayar.gorus_payi  # 25° = 0.4363 rad

    # 1. Pozitif 30° yaklaşma açısı
    nesne_gercek = (0.50, 0.0)
    yaklasma_yaw = math.radians(30.0)
    tum_kerterizler = []

    def kerteriz_kontrol_pos(k, t, gercek_poz, odom_poz, v, w, durum, denetleyici):
        rx, ry, ryaw = gercek_poz
        th_nesne = math.atan2(nesne_gercek[1] - ry, nesne_gercek[0] - rx)
        b = wrap(th_nesne - ryaw)
        tum_kerterizler.append(abs(b))
        # Hiçbir adımda kerteriz 25°'yi (b_sinir) aşmamalı
        assert abs(b) <= (b_sinir + 1e-4), (
            f"Adım {k}: kerteriz sınırı aşıldı! |b|={math.degrees(abs(b)):.2f}° > "
            f"{math.degrees(b_sinir):.2f}° (v={v:.2f}, w={w:.2f})"
        )

    durum_pos, gercek_son_pos, _, adim_pos = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.0, 0.0),
        nesne_gercek=nesne_gercek,
        yaklasma_yaw=yaklasma_yaw,
        ayar=ayar,
        dt=0.02,
        adim_cb=kerteriz_kontrol_pos,
    )
    assert durum_pos == 'TAMAM', f"Durum TAMAM olmalıydı, alınan: {durum_pos}"
    assert len(tum_kerterizler) > 0
    assert max(tum_kerterizler) <= (b_sinir + 1e-4)

    # 2. Negatif -30° yaklaşma açısı (simetri doğrulaması)
    yaklasma_yaw_neg = math.radians(-30.0)
    tum_kerterizler_neg = []

    def kerteriz_kontrol_neg(k, t, gercek_poz, odom_poz, v, w, durum, denetleyici):
        rx, ry, ryaw = gercek_poz
        th_nesne = math.atan2(nesne_gercek[1] - ry, nesne_gercek[0] - rx)
        b = wrap(th_nesne - ryaw)
        tum_kerterizler_neg.append(abs(b))
        assert abs(b) <= (b_sinir + 1e-4), (
            f"Adım {k}: negatif yönde kerteriz sınırı aşıldı! |b|={math.degrees(abs(b)):.2f}° > "
            f"{math.degrees(b_sinir):.2f}°"
        )

    durum_neg, gercek_son_neg, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.0, 0.0),
        nesne_gercek=nesne_gercek,
        yaklasma_yaw=yaklasma_yaw_neg,
        ayar=ayar,
        dt=0.02,
        adim_cb=kerteriz_kontrol_neg,
    )
    assert durum_neg == 'TAMAM', f"Durum TAMAM olmalıydı, alınan: {durum_neg}"
    assert len(tum_kerterizler_neg) > 0
    assert max(tum_kerterizler_neg) <= (b_sinir + 1e-4)


def test_olcum_kesilmesi_0_2m_tamam_0_4m_hata():
    """Ölçüm rho=0.20 m'de (<= olcumsuz_mesafe 0.25 m) kesilirse odomla TAMAM;

    rho=0.40 m'de (> olcumsuz_mesafe 0.25 m) kesilirse 'HATA:zaman_asimi'.
    """
    ayar = SonYaklasmaAyar(olcum_zaman_asimi=1.0, olcumsuz_mesafe=0.25)

    # 1. rho=0.20 m'de kesilme: olcumsuz_mesafe (0.25 m) altında kaldığı için odomla TAMAM
    durum_20cm, gercek_20cm, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.08, math.radians(15.0)),
        nesne_gercek=(0.45, 0.0),
        yaklasma_yaw=0.0,
        ayar=ayar,
        dt=0.02,
        olcum_kesilme_rho=0.20,  # rho <= 0.20 m iken ölçüm kesilir
    )
    assert durum_20cm == 'TAMAM', f"0.20 m'de kesilme TAMAM olmalıydı, alınan: {durum_20cm}"
    hedef = hedef_pozu((0.45, 0.0), 0.0, ayar)
    e_xy = math.hypot(hedef[0] - gercek_20cm[0], hedef[1] - gercek_20cm[1])
    assert e_xy < 0.025

    # 2. rho=0.40 m'de kesilme: olcumsuz_mesafe (0.25 m) aşıldığı için zaman aşımı hatası
    durum_40cm, _, _, _ = kapali_cevrim_simule(
        robot_poz_0=(0.0, 0.0, 0.0),
        nesne_gercek=(0.70, 0.0),
        yaklasma_yaw=0.0,
        ayar=ayar,
        dt=0.02,
        erken_kesilme_rho=0.40,  # rho <= 0.40 m iken ölçüm kesilir
    )
    assert durum_40cm.startswith('HATA'), f"0.40 m'de kesilme HATA vermeliydi, alınan: {durum_40cm}"
    assert 'zaman_asimi' in durum_40cm


def test_dondurma_rho_kucuk_sicrama_hata_uretmez():
    """(1) rho<0.25'te gelen 3 adet 0.5 m sıçramalı ölçüm HATA:sapma ÜRETMEZ ve komut hedefe doğru sürer."""
    ayar = SonYaklasmaAyar(olcumsuz_mesafe=0.25, sapma_esik=0.15, aykiri_kabul_sayisi=3)
    # Hedef nesne (0.40, 0.0), yaklasma_yaw=0.0 -> P = (0.40 - 0.22, 0.0) = (0.18, 0.0)
    # Robot (0.0, 0.0, 0.0)'da iken rho = 0.18 m (< 0.25 m)
    denetleyici = SonYaklasma(ayar=ayar, yaklasma_yaw_odom=0.0)

    # İlk geçerli ölçüm: nesne robotun önünde 0.40 m'de
    v0, w0, durum0, bilgi0 = denetleyici.adim(
        robot_odom_pose=(0.0, 0.0, 0.0),
        t=0.0,
        olcum_robot_xy=(0.40, 0.0),
    )
    assert durum0 in ('YAKLASIYOR', 'DONUYOR')
    assert bilgi0['rho'] < 0.25
    assert bilgi0['tahmin_donduruldu'] is True
    assert v0 > 0.0

    # 3 adet 0.50 m sıçramalı aykırı ölçüm gönderelim (nesne 0.40 -> 0.90 m sıçradı)
    for i in range(1, 4):
        v, w, durum, bilgi = denetleyici.adim(
            robot_odom_pose=(0.0, 0.0, 0.0),
            t=0.1 * i,
            olcum_robot_xy=(0.90, 0.0),  # 0.50 m sıçrama
        )
        assert durum != 'HATA:sapma'
        assert durum in ('YAKLASIYOR', 'DONUYOR')
        assert v > 0.0
        assert bilgi['tahmin_donduruldu'] is True
        # Tahmin dondurulduğu için nesne odom konumu sıçramaz, korunur
        assert bilgi['nesne_odom'][0] == pytest.approx(0.40, abs=1e-4)


def test_dondurma_rho_buyuk_sicrama_hata_sapma_uretir():
    """(2) rho>0.25'te aynı dizi (3 adet 0.5 m sıçrama) hâlâ HATA:sapma üretir (mevcut davranış korunur)."""
    ayar = SonYaklasmaAyar(olcumsuz_mesafe=0.25, sapma_esik=0.15, aykiri_kabul_sayisi=3)
    # Hedef nesne (0.60, 0.0), yaklasma_yaw=0.0 -> P = (0.60 - 0.22, 0.0) = (0.38, 0.0)
    # Robot (0.0, 0.0, 0.0)'da iken rho = 0.38 m (> 0.25 m)
    denetleyici = SonYaklasma(ayar=ayar, yaklasma_yaw_odom=0.0)

    # İlk geçerli ölçüm: nesne robotun önünde 0.60 m'de
    v0, w0, durum0, bilgi0 = denetleyici.adim(
        robot_odom_pose=(0.0, 0.0, 0.0),
        t=0.0,
        olcum_robot_xy=(0.60, 0.0),
    )
    assert durum0 in ('YAKLASIYOR', 'DONUYOR')
    assert bilgi0['rho'] > 0.25
    assert bilgi0['tahmin_donduruldu'] is False

    # 1. ve 2. sıçramalı ölçüm (reddedilir)
    for i in (1, 2):
        v, w, durum, bilgi = denetleyici.adim(
            robot_odom_pose=(0.0, 0.0, 0.0),
            t=0.1 * i,
            olcum_robot_xy=(1.10, 0.0),  # 0.50 m sıçrama
        )
        assert durum != 'HATA:sapma'
        assert bilgi['tahmin_donduruldu'] is False

    # 3. sıçramalı ölçüm: aykırı kabul eşiği dolduğunda sapma_esik (0.15 m) aşıldığı için HATA:sapma üretir
    v3, w3, durum3, bilgi3 = denetleyici.adim(
        robot_odom_pose=(0.0, 0.0, 0.0),
        t=0.3,
        olcum_robot_xy=(1.10, 0.0),
    )
    assert durum3 == 'HATA:sapma'
    assert v3 == 0.0
    assert w3 == 0.0


def test_renk_algi_kenar_ve_boyut_reddi():
    """(3) renk_algi kenar ve boyut reddi işlevleri doğrulanır."""
    from limo_autonomy.renk_algi import boyut_uygun_mu, kenara_yakin_mi

    # Kenar reddi: alt kenara (720 px) 4 px veya daha yakın sınırlayıcı kutu
    kutu_alt = (600, 717, 40, 3)  # y + h = 720 >= 716
    assert kenara_yakin_mi(kutu_alt, (720, 1280), kenar_payi_px=4)

    # Ortadaki kutu kenara yakın değildir
    kutu_orta = (600, 300, 40, 40)
    assert not kenara_yakin_mi(kutu_orta, (720, 1280), kenar_payi_px=4)

    # Boyut reddi: derinlik penceresi arka plana düşünce (örn. 0.70 m yerine 0.20 m)
    fx = 908.97
    r_px_beklenen = 0.035 * fx / 0.20
    assert not boyut_uygun_mu(r_px_beklenen, derinlik_m=0.70, fx=fx, beklenen_yaricap_m=0.035)
    assert boyut_uygun_mu(r_px_beklenen, derinlik_m=0.20, fx=fx, beklenen_yaricap_m=0.035)


def test_nesne_tahmini_aykiri_kabul_sayisi_parametresi():
    """NesneTahmini'nde aykiri_kabul_sayisi parametresinin davranışı test edilir."""
    # aykiri_kabul_sayisi = 4 olarak ayarlandığında 3 ardışık aykırı yetersiz kalmalı, 4.'de kabul edilmeli
    tahmin = NesneTahmini(alfa=0.4, esik_aykiri=0.10, aykiri_kabul_sayisi=4)
    assert tahmin.guncelle((1.0, 1.0), t=0.0) is True

    # 1, 2 ve 3. aykırılar reddedilmeli
    assert tahmin.guncelle((2.0, 1.0), t=0.1) is False
    assert tahmin.guncelle((2.0, 1.0), t=0.2) is False
    assert tahmin.guncelle((2.0, 1.0), t=0.3) is False
    assert tahmin.konum == (1.0, 1.0)

    # 4. aykırıda kabul edilmeli
    assert tahmin.guncelle((2.0, 1.0), t=0.4) is True
    assert tahmin.konum == (2.0, 1.0)


