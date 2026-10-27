"""Kavra ROS 2 eylem (action) saf yardımcı fonksiyon birim testleri (ROS'suz saf Python).

Test kapsamı:
  1. Eylem hedefinden iç parametrelere dönüşüm (yon_derece NaN -> None, derece -> radyan, Ayar.yon, algisiz_izinli)
  2. Sonuç doldurma yardımcısı (HAZIR / HATA metninden basarili, neden, nesne_robotta, hata_cm, yon_hatasi, sure_s)
  3. Geri bildirim kısıtlayıcı (durum değişimlerinde anında, SON_YAKLASMA'da 5 Hz kısıtlama)
  4. Görev düğümü kavrama_arayuzu seçimi (auto | eylem | konu) ve geri bildirim metni biçimlendirme
"""

import math
from types import SimpleNamespace

import pytest

from limo_autonomy.gorev_dugumu import (
    kavra_geri_bildirim_metni,
    kavrama_arayuzu_sec,
)
from limo_autonomy.kavrama_dugumu import (
    GeriBildirimKisitlayici,
    durum_metni_ayristir,
    durum_metninden_sonuc,
    eylem_hedef_coz,
    geri_bildirim_doldur,
    geri_bildirim_gonderilmeli_mi,
    sonuc_doldur,
    yon_derece_radyana,
)
from limo_autonomy.kavrama_yaklasim import Ayar


# ---------------------------------------------------------------------------
# 1. Eylem Hedefinden İç Parametrelere Dönüşüm Testleri
# ---------------------------------------------------------------------------


def test_yon_derece_radyana_nan_ve_none():
    """yon_derece NaN veya None olduğunda serbest yön (None) dönmelidir."""
    assert yon_derece_radyana(float('nan')) is None
    assert yon_derece_radyana(None) is None
    assert yon_derece_radyana(float('inf')) is None
    assert yon_derece_radyana(float('-inf')) is None


def test_yon_derece_radyana_sayisal():
    """yon_derece geçerli bir açı olduğunda radyan karşılığı dönmelidir."""
    assert yon_derece_radyana(0.0) == pytest.approx(0.0, abs=1e-9)
    assert yon_derece_radyana(90.0) == pytest.approx(math.pi / 2.0, abs=1e-7)
    assert yon_derece_radyana(-90.0) == pytest.approx(-math.pi / 2.0, abs=1e-7)
    assert yon_derece_radyana(180) == pytest.approx(math.pi, abs=1e-7)
    assert yon_derece_radyana(-45.0) == pytest.approx(math.radians(-45.0), abs=1e-7)


def test_eylem_hedef_coz_nan_yon_ve_ayar_guncelleme():
    """Hedef yon_derece NaN iken yon_rad None olmalı ve Ayar.yon None olarak güncellenmeli."""
    sahte_hedef = SimpleNamespace(
        nesne=SimpleNamespace(
            header=SimpleNamespace(frame_id='map'),
            point=SimpleNamespace(x=3.73, y=2.18, z=0.30),
        ),
        yon_derece=float('nan'),
        algisiz_izinli=True,
    )
    ayar = Ayar(yon=0.5)  # önceden ayarlanmış bir yön bulunsa dahi sıfırlanmalı
    cozulen = eylem_hedef_coz(sahte_hedef, ayar=ayar)

    assert cozulen['nesne_xyz'] == pytest.approx((3.73, 2.18, 0.30))
    assert cozulen['frame_id'] == 'map'
    assert cozulen['yon_rad'] is None
    assert cozulen['yon'] is None
    assert cozulen['algisiz_izinli'] is True
    assert ayar.yon is None


def test_eylem_hedef_coz_belirli_yon_dereceden_radyana():
    """Hedef yon_derece verildiğinde (NaN değilse) Ayar.yon'a radyan olarak yazılmalı."""
    sahte_hedef = SimpleNamespace(
        nesne=SimpleNamespace(
            header=SimpleNamespace(frame_id='odom'),
            point=SimpleNamespace(x=1.25, y=-0.80, z=0.25),
        ),
        yon_derece=60.0,
        algisiz_izinli=False,
    )
    ayar = Ayar(yon=None)
    cozulen = eylem_hedef_coz(sahte_hedef, ayar=ayar)

    beklenen_rad = math.radians(60.0)
    assert cozulen['nesne_xyz'] == pytest.approx((1.25, -0.80, 0.25))
    assert cozulen['frame_id'] == 'odom'
    assert cozulen['yon_rad'] == pytest.approx(beklenen_rad, abs=1e-7)
    assert cozulen['algisiz_izinli'] is False
    assert ayar.yon == pytest.approx(beklenen_rad, abs=1e-7)


def test_eylem_hedef_coz_sozluk_ve_tuple_girdisi():
    """eylem_hedef_coz sözlük ve tuple girdileriyle de çalışabilmeli."""
    sozluk_hedef = {
        'nesne': {
            'header': {'frame_id': 'map'},
            'point': {'x': 3.73, 'y': 2.18, 'z': 0.3},
        },
        'yon_derece': -30.0,
        'algisiz_izinli': True,
    }
    c1 = eylem_hedef_coz(sozluk_hedef)
    assert c1['nesne_xyz'] == pytest.approx((3.73, 2.18, 0.3))
    assert c1['yon_rad'] == pytest.approx(math.radians(-30.0))
    assert c1['algisiz_izinli'] is True

    c2 = eylem_hedef_coz((0.5, 1.5, 0.2), yon_derece=float('nan'), algisiz_izinli=False)
    assert c2['nesne_xyz'] == pytest.approx((0.5, 1.5, 0.2))
    assert c2['yon_rad'] is None
    assert c2['algisiz_izinli'] is False


# ---------------------------------------------------------------------------
# 2. Sonuç Doldurma Yardımcısı Testleri
# ---------------------------------------------------------------------------


def test_durum_metninden_sonuc_hazir_ve_hata():
    """HAZIR ve HATA metinlerinden (basarili, neden) doğru ayrıştırılmalı."""
    basarili, neden = durum_metninden_sonuc(
        'HAZIR nesne robotta (+0.220, +0.000, 0.30) m; P hatası 0.8 cm, 1.2°; sure 4.50 s'
    )
    assert basarili is True
    assert neden == 'HAZIR'

    basarili2, neden2 = durum_metninden_sonuc('HAZIR')
    assert basarili2 is True
    assert neden2 == 'HAZIR'

    basarili3, neden3 = durum_metninden_sonuc('HATA tüm adaylar denendi')
    assert basarili3 is False
    assert neden3 == 'HATA tüm adaylar denendi'

    basarili4, neden4 = durum_metninden_sonuc('HATA iptal')
    assert basarili4 is False
    assert neden4 == 'HATA iptal'

    basarili5, neden5 = durum_metninden_sonuc('erisilemez yukseklik z=0.75')
    assert basarili5 is False
    assert neden5 == 'HATA erisilemez yukseklik z=0.75'


def test_sonuc_doldur_hazir_metninden_otomatik_ayristirma():
    """HAZIR durum metnindeki nesne_robotta, P hatası (cm, derece) ve süre otomatik çözülmeli."""
    metin = 'HAZIR nesne robotta (+0.218, -0.004, 0.30) m; P hatası 0.9 cm, 1.4°; sure 3.75 s'
    sonuc = sonuc_doldur(metin)

    assert sonuc.basarili is True
    assert sonuc.neden == 'HAZIR'
    assert sonuc.tahmini_hata_cm == pytest.approx(0.9, abs=1e-5)
    assert sonuc.tahmini_yon_hatasi_derece == pytest.approx(1.4, abs=1e-5)
    assert sonuc.sure_s == pytest.approx(3.75, abs=1e-5)
    assert sonuc.nesne_robotta.header.frame_id == 'base_footprint'
    assert sonuc.nesne_robotta.point.x == pytest.approx(0.218, abs=1e-5)
    assert sonuc.nesne_robotta.point.y == pytest.approx(-0.004, abs=1e-5)
    assert sonuc.nesne_robotta.point.z == pytest.approx(0.30, abs=1e-5)


def test_sonuc_doldur_mevcut_sonuc_nesnesi_uzerine():
    """Verilen sahte Kavra.Result nesnesinin alanları yerinde doldurulmalı."""
    sahte_result = SimpleNamespace(
        basarili=False,
        neden='',
        nesne_robotta=SimpleNamespace(
            header=SimpleNamespace(frame_id=''),
            point=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        tahmini_hata_cm=0.0,
        tahmini_yon_hatasi_derece=0.0,
        sure_s=0.0,
    )

    out = sonuc_doldur(
        'HAZIR',
        sahte_result,
        nesne_robotta_xyz=(0.22, 0.01, 0.30),
        tahmini_hata_cm=0.6,
        tahmini_yon_hatasi_derece=0.8,
        sure_s=5.2,
    )
    assert out is sahte_result
    assert sahte_result.basarili is True
    assert sahte_result.neden == 'HAZIR'
    assert sahte_result.nesne_robotta.header.frame_id == 'base_footprint'
    assert sahte_result.nesne_robotta.point.x == pytest.approx(0.22)
    assert sahte_result.nesne_robotta.point.y == pytest.approx(0.01)
    assert sahte_result.nesne_robotta.point.z == pytest.approx(0.30)
    assert sahte_result.tahmini_hata_cm == pytest.approx(0.6)
    assert sahte_result.tahmini_yon_hatasi_derece == pytest.approx(0.8)
    assert sahte_result.sure_s == pytest.approx(5.2)


def test_sonuc_doldur_hata_durumu():
    """HATA durumunda basarili=False ve neden='HATA <ayrıntı>' olmalı."""
    sonuc = sonuc_doldur('HATA geçerli yaklaşma yok (engel / erişim)', sure_s=1.25)
    assert sonuc.basarili is False
    assert sonuc.neden == 'HATA geçerli yaklaşma yok (engel / erişim)'
    assert sonuc.tahmini_hata_cm == 0.0
    assert sonuc.tahmini_yon_hatasi_derece == 0.0
    assert sonuc.sure_s == pytest.approx(1.25)


# ---------------------------------------------------------------------------
# 3. Geri Bildirim Kısıtlayıcı (5 Hz) ve Ayrıştırıcı Testleri
# ---------------------------------------------------------------------------


def test_geri_bildirim_gonderilmeli_mi_saf_fonksiyon():
    """geri_bildirim_gonderilmeli_mi durum değişiminde anında, aynı durumda 5 Hz (0.2 s) izin vermeli."""
    # İlk gönderim (son_gonderim_s=None) -> True
    assert geri_bildirim_gonderilmeli_mi(0.0, None, 'PLAN', None, frekans_hz=5.0) is True

    # Aynı durumda 0.05 s ve 0.19 s sonra (< 0.20 s) -> False
    assert geri_bildirim_gonderilmeli_mi(0.05, 0.0, 'SON_YAKLASMA', 'SON_YAKLASMA', frekans_hz=5.0) is False
    assert geri_bildirim_gonderilmeli_mi(0.19, 0.0, 'SON_YAKLASMA', 'SON_YAKLASMA', frekans_hz=5.0) is False

    # Aynı durumda 0.20 s sonra (>= 0.20 s) -> True
    assert geri_bildirim_gonderilmeli_mi(0.20, 0.0, 'SON_YAKLASMA', 'SON_YAKLASMA', frekans_hz=5.0) is True

    # Süre dolmamış olsa bile durum değiştiğinde (ör. NAV_Q -> SON_YAKLASMA) anında True
    assert geri_bildirim_gonderilmeli_mi(0.02, 0.0, 'SON_YAKLASMA', 'NAV_Q', frekans_hz=5.0) is True


def test_geri_bildirim_kisitlayici_50hz_dongude_5hz_cikis():
    """50 Hz (dt=0.02 s) çalışan SON_YAKLASMA döngüsünde kısıtlayıcı 5 Hz (her 0.20 s'de bir) izin vermeli."""
    kisitlayici = GeriBildirimKisitlayici(frekans_hz=5.0)

    # Durum geçişleri anında izin almalı
    assert kisitlayici.izin_var_mi(0.00, 'PLAN') is True
    assert kisitlayici.izin_var_mi(0.01, 'NAV_Q') is True
    assert kisitlayici.izin_var_mi(1.00, 'SON_YAKLASMA') is True

    # t=1.02 .. 2.00 arasında 50 adım (1.00 sn süre): her 10 adımda (0.20 sn) bir izin verilmeli -> 5 kez
    izin_zamanlari = []
    for i in range(1, 51):
        t = 1.00 + i * 0.02
        if kisitlayici.izin_var_mi(t, 'SON_YAKLASMA'):
            izin_zamanlari.append(round(t, 2))

    assert izin_zamanlari == [1.20, 1.40, 1.60, 1.80, 2.00]

    # SON_YAKLASMA -> KURTARMA geçişi t=2.02'de anında izin almalı
    assert kisitlayici.izin_var_mi(2.02, 'KURTARMA') is True


def test_geri_bildirim_kisitlayici_sifirla():
    """sifirla() çağrıldığında kısıtlayıcı geçmişi temizlenmeli."""
    k = GeriBildirimKisitlayici(frekans_hz=5.0)
    assert k.izin_var_mi(10.0, 'SON_YAKLASMA') is True
    assert k.izin_var_mi(10.05, 'SON_YAKLASMA') is False

    k.sifirla()
    assert k.son_zaman_s is None
    assert k.son_durum is None
    assert k.izin_var_mi(10.05, 'SON_YAKLASMA') is True


def test_durum_metni_ayristir_ve_geri_bildirim_doldur():
    """Durum metni ayrıştırma ve Kavra.Feedback doldurma yardımcıları."""
    durum, ayrinti = durum_metni_ayristir('SON_YAKLASMA v=0.08 w=0.02 rho=0.14 m')
    assert durum == 'SON_YAKLASMA'
    assert ayrinti == 'v=0.08 w=0.02 rho=0.14 m'

    fb = geri_bildirim_doldur(
        durum=durum,
        ayrinti=ayrinti,
        aday_no=1,
        aday_sayisi=3,
        kalan_mesafe_m=0.14,
    )
    assert fb.durum == 'SON_YAKLASMA'
    assert fb.ayrinti == 'v=0.08 w=0.02 rho=0.14 m'
    assert fb.aday_no == 1
    assert fb.aday_sayisi == 3
    assert fb.kalan_mesafe_m == pytest.approx(0.14)


# ---------------------------------------------------------------------------
# 4. Görev Düğümü Kavrama Arayüzü Seçimi ve Geri Bildirim Metni Testleri
# ---------------------------------------------------------------------------


def test_kavrama_arayuzu_sec_auto_eylem_konu():
    """kavrama_arayuzu parametresine (auto|eylem|konu) göre arayüz seçimi ve 5 s yedek düşüşü."""
    # auto: sunucu hazırsa eylem, değilse konu arayüzüne düşer
    assert kavrama_arayuzu_sec('auto', eylem_paketi_var=True, sunucu_hazir_mi=True) == 'eylem'
    assert kavrama_arayuzu_sec('auto', eylem_paketi_var=True, sunucu_hazir_mi=False) == 'konu'
    assert kavrama_arayuzu_sec('auto', eylem_paketi_var=False, sunucu_hazir_mi=False) == 'konu'

    # eylem: sunucu hazırsa eylem, yoksa hata_eylem_yok
    assert kavrama_arayuzu_sec('eylem', eylem_paketi_var=True, sunucu_hazir_mi=True) == 'eylem'
    assert kavrama_arayuzu_sec('eylem', eylem_paketi_var=True, sunucu_hazir_mi=False) == 'hata_eylem_yok'

    # konu: her zaman konu
    assert kavrama_arayuzu_sec('konu', eylem_paketi_var=True, sunucu_hazir_mi=True) == 'konu'
    assert kavrama_arayuzu_sec('konu', eylem_paketi_var=False, sunucu_hazir_mi=False) == 'konu'


def test_kavra_geri_bildirim_metni():
    """Kavra eylem geri bildiriminin /gorev/durum için biçimlendirilmesi."""
    fb = SimpleNamespace(
        durum='SON_YAKLASMA',
        ayrinti='v=0.06 w=0.01 rho=0.12 m',
        aday_no=1,
        aday_sayisi=3,
        kalan_mesafe_m=0.12,
    )
    metin = kavra_geri_bildirim_metni(fb)
    assert 'KAVRAMA SON_YAKLASMA' in metin
    assert '(1/3)' in metin
    assert 'kalan=0.12m' in metin
    assert 'rho=0.12 m' in metin
