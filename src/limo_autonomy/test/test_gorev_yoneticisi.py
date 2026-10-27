"""limo_autonomy.gorev_yoneticisi testleri — ROS'tan bağımsız karar katmanı durum makinesi.

Test kapsamı:
  - Mutlu yol (BOSTA -> KAVRAMAYA_GIT -> KOL_KAVRA -> TASI -> KOL_BIRAK -> EVE_DON -> BOSTA TAMAM)
  - Kavrama ilk HATA sonra HAZIR
  - Kavrama hata hakkı tükenince BASARISIZ
  - Kol kavra başarısız olunca kavramayı yeniden deneme
  - Taşıma 3 kez başarısız -> BASARISIZ ve KOL_BIRAK yok, eve dönmeyi dene
  - İptal ortada (kavramada veya taşımada)
  - Bilinmeyen nesne adı -> 'HATA bilinmeyen nesne'
  - Aşama zaman aşımları (kavrama ve taşıma)
  - YAML yapılandırması yükleme ve bırakma pozu varsayılanı (yoksa ev)
"""

import pytest

from limo_autonomy.gorev_yoneticisi import (
    Ayar,
    Eylem,
    GorevYoneticisi,
    NesneKonumu,
    Poz2D,
    load_nesneler,
)

NESNELER = {
    'elma': NesneKonumu(x=0.23, y=1.40, z=0.75),
    'kutu': NesneKonumu(x=3.73, y=2.18, z=0.30),
}
EV = Poz2D(x=0.0, y=0.0, yaw_derece=0.0)
BIRAKMA = Poz2D(x=1.0, y=0.5, yaw_derece=90.0)


def mk(**kw):
    args = dict(
        nesneler=NESNELER,
        ev=EV,
        birakma=BIRAKMA,
        ayar=Ayar(
            kavrama_zaman_asimi=240.0,
            tasima_zaman_asimi=180.0,
            kol_zaman_asimi=30.0,
            eve_don_zaman_asimi=180.0,
            max_kavrama_yeniden_dene=1,
            max_tasima_yeniden_dene=2,
        ),
    )
    args.update(kw)
    return GorevYoneticisi(**args)


# ------------------------------------------------------------------ 1. Mutlu Yol

def test_mutlu_yol():
    """Uçtan uca başarılı görev akışı."""
    gy = mk()
    assert gy.durum == 'BOSTA'

    # Adım 1: İstek -> KAVRAMAYA_GIT
    eylemler = gy.istek('elma', t=0.0)
    assert gy.durum == 'KAVRAMAYA_GIT'
    assert any(e.tur == 'durum' and 'KAVRAMAYA_GIT' in e.veri for e in eylemler)
    assert any(e.tur == 'kavrama_hedef' and e.veri == NESNELER['elma'] for e in eylemler)

    # Ara kavrama durumları durumu değiştirmemeli
    ara = gy.olay('kavrama_durum', 'NAV Q -> P', t=10.0)
    assert ara == []
    assert gy.durum == 'KAVRAMAYA_GIT'

    # Adım 2: Kavrama HAZIR -> KOL_KAVRA
    eylemler = gy.olay('kavrama_durum', 'HAZIR nesne robotta (0.22, 0.0, 0.75)', t=25.0)
    assert gy.durum == 'KOL_KAVRA'
    assert any(e.tur == 'durum' and e.veri == 'KOL_KAVRA' for e in eylemler)
    assert any(e.tur == 'kol_kavra' for e in eylemler)

    # Adım 3: Kol kavrama başarılı -> TASI
    eylemler = gy.olay('kol_sonuc', True, t=28.0)
    assert gy.durum == 'TASI'
    assert any(e.tur == 'durum' and e.veri == 'TASI' for e in eylemler)
    assert any(e.tur == 'nav_git' and e.veri == BIRAKMA for e in eylemler)

    # Adım 4: Taşıma başarılı -> KOL_BIRAK
    eylemler = gy.olay('nav_sonuc', True, t=60.0)
    assert gy.durum == 'KOL_BIRAK'
    assert any(e.tur == 'durum' and e.veri == 'KOL_BIRAK' for e in eylemler)
    assert any(e.tur == 'kol_birak' for e in eylemler)

    # Adım 5: Kol bırakma başarılı -> EVE_DON
    eylemler = gy.olay('kol_sonuc', True, t=63.0)
    assert gy.durum == 'EVE_DON'
    assert any(e.tur == 'durum' and e.veri == 'EVE_DON' for e in eylemler)
    assert any(e.tur == 'nav_git' and e.veri == EV for e in eylemler)

    # Adım 6: Eve varış başarılı -> BOSTA, TAMAM
    eylemler = gy.olay('nav_sonuc', True, t=90.0)
    assert gy.durum == 'BOSTA'
    assert any(e.tur == 'durum' and 'TAMAM' in e.veri for e in eylemler)

    # Geçmiş listesi kontrolü: (zaman, eski, yeni, neden)
    assert len(gy.gecmis) >= 6
    for kayit in gy.gecmis:
        assert len(kayit) == 4
        zaman, eski, yeni, neden = kayit
        assert isinstance(zaman, float)
        assert isinstance(eski, str)
        assert isinstance(yeni, str)
        assert isinstance(neden, str)


# ------------------------------------------------------------------ 2. Kavrama Hataları

def test_kavrama_ilk_hata_sonra_hazir():
    """Kavrama ilk denemede HATA verip ikincide HAZIR olunca başarılı şekilde ilerlemeli."""
    gy = mk()
    gy.istek('elma', t=0.0)
    assert gy.durum == 'KAVRAMAYA_GIT'

    # İlk denemede hata geldi -> 1 kez yeniden hedef göndermeli
    eylemler = gy.olay('kavrama_durum', 'HATA: yaklaşma yolu engelli', t=15.0)
    assert gy.durum == 'KAVRAMAYA_GIT'
    assert any(e.tur == 'kavrama_hedef' and e.veri == NESNELER['elma'] for e in eylemler)

    # İkinci denemede HAZIR geldi -> KOL_KAVRA durumuna geçmeli
    eylemler = gy.olay('kavrama_durum', 'HAZIR nesne robotta', t=35.0)
    assert gy.durum == 'KOL_KAVRA'
    assert any(e.tur == 'kol_kavra' for e in eylemler)


def test_kavrama_hatalar_tukendi_basarisiz():
    """Kavrama 1 yeniden deneme hakkı bittikten sonra BASARISIZ kavrama ile BOSTA'ya dönmeli."""
    gy = mk()
    gy.istek('elma', t=0.0)

    # 1. hata -> yeniden dene
    eylemler1 = gy.olay('kavrama_durum', 'HATA: engel', t=10.0)
    assert gy.durum == 'KAVRAMAYA_GIT'
    assert any(e.tur == 'kavrama_hedef' for e in eylemler1)

    # 2. hata -> denemeler tükendi
    eylemler2 = gy.olay('kavrama_durum', 'HATA: engel', t=20.0)
    assert gy.durum == 'BOSTA'
    assert any(e.tur == 'durum' and 'BASARISIZ kavrama' in e.veri for e in eylemler2)


def test_kol_kavra_basarisiz_kavrama_yeniden_dener():
    """Kol kavra başarısız olunca kavramayı 1 kez yeniden denemeli."""
    gy = mk()
    gy.istek('elma', t=0.0)
    gy.olay('kavrama_durum', 'HAZIR', t=20.0)
    assert gy.durum == 'KOL_KAVRA'

    # Kol kavrama başarısız
    eylemler = gy.olay('kol_sonuc', False, t=25.0)
    assert gy.durum == 'KAVRAMAYA_GIT'
    assert any(e.tur == 'kavrama_hedef' for e in eylemler)

    # İkinci denemede de kol kavrama başarısız olursa BASARISIZ kavrama
    gy.olay('kavrama_durum', 'HAZIR', t=45.0)
    assert gy.durum == 'KOL_KAVRA'
    eylemler2 = gy.olay('kol_sonuc', False, t=50.0)
    assert gy.durum == 'BOSTA'
    assert any(e.tur == 'durum' and 'BASARISIZ kavrama' in e.veri for e in eylemler2)


# ------------------------------------------------------------------ 3. Taşıma Başarısızlığı

def test_tasima_uc_kez_basarisiz_kol_birak_yok():
    """Taşıma 3 kez (1 asıl + 2 yeniden) başarısız olunca KOL_BIRAK yapmadan eve dönmeli."""
    gy = mk()
    gy.istek('elma', t=0.0)
    gy.olay('kavrama_durum', 'HAZIR', t=20.0)
    gy.olay('kol_sonuc', True, t=25.0)
    assert gy.durum == 'TASI'

    # 1. başarısızlık (ilk deneme) -> 1. yeniden deneme
    eylemler1 = gy.olay('nav_sonuc', False, t=35.0)
    assert gy.durum == 'TASI'
    assert any(e.tur == 'nav_git' and e.veri == BIRAKMA for e in eylemler1)

    # 2. başarısızlık -> 2. yeniden deneme
    eylemler2 = gy.olay('nav_sonuc', False, t=50.0)
    assert gy.durum == 'TASI'
    assert any(e.tur == 'nav_git' and e.veri == BIRAKMA for e in eylemler2)

    # 3. başarısızlık (2 yeniden deneme de tükendi, toplam 3 başarısızlık)
    # 'BASARISIZ tasima' ve olduğun yerde KOL_BIRAK YAPMA, eve dönmeyi dene.
    eylemler3 = gy.olay('nav_sonuc', False, t=65.0)
    assert gy.durum == 'EVE_DON'
    assert not any(e.tur == 'kol_birak' for e in eylemler3)
    assert any(e.tur == 'durum' and 'BASARISIZ tasima' in e.veri for e in eylemler3)
    assert any(e.tur == 'nav_git' and e.veri == EV for e in eylemler3)

    # Eve varış
    eylemler4 = gy.olay('nav_sonuc', True, t=100.0)
    assert gy.durum == 'BOSTA'
    # Taşıma başarısız bittiği için TAMAM bildirilmemeli
    assert not any(e.tur == 'durum' and e.veri == 'TAMAM' for e in eylemler4)


# ------------------------------------------------------------------ 4. İptal Durumları

def test_iptal_kavramada():
    """Kavrama aşamasında iptal gelince kavrama_iptal üretip BOSTA'ya dönmeli."""
    gy = mk()
    gy.istek('elma', t=0.0)
    assert gy.durum == 'KAVRAMAYA_GIT'

    eylemler = gy.olay('iptal', None, t=10.0)
    assert gy.durum == 'BOSTA'
    assert any(e.tur == 'kavrama_iptal' for e in eylemler)


def test_iptal_tasimada():
    """Taşıma aşamasında /gorev/istek iptal gelince nav_iptal üretip BOSTA'ya dönmeli."""
    gy = mk()
    gy.istek('elma', t=0.0)
    gy.olay('kavrama_durum', 'HAZIR', t=20.0)
    gy.olay('kol_sonuc', True, t=25.0)
    assert gy.durum == 'TASI'

    eylemler = gy.istek('iptal', t=40.0)
    assert gy.durum == 'BOSTA'
    assert any(e.tur == 'nav_iptal' for e in eylemler)


def test_bosta_iken_iptal_guvenli():
    """BOSTA durumundayken iptal istenirse güvenle BOSTA durumunu korumalı."""
    gy = mk()
    assert gy.durum == 'BOSTA'
    eylemler = gy.istek('iptal', t=0.0)
    assert gy.durum == 'BOSTA'
    assert any(e.tur == 'durum' and e.veri == 'BOSTA' for e in eylemler)


# ------------------------------------------------------------------ 5. Bilinmeyen Nesne ve Meşgul

def test_bilinmeyen_nesne():
    """Tanımlı olmayan nesne adı istendiğinde hata bildirip BOSTA kalmalı."""
    gy = mk()
    eylemler = gy.istek('armut', t=0.0)
    assert gy.durum == 'BOSTA'
    assert any(e.tur == 'durum' and 'HATA bilinmeyen nesne' in e.veri for e in eylemler)


def test_mesgulken_yeni_istek_reddedilir():
    """Görev sürerken yeni istek gelirse HATA mesgul bildirilmeli."""
    gy = mk()
    gy.istek('elma', t=0.0)
    assert gy.durum == 'KAVRAMAYA_GIT'

    eylemler = gy.istek('kutu', t=5.0)
    assert gy.durum == 'KAVRAMAYA_GIT'
    assert any(e.tur == 'durum' and 'HATA mesgul' in e.veri for e in eylemler)


# ------------------------------------------------------------------ 6. Zaman Aşımı

def test_zaman_asimi_kavrama():
    """Kavrama aşamasında 240 sn aşılınca önce yeniden denemeli, tekrar aşarsa durdurmalı."""
    gy = mk()
    gy.istek('elma', t=0.0)
    assert gy.durum == 'KAVRAMAYA_GIT'

    # 100 sn sonra henüz zaman aşımı yok
    assert gy.olay('zaman', None, t=100.0) == []
    assert gy.durum == 'KAVRAMAYA_GIT'

    # 241. sn: 1. hak doldu -> kavrama_iptal + yeniden hedef
    eylemler1 = gy.olay('zaman', None, t=241.0)
    assert gy.durum == 'KAVRAMAYA_GIT'
    assert any(e.tur == 'kavrama_iptal' for e in eylemler1)
    assert any(e.tur == 'kavrama_hedef' for e in eylemler1)

    # 241 + 241 = 482. sn: 2. deneme de zaman aşımına uğradı -> BASARISIZ kavrama
    eylemler2 = gy.olay('zaman', None, t=482.0)
    assert gy.durum == 'BOSTA'
    assert any(e.tur == 'kavrama_iptal' for e in eylemler2)
    assert any(e.tur == 'durum' and 'BASARISIZ kavrama' in e.veri for e in eylemler2)


def test_zaman_asimi_tasima():
    """Taşıma aşamasında 180 sn zaman aşımı politikasının işletilmesi."""
    gy = mk()
    gy.istek('elma', t=0.0)
    gy.olay('kavrama_durum', 'HAZIR', t=10.0)
    gy.olay('kol_sonuc', True, t=15.0)
    assert gy.durum == 'TASI'

    # Taşıma t=15'te başladı; 181 sn sonra (t=196) 1. yeniden deneme
    eylemler1 = gy.olay('zaman', None, t=196.0)
    assert gy.durum == 'TASI'
    assert any(e.tur == 'nav_iptal' for e in eylemler1)
    assert any(e.tur == 'nav_git' for e in eylemler1)

    # 2. deneme de 181 sn sürerse (t=377) -> 2. yeniden deneme
    eylemler2 = gy.olay('zaman', None, t=377.0)
    assert gy.durum == 'TASI'

    # 3. deneme de aşarsa (t=558) -> haklar bitti, eve dönmeli, KOL_BIRAK yok
    eylemler3 = gy.olay('zaman', None, t=558.0)
    assert gy.durum == 'EVE_DON'
    assert not any(e.tur == 'kol_birak' for e in eylemler3)
    assert any(e.tur == 'durum' and 'BASARISIZ tasima' in e.veri for e in eylemler3)
    assert any(e.tur == 'nav_git' and e.veri == EV for e in eylemler3)


# ------------------------------------------------------------------ 7. YAML Yükleme

def test_load_nesneler(tmp_path):
    """YAML dosyasından konfigürasyon yükleme doğrulaması."""
    p = tmp_path / 'nesneler.yaml'
    p.write_text("""
ev: {x: 1.0, y: 2.0, yaw_derece: 45.0}
birakma: {x: 3.0, y: 4.0, yaw_derece: -90.0}
nesneler:
  elma: {x: 0.5, y: 1.5, z: 0.8}
  kutu: {x: 2.0, y: 3.0, z: 0.4}
""")
    cfg = load_nesneler(str(p))
    assert cfg['ev'] == Poz2D(1.0, 2.0, 45.0)
    assert cfg['birakma'] == Poz2D(3.0, 4.0, -90.0)
    assert 'elma' in cfg['nesneler']
    assert cfg['nesneler']['elma'] == NesneKonumu(0.5, 1.5, 0.8)
    assert cfg['nesneler']['kutu'] == NesneKonumu(2.0, 3.0, 0.4)


def test_load_nesneler_birakma_yoksa_ev_olur(tmp_path):
    """YAML dosyasında bırakma alanı yoksa ev pozu varsayılan olmalıdır."""
    p = tmp_path / 'nesneler_no_birakma.yaml'
    p.write_text("""
ev: {x: 0.5, y: -0.5, yaw_derece: 180.0}
nesneler:
  elma: {x: 1.0, y: 1.0, z: 0.5}
""")
    cfg = load_nesneler(str(p))
    assert cfg['birakma'] == cfg['ev']


def test_load_nesneler_hatali_format(tmp_path):
    """Eksik veya hatalı alanlar ValueError fırlatmalı."""
    p = tmp_path / 'bad.yaml'
    p.write_text("ev: {x: 0.0}\n")  # y eksik, nesneler eksik
    with pytest.raises(ValueError):
        load_nesneler(str(p))
