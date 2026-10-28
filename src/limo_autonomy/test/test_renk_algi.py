"""test_renk_algi: Renk ve derinlik tabanlı nesne algısı saf fonksiyon ve birim testleri."""

from __future__ import annotations

import math
import cv2
import numpy as np
import pytest

from limo_autonomy.renk_algi import (
    boyut_uygun_mu,
    derinlik_gecerli_pikselleri_ayikla,
    derinlik_medyan_al,
    derinlik_saglam_hesapla,
    derinlik_tutarlilik_orani,
    derinlik_tutarlilik_uygun_mu,
    derinlik_yuzdelik_al,
    en_buyuk_bilesen_filtrele,
    fiziksel_yaricap_kestir,
    gorunen_yaricap_px_hesapla,
    goruntu_mesajindan_numpy,
    hsv_maske,
    isin_derinlik_3b,
    k_ayristir,
    kenara_yakin_mi,
    leke_kutusu_ic_sinirlari,
    maske_bul,
    menzil_uygun_mu,
    merkez_bul,
    numpy_dan_goruntu_mesaji,
    piksel_isin,
    quaternion_dan_rotasyon_matrisi,
    renk_derinlik_esle,
    sinirlayici_kutu_bul,
)


# ==============================================================================
# 1. K Ayrıştırma ve Temel Geometri Testleri
# ==============================================================================

def test_k_ayristir_cesitli_formatlar():
    """K iç parametrelerinin 3x3 matris, 9'lu liste ve 4'lü demet olarak ayrıştırılması."""
    # 3x3 matris
    k_mat = np.array([
        [908.97, 0.0, 640.0],
        [0.0, 908.97, 360.0],
        [0.0, 0.0, 1.0],
    ])
    fx, fy, cx, cy = k_ayristir(k_mat)
    assert fx == pytest.approx(908.97, abs=1e-3)
    assert fy == pytest.approx(908.97, abs=1e-3)
    assert cx == pytest.approx(640.0, abs=1e-3)
    assert cy == pytest.approx(360.0, abs=1e-3)

    # 9 elemanlı düz liste (CameraInfo.k)
    k_9 = [908.97, 0.0, 640.0, 0.0, 908.97, 360.0, 0.0, 0.0, 1.0]
    assert k_ayristir(k_9) == (
        pytest.approx(908.97, abs=1e-3),
        pytest.approx(908.97, abs=1e-3),
        pytest.approx(640.0, abs=1e-3),
        pytest.approx(360.0, abs=1e-3),
    )

    # 4 elemanlı demet
    k_4 = (429.73, 429.73, 424.0, 240.0)
    assert k_ayristir(k_4) == (
        pytest.approx(429.73, abs=1e-3),
        pytest.approx(429.73, abs=1e-3),
        pytest.approx(424.0, abs=1e-3),
        pytest.approx(240.0, abs=1e-3),
    )


def test_quaternion_rotasyon_matrisi():
    """Birim quaternion'dan 3x3 birim rotasyon matrisi üretimi doğrulanır."""
    r_id = quaternion_dan_rotasyon_matrisi(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(r_id, np.eye(3), atol=1e-6)

    # Z ekseni etrafında 90 derece dönüş (yaw = pi/2)
    qz = math.sin(math.pi / 4.0)
    qw = math.cos(math.pi / 4.0)
    r_z90 = quaternion_dan_rotasyon_matrisi(0.0, 0.0, qz, qw)
    beklenen = np.array([
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0],
    ])
    np.testing.assert_allclose(r_z90, beklenen, atol=1e-6)


# ==============================================================================
# 2. Renk Maskeleme, Kırmızı Sarması ve Merkez Tespiti Testleri
# ==============================================================================

def test_maske_bul_ve_merkez_sentetik():
    """Siyah zemin üzerinde bilinen konumdaki kırmızı dairenin maskesi ve merkezi bulunur."""
    # 720x1280 siyah görüntü
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    merkez_x, merkez_y = 640, 360
    yaricap = 25
    # OpenCV BGR: kırmızı = (0, 0, 255)
    cv2.circle(img, (merkez_x, merkez_y), yaricap, (0, 0, 255), thickness=-1)

    mask, alan = maske_bul(img, min_alan_px=150)
    assert alan > 150
    # Daire alanı ~ pi * r^2 = 3.14159 * 625 ~= 1963 piksel
    assert alan == pytest.approx(math.pi * yaricap * yaricap, rel=0.10)

    merkez = merkez_bul(mask, min_alan_px=150)
    assert merkez is not None
    cx, cy = merkez
    assert cx == pytest.approx(merkez_x, abs=0.5)
    assert cy == pytest.approx(merkez_y, abs=0.5)


def test_kirmizi_sarmasi_h175_bulunur():
    """HSV uzayında kırmızı sarması test edilir (H=175 ve H=5 tespit edilir, mavi/yeşil reddedilir)."""
    # 1. H=175 kırmızı sarması (üst kırmızı aralığı 170-180)
    img_h175 = np.zeros((200, 200, 3), dtype=np.uint8)
    # HSV: H=175, S=220, V=200
    hsv_renk_175 = np.uint8([[[175, 220, 200]]])
    bgr_renk_175 = cv2.cvtColor(hsv_renk_175, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    cv2.circle(img_h175, (100, 100), 20, tuple(bgr_renk_175), thickness=-1)

    mask_175, alan_175 = maske_bul(img_h175, min_alan_px=150)
    assert alan_175 > 150
    merkez_175 = merkez_bul(mask_175, min_alan_px=150)
    assert merkez_175 is not None
    assert merkez_175[0] == pytest.approx(100.0, abs=1.0)
    assert merkez_175[1] == pytest.approx(100.0, abs=1.0)

    # 2. H=5 kırmızı aralığı (alt kırmızı aralığı 0-10)
    img_h5 = np.zeros((200, 200, 3), dtype=np.uint8)
    hsv_renk_5 = np.uint8([[[5, 220, 200]]])
    bgr_renk_5 = cv2.cvtColor(hsv_renk_5, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    cv2.circle(img_h5, (100, 100), 20, tuple(bgr_renk_5), thickness=-1)

    mask_5, alan_5 = maske_bul(img_h5, min_alan_px=150)
    assert alan_5 > 150
    assert merkez_bul(mask_5, min_alan_px=150) is not None

    # 3. Mavi (H=120) nesne kırmızı maskesinde bulunmamalıdır
    img_mavi = np.zeros((200, 200, 3), dtype=np.uint8)
    hsv_mavi = np.uint8([[[120, 220, 200]]])
    bgr_mavi = cv2.cvtColor(hsv_mavi, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    cv2.circle(img_mavi, (100, 100), 20, tuple(bgr_mavi), thickness=-1)

    mask_mavi, alan_mavi = maske_bul(img_mavi, min_alan_px=150)
    assert alan_mavi == 0
    assert merkez_bul(mask_mavi, min_alan_px=150) is None

    # 4. Yeşil (H=60) nesne de elenmelidir
    img_yesil = np.zeros((200, 200, 3), dtype=np.uint8)
    hsv_yesil = np.uint8([[[60, 220, 200]]])
    bgr_yesil = cv2.cvtColor(hsv_yesil, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    cv2.circle(img_yesil, (100, 100), 20, tuple(bgr_yesil), thickness=-1)

    mask_yesil, alan_yesil = maske_bul(img_yesil, min_alan_px=150)
    assert alan_yesil == 0
    assert merkez_bul(mask_yesil, min_alan_px=150) is None


def test_kucuk_leke_reddi():
    """min_alan_px (150 px) altındaki küçük kırmızı lekelerin elendiği doğrulanır."""
    img = np.zeros((200, 200, 3), dtype=np.uint8)

    # Yarıçapı 4 px olan küçük kırmızı nokta: alan ~ pi * 4^2 ~= 50 piksel (< 150)
    cv2.circle(img, (50, 50), 4, (0, 0, 255), thickness=-1)

    mask, alan = maske_bul(img, min_alan_px=150)
    assert alan == 0
    assert merkez_bul(mask, min_alan_px=150) is None

    # Şimdi yanına yarıçapı 10 px olan geçerli kırmızı daire ekleyelim (~314 piksel)
    cv2.circle(img, (140, 140), 10, (0, 0, 255), thickness=-1)

    mask2, alan2 = maske_bul(img, min_alan_px=150)
    assert alan2 >= 150
    merkez2 = merkez_bul(mask2, min_alan_px=150)
    assert merkez2 is not None
    assert merkez2[0] == pytest.approx(140.0, abs=1.5)
    assert merkez2[1] == pytest.approx(140.0, abs=1.5)


# ==============================================================================
# 3. Pikselden Işına ve Işın+Derinlikten 3B Konum Testleri
# ==============================================================================

def test_piksel_isin_optik_merkez_ve_kenar():
    """Pikselden kamera optik çerçevesindeki görüş ışını doğru hesaplanır."""
    k = (908.97, 908.97, 640.0, 360.0)

    # Optik merkez (640, 360) tam ileriye bakmalıdır: (0, 0, 1)
    isin_merkez = piksel_isin(640.0, 360.0, k, birim=True)
    assert isin_merkez[0] == pytest.approx(0.0, abs=1e-6)
    assert isin_merkez[1] == pytest.approx(0.0, abs=1e-6)
    assert isin_merkez[2] == pytest.approx(1.0, abs=1e-6)

    # Sağa doğru piksel (640 + fx, 360): xn = 1.0, yn = 0.0, zn = 1.0
    # Birim vektör: (1 / sqrt(2), 0, 1 / sqrt(2)) ~= (0.7071, 0, 0.7071)
    isin_sag = piksel_isin(640.0 + 908.97, 360.0, k, birim=True)
    kok2_yari = 1.0 / math.sqrt(2.0)
    assert isin_sag[0] == pytest.approx(kok2_yari, abs=1e-4)
    assert isin_sag[1] == pytest.approx(0.0, abs=1e-6)
    assert isin_sag[2] == pytest.approx(kok2_yari, abs=1e-4)
    # Norm 1.0 olmalıdır
    norm = math.hypot(isin_sag[0], isin_sag[1], isin_sag[2])
    assert norm == pytest.approx(1.0, abs=1e-6)

    # birim=False seçeneği
    isin_ham = piksel_isin(640.0 + 908.97, 360.0, k, birim=False)
    assert isin_ham == (pytest.approx(1.0, abs=1e-6), pytest.approx(0.0, abs=1e-6), pytest.approx(1.0, abs=1e-6))


def test_isin_derinlik_3b_nesne_yaricap_ilerletme():
    """Ölçülen yüzey derinliğinden küre merkezine yarıçap kadar ilerletilir."""
    # Eksen boyunca ışın: (0, 0, 1)
    isin = (0.0, 0.0, 1.0)
    derinlik = 0.50  # 50 cm
    yaricap = 0.035  # 3.5 cm elma yarıçapı

    x, y, z = isin_derinlik_3b(isin, derinlik, nesne_yaricap=yaricap, oklid_mi=False)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)
    # Ön yüzey z=0.50 iken merkez z = 0.50 + 0.035 = 0.535 m olmalıdır
    assert z == pytest.approx(0.535, abs=1e-5)

    # 45 derece eğimli ışında ilerletme
    kok2_yari = 1.0 / math.sqrt(2.0)
    isin_45 = (kok2_yari, 0.0, kok2_yari)
    x_45, y_45, z_45 = isin_derinlik_3b(isin_45, derinlik, nesne_yaricap=yaricap, oklid_mi=False)
    # Z yüzeyi 0.50 m, X yüzeyi 0.50 m; birim yön (kok2_yari, 0, kok2_yari)
    assert x_45 == pytest.approx(0.50 + yaricap * kok2_yari, abs=1e-5)
    assert y_45 == pytest.approx(0.0, abs=1e-6)
    assert z_45 == pytest.approx(0.50 + yaricap * kok2_yari, abs=1e-5)


# ==============================================================================
# 4. Renk-Derinlik Piksel Eşleme (Dış Kalibrasyon) ve 7x7 Medyan Testleri
# ==============================================================================

def test_renk_derinlik_piksel_esle_es_konumlu():
    """Dış kalibrasyon sıfır iken (eş konumlu kamera) optik merkezler birebir eşleşir."""
    k_renk = (908.97, 908.97, 640.0, 360.0)
    k_derinlik = (429.73, 429.73, 424.0, 240.0)

    # Renk merkez pikseli (640, 360) -> derinlik merkez pikseline (424, 240) gitmelidir
    ud, vd = renk_derinlik_esle(640.0, 360.0, 0.50, k_renk, k_derinlik, t_renk_derinlik=None)
    assert ud == pytest.approx(424.0, abs=1e-4)
    assert vd == pytest.approx(240.0, abs=1e-4)


def test_renk_derinlik_piksel_esle_bilinen_dis_kalibrasyon():
    """Intel RealSense D435 fabrika dış kalibrasyonu (tx=-0.0148 m) ile piksel dönüşümü doğrulanır."""
    k_renk = (908.97, 908.97, 640.0, 360.0)
    k_derinlik = (429.73, 429.73, 424.0, 240.0)

    # D435 optik çerçeveler arası dış kalibrasyon matrisi
    t_renk_derinlik = np.array([
        [1.0, 0.0, 0.0, -0.0148],
        [0.0, 1.0, 0.0, -0.0002],
        [0.0, 0.0, 1.0,  0.0000],
        [0.0, 0.0, 0.0,  1.0000],
    ])

    derinlik = 0.50  # 0.5 metre
    # Renk optik merkez (640, 360): Pc = (0, 0, 0.50)
    # Pd = Pc + t = (-0.0148, -0.0002, 0.50)
    # ud = fx_d * (Xd / Zd) + cx_d = 429.73 * (-0.0148 / 0.50) + 424.0 = 424.0 - 12.720 = 411.28
    # vd = fy_d * (Yd / Zd) + cy_d = 429.73 * (-0.0002 / 0.50) + 240.0 = 240.0 - 0.172 = 239.83
    ud, vd = renk_derinlik_esle(640.0, 360.0, derinlik, k_renk, k_derinlik, t_renk_derinlik=t_renk_derinlik)
    assert ud == pytest.approx(411.28, abs=0.1)
    assert vd == pytest.approx(239.83, abs=0.1)


def test_derinlik_medyan_al_7x7_filtreleme():
    """7x7 pencereden geçerli derinliklerin medyanı doğru hesaplanır ve gürültüler elenir."""
    # 480x848 derinlik görüntüsü (16UC1 milimetre cinsinden)
    derinlik_img = np.zeros((480, 848), dtype=np.uint16)

    # Hedef (411, 240) etrafındaki 15x15 alana 500 mm (0.50 m) doldur
    u_hedef, v_hedef = 411, 240
    derinlik_img[v_hedef - 7:v_hedef + 8, u_hedef - 7:u_hedef + 8] = 500

    # 7x7 pencere içine birkaç bozuk piksel (gürültü ve veri kaybı) ekleyelim
    derinlik_img[v_hedef, u_hedef] = 0        # Sensör ışın kaybı (0 mm)
    derinlik_img[v_hedef + 1, u_hedef] = 0    # Işın kaybı
    derinlik_img[v_hedef - 1, u_hedef] = 1200 # Arka plan sıçraması (1200 mm)

    medyan_m = derinlik_medyan_al(derinlik_img, u_hedef, v_hedef, pencere_boyut=7)
    assert medyan_m is not None
    # Medyan gürültüyü reddetmeli ve tam 0.50 m vermelidir
    assert medyan_m == pytest.approx(0.50, abs=1e-4)

    # Tamamen sıfır/geçersiz bölgede None dönmelidir
    medyan_bos = derinlik_medyan_al(derinlik_img, 50, 50, pencere_boyut=7)
    assert medyan_bos is None


def test_derinlik_medyan_al_float32():
    """float32 formatındaki (metre cinsinden) derinlik görüntüsü medyanı doğrulanır."""
    derinlik_float = np.full((100, 100), np.nan, dtype=np.float32)
    # 7x7 pencereye 0.85 m doldur
    derinlik_float[45:55, 45:55] = 0.85
    derinlik_float[50, 50] = np.nan  # NaN gürültüsü

    medyan = derinlik_medyan_al(derinlik_float, 50.0, 50.0, pencere_boyut=7)
    assert medyan is not None
    assert medyan == pytest.approx(0.85, abs=1e-4)


# ==============================================================================
# 5. Görüntü Mesaj Dönüşüm Testleri
# ==============================================================================

def test_goruntu_mesaj_donusumleri():
    """OpenCV dizisi ile sensor_msgs/Image mesajı arasındaki çift yönlü dönüşüm doğrulanır."""
    sentetik_bgr = np.zeros((100, 200, 3), dtype=np.uint8)
    sentetik_bgr[20:50, 30:80] = [10, 20, 200]

    msg = numpy_dan_goruntu_mesaji(sentetik_bgr, encoding='bgr8', frame_id='kamera')
    assert msg.height == 100
    assert msg.width == 200
    assert msg.encoding == 'bgr8'

    geri_arr = goruntu_mesajindan_numpy(msg)
    np.testing.assert_array_equal(sentetik_bgr, geri_arr)


# ==============================================================================
# 6. Kenar Payı ve Boyut Denetimi Testleri (N17)
# ==============================================================================

def test_kenara_yakin_mi_dort_kenar():
    """Lekenin sınırlayıcı kutusunun 4 kenara yakınlık durumları test edilir."""
    goruntu_sekli = (720, 1280)
    pay = 4

    # 1. Ortada nesne: kenara yakın değil
    kutu_ortada = (600, 300, 50, 50)
    assert not kenara_yakin_mi(kutu_ortada, goruntu_sekli, kenar_payi_px=pay)

    # 2. Sol kenara yakın (x <= 4)
    kutu_sol = (3, 300, 50, 50)
    assert kenara_yakin_mi(kutu_sol, goruntu_sekli, kenar_payi_px=pay)
    kutu_sol_sinir = (4, 300, 50, 50)
    assert kenara_yakin_mi(kutu_sol_sinir, goruntu_sekli, kenar_payi_px=pay)
    kutu_sol_guvenli = (5, 300, 50, 50)
    assert not kenara_yakin_mi(kutu_sol_guvenli, goruntu_sekli, kenar_payi_px=pay)

    # 3. Üst kenara yakın (y <= 4)
    kutu_ust = (600, 2, 50, 50)
    assert kenara_yakin_mi(kutu_ust, goruntu_sekli, kenar_payi_px=pay)
    kutu_ust_sinir = (600, 4, 50, 50)
    assert kenara_yakin_mi(kutu_ust_sinir, goruntu_sekli, kenar_payi_px=pay)

    # 4. Sağ kenara yakın (x + w >= 1280 - 4 = 1276)
    kutu_sag = (1230, 300, 50, 50)  # x + w = 1280 >= 1276
    assert kenara_yakin_mi(kutu_sag, goruntu_sekli, kenar_payi_px=pay)
    kutu_sag_sinir = (1226, 300, 50, 50)  # 1276
    assert kenara_yakin_mi(kutu_sag_sinir, goruntu_sekli, kenar_payi_px=pay)
    kutu_sag_guvenli = (1225, 300, 50, 50)  # 1275 < 1276
    assert not kenara_yakin_mi(kutu_sag_guvenli, goruntu_sekli, kenar_payi_px=pay)

    # 5. Alt kenara yakın (N17 vakası: elma alt kenara giriyor: y + h >= 720 - 4 = 716)
    kutu_alt = (600, 670, 50, 50)  # y + h = 720 >= 716
    assert kenara_yakin_mi(kutu_alt, goruntu_sekli, kenar_payi_px=pay)
    kutu_alt_sinir = (600, 666, 50, 50)  # 716
    assert kenara_yakin_mi(kutu_alt_sinir, goruntu_sekli, kenar_payi_px=pay)
    kutu_alt_guvenli = (600, 665, 50, 50)  # 715 < 716
    assert not kenara_yakin_mi(kutu_alt_guvenli, goruntu_sekli, kenar_payi_px=pay)

    # numpy dizisi veya (W, H) tuple formatları ile çağrım desteği
    img_sentetik = np.zeros((720, 1280, 3), dtype=np.uint8)
    assert kenara_yakin_mi(kutu_alt, img_sentetik, kenar_payi_px=pay)
    assert kenara_yakin_mi(kutu_alt, (1280, 720), kenar_payi_px=pay)


def test_boyut_uygun_mu_tutarlilik_ve_aykiri_reddi():
    """Leke boyutu ve derinlik tutarlılığı denetimi (beklenen 0.035 m ±%60)."""
    fx = 908.97  # D435 renk kamerası odak uzaklığı
    beklenen_r = 0.035  # elma yarıçapı: 3.5 cm

    # İdeal durum: d = 0.50 m, r = 0.035 m -> r_px = 0.035 * 908.97 / 0.50 = 63.63 px
    r_px_ideal = beklenen_r * fx / 0.50
    assert boyut_uygun_mu(r_px_ideal, derinlik_m=0.50, fx=fx, beklenen_yaricap_m=beklenen_r)
    kestirilen = fiziksel_yaricap_kestir(r_px_ideal, 0.50, fx)
    assert kestirilen == pytest.approx(0.035, abs=1e-4)

    # %50 sapma (tolerans ±%60 içinde, kabul edilmeli)
    assert boyut_uygun_mu(r_px_ideal * 1.5, derinlik_m=0.50, fx=fx, beklenen_yaricap_m=beklenen_r)
    assert boyut_uygun_mu(r_px_ideal * 0.5, derinlik_m=0.50, fx=fx, beklenen_yaricap_m=beklenen_r)

    # N17 vakası: elma kesildiği için derinlik penceresi arka plana/zemine düşüyor
    # Derinlik 0.20 m yerine 0.70 m ölçülüyor (3.5 kat büyük)
    # r_kestirilen = r_px * 0.70 / fx = 0.035 * (0.70 / 0.20) = 0.1225 m >> 0.056 m -> REDDEDİLMELİ
    r_px_yakin = beklenen_r * fx / 0.20
    d_hatali = 0.70
    assert not boyut_uygun_mu(r_px_yakin, derinlik_m=d_hatali, fx=fx, beklenen_yaricap_m=beklenen_r)

    # Aşırı küçük kestirim reddedilmeli
    assert not boyut_uygun_mu(10.0, derinlik_m=0.20, fx=fx, beklenen_yaricap_m=beklenen_r)


def test_sinirlayici_kutu_ve_gorunen_yaricap_sentetik():
    """Sentetik kırmızı daire ile sınırlayıcı kutu ve görünen yarıçap hesaplaması doğrulanır."""
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    cx, cy, r = 640, 360, 30
    cv2.circle(img, (cx, cy), r, (0, 0, 255), thickness=-1)

    sonuc = maske_bul(img, min_alan_px=150, kutu_ver=True)
    mask, alan, kutu = sonuc  # type: ignore[misc]
    assert kutu is not None
    x, y, w, h = kutu
    assert x == pytest.approx(cx - r, abs=2)
    assert y == pytest.approx(cy - r, abs=2)
    assert w == pytest.approx(2 * r, abs=2)
    assert h == pytest.approx(2 * r, abs=2)

    r_px = gorunen_yaricap_px_hesapla(alan, kutu)
    assert r_px == pytest.approx(r, abs=1.5)
    assert not kenara_yakin_mi(kutu, img.shape[:2], kenar_payi_px=4)

    # Alt kenara yapışık daire
    img_alt = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.circle(img_alt, (640, 710), 20, (0, 0, 255), thickness=-1)
    sonuc_alt = maske_bul(img_alt, min_alan_px=150, kutu_ver=True)
    _, _, kutu_alt = sonuc_alt  # type: ignore[misc]
    assert kutu_alt is not None
    assert kenara_yakin_mi(kutu_alt, img_alt.shape[:2], kenar_payi_px=4)


def test_en_buyuk_bilesen_filtrele_kutu_ver_secenegi():
    """kutu_ver parametresinin geriye dönük uyumluluğu ve 3'lü demet üretimi test edilir."""
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[20:40, 30:60] = 255  # 20x30 alan = 600 px

    # kutu_ver=False: 2 elemanlı demet (temiz, alan)
    temiz, alan = en_buyuk_bilesen_filtrele(mask, min_alan_px=150, kutu_ver=False)
    assert alan == 600
    assert isinstance(temiz, np.ndarray)

    # kutu_ver=True: 3 elemanlı demet (temiz, alan, kutu)
    sonuc = en_buyuk_bilesen_filtrele(mask, min_alan_px=150, kutu_ver=True)
    assert len(sonuc) == 3
    temiz3, alan3, kutu3 = sonuc  # type: ignore[misc]
    assert alan3 == 600
    assert kutu3 == (30, 20, 30, 20)
    assert sinirlayici_kutu_bul(mask, min_alan_px=150) == (30, 20, 30, 20)


# ==============================================================================
# 7. Sağlam Derinlik Seçimi, Tutarlılık ve Azami Menzil Testleri (N20)
# ==============================================================================

def test_iki_modlu_derinlik_yuzdelik_secimi():
    """İki modlu derinlik dizisinde (ön yüzey 2.00 m %50, arka plan 2.18 m %50)
    düşük yüzdelik seçiminin (~30. yüzdelik) ön yüzeye yakın (~2.00 m) döndüğü doğrulanır.
    """
    # 50 piksel ön yüzey (2.00 m), 50 piksel arka plan/zemin (2.18 m)
    derinlikler = np.array([2.00] * 50 + [2.18] * 50, dtype=np.float64)

    # Standart medyan (50. yüzdelik) iki modun ortasına veya arka plana kayar
    medyan = float(np.median(derinlikler))
    assert medyan == pytest.approx(2.09, abs=0.01)

    # 30. yüzdelik doğrudan ön yüzey modunu seçmelidir (~2.00 m)
    d_secilen = derinlik_yuzdelik_al(derinlikler, yuzdelik=30.0)
    assert d_secilen is not None
    assert d_secilen == pytest.approx(2.00, abs=0.01)

    # Tutarlılık denetimi: 2.00 m etrafındaki ±0.05 m bandında piksellerin %50'si vardır (>= %40)
    oran = derinlik_tutarlilik_orani(derinlikler, d_secilen, derinlik_bant_m=0.05)
    assert oran == pytest.approx(0.50, abs=1e-4)
    assert derinlik_tutarlilik_uygun_mu(
        derinlikler, d_secilen, derinlik_bant_m=0.05, min_oran=0.40
    )

    # Sağlam derinlik hesaplama boru hattı
    d_sonuc, durum = derinlik_saglam_hesapla(
        derinlikler,
        derinlik_yuzdelik=30.0,
        min_derinlik_piksel=6,
        derinlik_bant_m=0.05,
        min_tutarlilik_orani=0.40,
        maks_menzil_m=3.0,
        durum_ver=True,
    )
    assert durum == 'GECERLI'
    assert d_sonuc is not None
    assert d_sonuc == pytest.approx(2.00, abs=0.01)


def test_tutarlilik_reddi_arka_plan_agirlikli():
    """Arka plan ağırlıklı (ön yüzey azınlıkta, arka plan %70-%80) veya dağınık derinlik
    dağılımında tutarlılık oranının <%40 olması nedeniyle ölçümün reddedildiği doğrulanır.
    """
    # Senaryo A: Dağınık zemin/arka plan (%20 ön yüzey 2.00 m, %80 zemin/arka plan 2.18-2.80 m)
    derinlikler_daginik = np.array(
        [2.00] * 20 + [2.18] * 20 + [2.35] * 20 + [2.50] * 20 + [2.80] * 20,
        dtype=np.float64,
    )
    d_daginik, durum_daginik = derinlik_saglam_hesapla(
        derinlikler_daginik,
        derinlik_yuzdelik=30.0,
        derinlik_bant_m=0.05,
        min_tutarlilik_orani=0.40,
        durum_ver=True,
    )
    assert d_daginik is None
    assert durum_daginik == 'TUTARSIZ_ARKA_PLAN'

    # Senaryo B: İki modlu ancak %65 arka plan (2.18 m), %35 ön yüzey (2.00 m)
    # 30. yüzdelik 2.00 m seçer, ancak ±0.05 m bandında yalnızca %35 piksel vardır (< %40)
    derinlikler_70_30 = np.array([2.00] * 35 + [2.18] * 65, dtype=np.float64)
    d_secilen = derinlik_yuzdelik_al(derinlikler_70_30, yuzdelik=30.0)
    assert d_secilen == pytest.approx(2.00, abs=0.01)

    oran = derinlik_tutarlilik_orani(derinlikler_70_30, d_secilen, derinlik_bant_m=0.05)
    assert oran == pytest.approx(0.35, abs=1e-4)
    assert not derinlik_tutarlilik_uygun_mu(
        derinlikler_70_30, d_secilen, derinlik_bant_m=0.05, min_oran=0.40
    )

    d_ret, durum_ret = derinlik_saglam_hesapla(
        derinlikler_70_30,
        derinlik_yuzdelik=30.0,
        derinlik_bant_m=0.05,
        min_tutarlilik_orani=0.40,
        durum_ver=True,
    )
    assert d_ret is None
    assert durum_ret == 'TUTARSIZ_ARKA_PLAN'


def test_az_piksel_yedek_yola_donus():
    """Leke içi geçerli piksel sayısı min_derinlik_piksel (vars. 6) altındaysa
    eski pencere-medyan yedek yoluna dönüldüğü doğrulanır.
    """
    # Yalnızca 4 piksel içeren derinlik dizisi (< min_derinlik_piksel=6)
    derinlikler_az = np.array([2.01, 2.02, 1.99, 2.00], dtype=np.float64)
    yedek_medyan = 2.05

    # Yedek derinlik mevcutsa 'YEDEK_YOL' ile yedek değer seçilmelidir
    d_yedek, durum_yedek = derinlik_saglam_hesapla(
        derinlikler_az,
        yedek_derinlik=yedek_medyan,
        min_derinlik_piksel=6,
        durum_ver=True,
    )
    assert d_yedek == pytest.approx(2.05, abs=1e-4)
    assert durum_yedek == 'YEDEK_YOL'

    # Yedek yoksa 'AZ_PIKSEL' ve None dönmelidir
    d_bos, durum_bos = derinlik_saglam_hesapla(
        derinlikler_az,
        yedek_derinlik=None,
        min_derinlik_piksel=6,
        durum_ver=True,
    )
    assert d_bos is None
    assert durum_bos == 'AZ_PIKSEL'


def test_maks_menzil_reddi():
    """maks_menzil_m (varsayılan 3.0 m) üzerindeki uzak ölçümlerin reddedildiği doğrulanır."""
    assert menzil_uygun_mu(2.95, maks_menzil_m=3.0)
    assert not menzil_uygun_mu(3.01, maks_menzil_m=3.0)
    assert not menzil_uygun_mu(3.50, maks_menzil_m=3.0)

    # 3.20 m'deki ölçüm maks_menzil_m=3.0 ile reddedilir
    derinlikler_uzak = np.array([3.20] * 50, dtype=np.float64)
    d_uzak, durum_uzak = derinlik_saglam_hesapla(
        derinlikler_uzak,
        maks_menzil_m=3.0,
        durum_ver=True,
    )
    assert d_uzak is None
    assert durum_uzak == 'MAKS_MENZIL_ASILDI'

    # Yedek yolun da maks_menzil sınırını aşması halinde reddedildiği doğrulanır
    derinlikler_az = np.array([3.50] * 3, dtype=np.float64)
    d_yedek_uzak, durum_yedek_uzak = derinlik_saglam_hesapla(
        derinlikler_az,
        yedek_derinlik=3.50,
        min_derinlik_piksel=6,
        maks_menzil_m=3.0,
        durum_ver=True,
    )
    assert d_yedek_uzak is None
    assert durum_yedek_uzak == 'MAKS_MENZIL_ASILDI'


def test_leke_kutusu_ic_sinirlari_ve_ayiklama():
    """Leke kutusunun iç çekirdek sınırlarının (%25 kenar payı) doğru hesaplandığı ve
    derinlik görüntüsünden geçerli piksellerin ayıklandığı doğrulanır.
    """
    # 40x40 piksellik leke kutusu, merkez (400, 200)
    # Kenardan %25 içeri -> genişlik 20 px, yükseklik 20 px
    u_min, u_max, v_min, v_max = leke_kutusu_ic_sinirlari(
        u_merkez=400.0,
        v_merkez=200.0,
        genislik=40.0,
        yukseklik=40.0,
        kenar_payi_orani=0.25,
        goruntu_genislik=848,
        goruntu_yukseklik=480,
    )
    assert u_min == 390
    assert u_max == 410
    assert v_min == 190
    assert v_max == 210
    assert (u_max - u_min) == 20
    assert (v_max - v_min) == 20

    # uint16 (mm) derinlik görüntüsü üzerinde ayıklama
    img_derinlik = np.zeros((480, 848), dtype=np.uint16)
    img_derinlik[190:210, 390:410] = 2000  # 2.00 metre
    img_derinlik[195, 395] = 0             # Gürültü (ışın kaybı)

    kirpma = img_derinlik[v_min:v_max, u_min:u_max]
    pikseller = derinlik_gecerli_pikselleri_ayikla(kirpma)
    # 20x20 = 400 piksel, 1 bozuk -> 399 geçerli piksel
    assert len(pikseller) == 399
    np.testing.assert_allclose(pikseller, 2.00, atol=1e-4)


def test_dugum_n20_varsayilan_parametreler():
    """RenkAlgiDugumu N20 parametrelerinin varsayılan değerleri doğrulanır."""
    import rclpy
    from limo_autonomy.renk_algi import RenkAlgiDugumu

    if not rclpy.ok():
        rclpy.init()
    try:
        dugum = RenkAlgiDugumu()
        assert dugum.get_parameter('derinlik_yuzdelik').value == 30
        assert dugum.get_parameter('min_derinlik_piksel').value == 6
        assert dugum.get_parameter('derinlik_bant_m').value == pytest.approx(0.05, abs=1e-4)
        assert dugum.get_parameter('maks_menzil_m').value == pytest.approx(3.0, abs=1e-4)
        assert dugum.sayac_yedek_derinlik == 0
        assert dugum.sayac_tutarsiz_derinlik == 0
        assert dugum.sayac_asiri_menzil == 0
        dugum.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
