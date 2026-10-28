"""Hareketli engel kalkanı testleri (test_hareketli_kalkan.py).

ROS'suz saf Python ve pytest ile çalışır:
1. Kümeleme (iki ayrı nesne, gürültülü tek nesne)
2. İzleyici hız kestirimi (0.6 m/s doğrusal hareket 10 adımda ±0.1)
3. TTC analitik durumlar (kafa kafaya, paralel geçen -> inf, zaten temas -> 0)
4. Durağan nesne robot ona 0.3 m/s giderken kararı 'serbest' (kavrama koruması)
5. Karşıdan gelen 0.6 m/s -> 'dur'
6. 'dur' iken geri komutu ve dönüş kısılmıyor
7. Robot da odom'da hareket ederken durağan nesnenin hızı ≈ 0 kestiriliyor
8. Tarama noktaları filtreleme ve lidar_x ofseti
9. 0.5 s boyunca görülmeyen izin silinmesi
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from limo_autonomy.hareketli_kalkan import (
    Iz,
    IzTakipci,
    carpismaya_kalan_sure,
    kalkan_karari,
    komutu_olcekle,
    kume_merkezleri_odoma,
    kumele,
    tarama_noktalari,
)


def test_tarama_noktalari_kartezyen_ve_ofset() -> None:
    """LaserScan ışınlarını kartezyen noktalara dönüştürme ve ofset testi."""
    # 3 ışın: 0 rad (ön), pi/2 rad (sol), -pi/2 rad (sağ)
    ranges = [2.0, 1.5, 3.0]
    angle_min = 0.0
    angle_increment = math.pi / 2.0
    range_min = 0.1
    range_max = 5.0

    # Ofsetsiz
    pts = tarama_noktalari(
        ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
        max_menzil=4.0,
        lidar_x=0.0,
    )
    assert pts.shape == (3, 2)
    # Ön nokta: (2.0, 0.0)
    assert abs(pts[0, 0] - 2.0) < 1e-4
    assert abs(pts[0, 1] - 0.0) < 1e-4
    # Sol nokta: (0.0, 1.5)
    assert abs(pts[1, 0] - 0.0) < 1e-4
    assert abs(pts[1, 1] - 1.5) < 1e-4

    # lidar_x ofsetli (+0.10 m)
    pts_ofset = tarama_noktalari(
        ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
        max_menzil=4.0,
        lidar_x=0.10,
    )
    assert abs(pts_ofset[0, 0] - 2.10) < 1e-4

    # max_menzil filtreleme: 2.5 m üzerindeki noktalar elenmeli
    pts_filtre = tarama_noktalari(
        ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
        max_menzil=2.5,
    )
    # 3.0 m olan 3. nokta elenir, 2 nokta kalır
    assert pts_filtre.shape == (2, 2)


def test_kumeleme_iki_ayri_nesne() -> None:
    """Aralarında boşluk olan iki ayrı nesnenin iki ayrı küme olarak bulunması."""
    # 1. Nesne: (1.0, 0.0) civarında 5 nokta (aralık 0.02 m <= esik 0.10 m)
    nesne1 = [[1.0, -0.04], [1.0, -0.02], [1.0, 0.0], [1.0, 0.02], [1.0, 0.04]]
    # 2. Nesne: (2.5, 0.0) civarında 5 nokta
    # 1. nesne ile 2. nesne arasındaki mesafe ~1.5 m (> 0.10 m)
    nesne2 = [[2.5, -0.04], [2.5, -0.02], [2.5, 0.0], [2.5, 0.02], [2.5, 0.04]]

    noktalar = np.array(nesne1 + nesne2)
    merkezler, yaricaplar = kumele(noktalar, esik=0.10, min_nokta=3)

    assert len(merkezler) == 2, f"2 küme beklenirken {len(merkezler)} bulundu"
    assert len(yaricaplar) == 2

    # 1. küme merkezi ve yarıçapı
    assert abs(merkezler[0][0] - 1.0) < 0.02
    assert abs(merkezler[0][1] - 0.0) < 0.02
    assert abs(yaricaplar[0] - 0.04) < 0.01

    # 2. küme merkezi ve yarıçapı
    assert abs(merkezler[1][0] - 2.5) < 0.02
    assert abs(merkezler[1][1] - 0.0) < 0.02
    assert abs(yaricaplar[1] - 0.04) < 0.01


def test_kumeleme_gurultulu_tek_nesne() -> None:
    """Gürültülü tek bir silindirik nesnenin tek küme olarak doğru tespit edilmesi."""
    # (1.5, 0.5) merkezli, r=0.20 m olan bir silindirin ön yüzeyinden 15 nokta
    cx_true, cy_true, r_true = 1.5, 0.5, 0.20
    np.random.seed(42)

    acilar = np.linspace(-math.pi / 3.0, math.pi / 3.0, 15)
    # Ardışık nokta mesafesi: r * d_theta ≈ 0.20 * (2pi/3 / 14) ≈ 0.03 m < 0.10 m
    pts = []
    for th in acilar:
        gurultu = np.random.uniform(-0.005, 0.005)
        x = cx_true - (r_true + gurultu) * math.cos(th)
        y = cy_true + (r_true + gurultu) * math.sin(th)
        pts.append([x, y])

    merkezler, yaricaplar = kumele(pts, esik=0.10, min_nokta=3)

    assert len(merkezler) == 1, f"Tek küme beklenirken {len(merkezler)} bulundu"
    # Küme merkezi yay üzerinde olduğu için merkeze yakın olmalıdır
    assert abs(merkezler[0][0] - (cx_true - r_true)) < 0.10
    assert abs(merkezler[0][1] - cy_true) < 0.05
    assert yaricaplar[0] > 0.05


def test_izleyici_hiz_kestirimi_dogrusal_hareket() -> None:
    """0.6 m/s doğrusal hareket eden nesnenin 10 adımda hızının ±0.1 toleransla kestirilmesi."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5)

    dt = 0.1  # 10 Hz
    v_gercek = 0.6  # m/s
    x_baslangic = 1.0

    # 10 adım doğrusal hareket (k = 0..9)
    for k in range(10):
        t = k * dt
        x_k = x_baslangic + v_gercek * t
        merkezler = [(x_k, 0.0)]
        yaricaplar = [0.20]
        takipci.guncelle(merkezler, yaricaplar, t)

    izler = takipci.izler()
    assert len(izler) == 1, f"Tek iz beklenirken {len(izler)} iz var"

    iz = izler[0]
    hiz_tahmin = math.hypot(iz.vx, iz.vy)

    # 10 adımda ±0.1 m/s tolerans (0.50 ile 0.70 m/s arasında olmalı)
    assert abs(hiz_tahmin - v_gercek) <= 0.10, (
        f"Kestirilen hız {hiz_tahmin:.3f} m/s, beklenen {v_gercek} ± 0.1 m/s"
    )
    assert iz.gorulme == 10


def test_ttc_analitik_durumlar() -> None:
    """TTC analitik durumları: kafa kafaya, paralel geçen -> inf, zaten temas -> 0, uzaklaşan -> inf."""
    r_toplam = 0.40  # robot (0.20) + engel (0.20)

    # 1. Kafa kafaya: robot (0, 0), engel (2.0, 0.0), bağıl hız (-1.0, 0.0)
    # Temas anı: 2.0 - 1.0 * t = 0.40  => t = 1.60 s
    p_rel = (2.0, 0.0)
    v_rel = (-1.0, 0.0)
    ttc_kafa = carpismaya_kalan_sure(p_rel, v_rel, r_toplam)
    assert abs(ttc_kafa - 1.60) < 1e-4, f"Kafa kafaya TTC: {ttc_kafa:.4f}, beklenen: 1.60"

    # 2. Paralel geçen: engel yanal 1.0 m mesafeden paralel geçiyor
    # Min yaklaşma mesafesi 1.0 m > r_toplam (0.40 m) -> çarpışma yok, inf
    p_paralel = (2.0, 1.0)
    v_paralel = (-1.0, 0.0)
    ttc_paralel = carpismaya_kalan_sure(p_paralel, v_paralel, r_toplam)
    assert ttc_paralel == math.inf, f"Paralel geçen TTC inf olmalı: {ttc_paralel}"

    # 3. Zaten temas / iç içe: mesafe 0.30 m <= 0.40 m -> TTC 0.0
    p_temas = (0.30, 0.0)
    v_temas = (-1.0, 0.0)
    ttc_temas = carpismaya_kalan_sure(p_temas, v_temas, r_toplam)
    assert ttc_temas == 0.0, f"Zaten temas TTC 0.0 olmalı: {ttc_temas}"
    # 3b. İç içe ama uzaklaşıyor (tepkisiz engel yanından geçip gidiyor) -> tehdit yok
    assert math.isinf(carpismaya_kalan_sure(p_temas, (1.0, 0.0), r_toplam))

    # 4. Uzaklaşan engel: engel robottan uzaklaşıyor -> inf
    p_uzaklasan = (2.0, 0.0)
    v_uzaklasan = (1.0, 0.0)
    ttc_uzak = carpismaya_kalan_sure(p_uzaklasan, v_uzaklasan, r_toplam)
    assert ttc_uzak == math.inf, f"Uzaklaşan engel TTC inf olmalı: {ttc_uzak}"

    # 5. Sıfır hız: duran iki cisim
    ttc_sifir = carpismaya_kalan_sure((2.0, 0.0), (0.0, 0.0), r_toplam)
    assert ttc_sifir == math.inf


def test_duragan_nesne_kavrama_korumasi() -> None:
    """DURAĞAN nesne robot ona 0.3 m/s giderken kararı 'serbest' (kavrama hedefi/mobilya koruması)."""
    # Nesne odom'da (0.5, 0.0)'da hareketsiz duruyor (v_iz = 0.0 m/s)
    # Robot ona doğru 0.3 m/s ile gidiyor
    iz_duragan = Iz(
        id=1,
        x=0.50,
        y=0.0,
        r=0.10,
        vx=0.0,
        vy=0.0,
        gorulme=10,
        yas=10,
    )

    robot_v_odom = (0.30, 0.0)
    robot_p_odom = (0.0, 0.0)

    karar, olcek, gerekce = kalkan_karari(
        izler=[iz_duragan],
        robot_v_odom=robot_v_odom,
        robot_p_odom=robot_p_odom,
        min_engel_hizi=0.20,
        min_gorulme=3,
        ttc_dur=1.5,
        ttc_yavas=3.0,
    )

    # En kritik kural: Durağan nesneler kalkana ASLA takılmamalıdır!
    assert karar == 'serbest', f"Durağan nesne için karar 'serbest' olmalı, alınan: {karar}"
    assert olcek == 1.0
    assert gerekce == 'serbest'


def test_karsidan_gelen_hareketli_engel_dur_ve_yavas() -> None:
    """Karşıdan 0.6 m/s ile gelen hareketli engel durumunda 'dur' ve 'yavas' kararları."""
    # Robot (0, 0)'da ve 0.2 m/s ileri gidiyor
    robot_v_odom = (0.20, 0.0)
    robot_p_odom = (0.0, 0.0)

    # 1. 'dur' durumu: Engel (1.4, 0.0)'da, hızı -0.6 m/s
    # Bağıl hız: -0.6 - 0.2 = -0.8 m/s
    # r_toplam = 0.20 + 0.20 = 0.40 m
    # TTC = (1.4 - 0.4) / 0.8 = 1.25 s (< 1.5 s ttc_dur) -> 'dur'
    iz_yakin = Iz(
        id=1,
        x=1.40,
        y=0.0,
        r=0.20,
        vx=-0.60,
        vy=0.0,
        gorulme=5,
        yas=5,
    )

    karar_dur, olcek_dur, gerekce_dur = kalkan_karari(
        izler=[iz_yakin],
        robot_v_odom=robot_v_odom,
        robot_p_odom=robot_p_odom,
        ttc_dur=1.5,
        ttc_yavas=3.0,
    )

    assert karar_dur == 'dur'
    assert olcek_dur == 0.0
    assert 'DUR' in gerekce_dur

    # 2. 'yavas' durumu: Engel (2.4, 0.0)'da, hızı -0.6 m/s
    # TTC = (2.4 - 0.4) / 0.8 = 2.50 s (1.5 s < TTC < 3.0 s) -> 'yavas'
    # Doğrusal ölçek: (2.50 - 1.5) / (3.0 - 1.5) = 1.0 / 1.5 ≈ 0.667
    iz_orta = Iz(
        id=2,
        x=2.40,
        y=0.0,
        r=0.20,
        vx=-0.60,
        vy=0.0,
        gorulme=5,
        yas=5,
    )

    karar_yavas, olcek_yavas, gerekce_yavas = kalkan_karari(
        izler=[iz_orta],
        robot_v_odom=robot_v_odom,
        robot_p_odom=robot_p_odom,
        ttc_dur=1.5,
        ttc_yavas=3.0,
    )

    assert karar_yavas == 'yavas'
    assert abs(olcek_yavas - (2.5 - 1.5) / 1.5) < 1e-3
    assert 'YAVAS' in gerekce_yavas


def test_dur_iken_geri_ve_donus_kisilmiyor() -> None:
    """'dur' kararı aktifken öndeki engele karşı geri komutu ve açısal hızın (wz) kısılmaması."""
    # Engel önde: engel_yon_x = 1.0
    # 1. Geri komutu: vx = -0.20 m/s, wz = 0.5 rad/s
    vx_cikis, wz_cikis = komutu_olcekle(
        vx=-0.20,
        wz=0.50,
        karar='dur',
        olcek=0.0,
        engel_yon_x=1.0,
    )
    # Geri hız ve dönüş kısılmamalıdır!
    assert vx_cikis == -0.20, f"Geri hız kısılmamalı: {vx_cikis}"
    assert wz_cikis == 0.50, f"Açısal hız korunmalı: {wz_cikis}"

    # 2. Yerinde dönüş komutu: vx = 0.0, wz = -0.6 rad/s
    vx_don, wz_don = komutu_olcekle(
        vx=0.0,
        wz=-0.60,
        karar='dur',
        olcek=0.0,
        engel_yon_x=1.0,
    )
    assert vx_don == 0.0
    assert wz_don == -0.60

    # 3. Engele doğru ileri komut: vx = 0.20 m/s -> kısılmalı (0.0 olmalı)
    vx_ileri, wz_ileri = komutu_olcekle(
        vx=0.20,
        wz=0.30,
        karar='dur',
        olcek=0.0,
        engel_yon_x=1.0,
    )
    assert vx_ileri == 0.0, f"Engele doğru ileri hız sıfırlanmalı: {vx_ileri}"
    assert wz_ileri == 0.30


def test_robot_hareketli_duragan_nesne_hizi_sifir() -> None:
    """Robot odom'da hareket ederken durağan nesnenin hızının ≈ 0 kestirilmesi."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5)

    # Odom'da durağan duran nesne konumu: (3.0, 1.0)
    nesne_odom_gercek = (3.0, 1.0)
    v_robot = 0.40  # m/s ileri hareket
    dt = 0.1

    for k in range(10):
        t = k * dt
        # Robot odom'da ilerliyor: x_r = v_robot * t, y_r = 0, yaw = 0
        x_r = v_robot * t
        robot_pose = (x_r, 0.0, 0.0)

        # Nesnenin robot tabanındaki (base_link) göreli konumu:
        # x_base = nesne_x - x_r, y_base = nesne_y - y_r
        x_b = nesne_odom_gercek[0] - x_r
        y_b = nesne_odom_gercek[1] - 0.0

        # Koordinat dönüştürme fonksiyonu ile odom'a çevir
        merkezler_odom = kume_merkezleri_odoma([(x_b, y_b)], robot_pose)
        takipci.guncelle(merkezler_odom, [0.15], t)

    izler = takipci.izler()
    assert len(izler) == 1

    iz = izler[0]
    hiz_kestirimi = math.hypot(iz.vx, iz.vy)

    # Durağan nesnenin odom hızı pratikte 0 olmalıdır (< 0.05 m/s)
    assert hiz_kestirimi < 0.05, f"Durağan nesne hızı {hiz_kestirimi:.3f} m/s; < 0.05 olmalı"

    # Karar 'serbest' kalmalıdır
    karar, olcek, _ = kalkan_karari(
        izler=izler,
        robot_v_odom=(v_robot, 0.0),
        robot_p_odom=(v_robot * 0.9, 0.0),
    )
    assert karar == 'serbest'
    assert olcek == 1.0


def test_iz_silme_zaman_asimi() -> None:
    """0.5 s boyunca görülmeyen izlerin izleyiciden silinmesi."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5)

    # t = 0.0'da bir iz oluştur
    takipci.guncelle([(1.0, 0.0)], [0.2], 0.0)
    assert len(takipci.izler()) == 1

    # t = 0.3 s'de güncelleme yok (boş tarama) -> silme süresi (0.5 s) dolmadı, iz durmalı
    takipci.guncelle([], [], 0.3)
    assert len(takipci.izler()) == 1

    # t = 0.6 s'de (0.6 - 0.0 = 0.6 > 0.5 s) -> iz zaman aşımından silinmeli
    takipci.guncelle([], [], 0.6)
    assert len(takipci.izler()) == 0


def test_kalkan_karari_coklu_iz_en_kritik_secimi() -> None:
    """Birden fazla nesne varken kalkanın en küçük TTC'ye sahip tehdidi seçmesi."""
    # 1. Durağan nesne: yakında ama durağan
    iz_statik = Iz(id=1, x=0.6, y=0.0, r=0.1, vx=0.0, vy=0.0, gorulme=10, yas=10)
    # 2. Uzakta hareketli: TTC büyük (~10 s)
    iz_uzak = Iz(id=2, x=8.0, y=0.0, r=0.2, vx=-0.5, vy=0.0, gorulme=5, yas=5)
    # 3. Yaklaşan tehdit: (1.5, 0.0)'da, hızı -0.8 m/s, TTC ~ 1.375 s (< 1.5 s ttc_dur)
    iz_tehdit = Iz(id=3, x=1.5, y=0.0, r=0.2, vx=-0.8, vy=0.0, gorulme=4, yas=4)

    karar, olcek, gerekce = kalkan_karari(
        izler=[iz_statik, iz_uzak, iz_tehdit],
        robot_v_odom=(0.0, 0.0),
        robot_p_odom=(0.0, 0.0),
        ttc_dur=1.5,
        ttc_yavas=3.0,
    )

    assert karar == 'dur'
    assert olcek == 0.0
    assert 'DUR' in gerekce
