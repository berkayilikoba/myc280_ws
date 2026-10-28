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
    EslestirmeSonucu,
    HareketliKalkan,
    Iz,
    IzTakipci,
    belirsiz_izler_kumesi,
    carpismaya_kalan_sure,
    eslestir,
    kalkan_karari,
    komutu_olcekle,
    kume_merkezleri_odoma,
    kumele,
    odom_zaman_asimi_kontrolu,
    poz_aradeger,
    tarama_noktalari,
    tarama_zaman_asimi_yonetimi,
    yon_tutarliligi,
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
        yas=10, t_ilk=0.0, t_son=1.0,
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
        yas=5, t_ilk=0.0, t_son=0.5,
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
        yas=5, t_ilk=0.0, t_son=0.5,
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
    iz_statik = Iz(id=1, x=0.6, y=0.0, r=0.1, vx=0.0, vy=0.0, gorulme=10, yas=10, t_ilk=0.0, t_son=1.0)
    # 2. Uzakta hareketli: TTC büyük (~10 s)
    iz_uzak = Iz(id=2, x=8.0, y=0.0, r=0.2, vx=-0.5, vy=0.0, gorulme=5, yas=5, t_ilk=0.0, t_son=0.5)
    # 3. Yaklaşan tehdit: (1.5, 0.0)'da, hızı -0.8 m/s, TTC ~ 1.375 s (< 1.5 s ttc_dur)
    # min_gorulme varsayılanı 3->5 yapıldığı ve min_iz_yasi_s=0.4 s eklendiği için gorulme=5, yas=5, t_ilk=0.0, t_son=0.5
    iz_tehdit = Iz(id=3, x=1.5, y=0.0, r=0.2, vx=-0.8, vy=0.0, gorulme=5, yas=5, t_ilk=0.0, t_son=0.5)

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


def test_donus_gecikmeli_tarama_aradeger_hiz_kucuk() -> None:
    """Robot yerinde 1 rad/s dönerken, tarama 80 ms gecikmeli ve DOĞRU aradeğerli pozla
    verilince durağan noktaların hızının < 0.1 m/s kalması."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5, maks_donus_hizi=1.5)

    wz = 1.0  # rad/s yerinde dönüş
    # Odom geçmişi (50 Hz, dt = 0.02 s)
    gecmis: list[tuple[float, float, float, float]] = []
    for k in range(100):
        t_o = k * 0.02
        yaw_o = wz * t_o
        gecmis.append((t_o, 0.0, 0.0, yaw_o))

    # Odom'da (2.0, 1.0) konumunda durağan bir nesne
    nesne_odom_gercek = (2.0, 1.0)

    # Taramalar 10 Hz (her 100 ms) geliyor ancak t_scan damgası var (80 ms gecikmeyle)
    for step in range(10):
        t_scan = 0.20 + step * 0.10  # 0.20 s ile 1.10 s arasında
        pose_aradeger = poz_aradeger(gecmis, t_scan)
        assert pose_aradeger is not None

        # Gerçekte t_scan anındaki robot açısına göre sensörün gördüğü bağıl nokta
        yaw_true = wz * t_scan
        c = math.cos(yaw_true)
        s = math.sin(yaw_true)
        dx = nesne_odom_gercek[0]
        dy = nesne_odom_gercek[1]
        x_base = c * dx + s * dy
        y_base = -s * dx + c * dy

        # Aradeğerli pozla odom'a çevir
        merkezler_odom = kume_merkezleri_odoma([(x_base, y_base)], pose_aradeger)
        # robot_wz=0 verilse bile (yani hız güncellemesi açık tutulsa bile) aradeğerin doğruluğu
        # sayesinde durağan noktanın hızı < 0.1 m/s kalmalıdır
        takipci.guncelle(merkezler_odom, [0.15], t_scan, robot_wz=0.0)

    izler = takipci.izler()
    assert len(izler) == 1
    hiz_kestirimi = izler[0].hiz
    assert hiz_kestirimi < 0.10, (
        f"Durağan noktanın kestirilen hızı {hiz_kestirimi:.4f} m/s >= 0.10 m/s!"
    )


def test_yakin_duragan_kumeler_gurultulu_serbest_karari() -> None:
    """0.4 m arayla iki durağan küme, gürültülü ölçümle 30 adımda 'serbest' kararı."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5)

    # 0.4 m arayla iki masa ayağı / durağan nesne (1.5, 0.0) ve (1.5, 0.40)
    kume1_true = (1.50, 0.00)
    kume2_true = (1.50, 0.40)
    rng = np.random.RandomState(42)

    for k in range(30):
        t = k * 0.10
        # ±0.02 m gürültü
        g1 = rng.uniform(-0.02, 0.02, size=2)
        g2 = rng.uniform(-0.02, 0.02, size=2)
        c1 = (kume1_true[0] + float(g1[0]), kume1_true[1] + float(g1[1]))
        c2 = (kume2_true[0] + float(g2[0]), kume2_true[1] + float(g2[1]))

        takipci.guncelle([c1, c2], [0.10, 0.10], t, robot_wz=0.0)

        karar = kalkan_karari(
            izler=takipci.izler(),
            robot_v_odom=(0.0, 0.0),
            robot_p_odom=(0.0, 0.0),
            min_engel_hizi=0.20,
            min_gorulme=5,
            min_iz_yasi_s=0.4,
            yon_tutarlilik=0.7,
            ttc_dur=1.5,
            ttc_yavas=3.0,
        )

        assert karar.karar == 'serbest', (
            f"Adım {k} (t={t:.1f}s)'de durağan kümeler için karar 'serbest' olmalı, "
            f"alınan: {karar.karar} ({karar.gerekce})"
        )


def test_karsidan_gelen_insan_1s_icinde_dur_tetikler() -> None:
    """Karşıdan 0.6 m/s gelen insan hâlâ <= 1.0 s içinde 'dur' tetikler (gerçek tehditi kaçırma)."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5)

    # Robot (0, 0)'da, insan 1.4 m öteden robota doğru 0.6 m/s ile yaklaşıyor
    dur_tetiklendi = False
    tetiklenme_t: float | None = None

    for k in range(11):  # 0.0 - 1.0 saniye arası (10 Hz, 11 adım)
        t = k * 0.10
        x_insan = 1.40 - 0.60 * t
        merkezler = [(x_insan, 0.0)]
        takipci.guncelle(merkezler, [0.20], t, robot_wz=0.0)

        karar = kalkan_karari(
            izler=takipci.izler(),
            robot_v_odom=(0.0, 0.0),
            robot_p_odom=(0.0, 0.0),
            robot_yaricap=0.20,
            min_engel_hizi=0.20,
            min_gorulme=5,
            min_iz_yasi_s=0.4,
            yon_tutarlilik=0.7,
            ttc_dur=1.5,
            ttc_yavas=3.0,
        )

        if karar.karar == 'dur':
            dur_tetiklendi = True
            tetiklenme_t = t
            break

    assert dur_tetiklendi is True, "Karşıdan gelen 0.6 m/s insan 1.0 s içinde DUR tetiklemeli!"
    assert tetiklenme_t is not None and tetiklenme_t <= 1.0, (
        f"DUR tetiklenme zamanı {tetiklenme_t}s > 1.0s!"
    )


def test_poz_aradeger_yaw_wrap() -> None:
    """poz_aradeger fonksiyonunda yaw wrap (179° -> -179°) aradeğer testi."""
    yaw_179 = math.radians(179.0)
    yaw_eksi_179 = math.radians(-179.0)

    # t = 0.0'da (1.0, 2.0, 179°), t = 1.0'da (3.0, 4.0, -179°)
    # En kısa açısal fark: 179° -> 180° / -180° -> -179° = +2° (+0.0349 rad)
    gecmis = [
        (0.0, 1.0, 2.0, yaw_179),
        (1.0, 3.0, 4.0, yaw_eksi_179),
    ]

    # 1. Tam ortada t = 0.5: x=2.0, y=3.0, yaw = 180° veya -180° (pi veya -pi)
    poz_orta = poz_aradeger(gecmis, 0.5, maks_dt=1.0)
    assert poz_orta is not None
    assert abs(poz_orta[0] - 2.0) < 1e-4
    assert abs(poz_orta[1] - 3.0) < 1e-4
    assert abs(abs(poz_orta[2]) - math.pi) < 1e-4

    # 2. Çeyrek noktada t = 0.25: yaw = 179.5°
    poz_25 = poz_aradeger(gecmis, 0.25, maks_dt=1.0)
    assert poz_25 is not None
    assert abs(poz_25[2] - math.radians(179.5)) < 1e-4

    # 3. Üç çeyrekte t = 0.75: yaw = -179.5°
    poz_75 = poz_aradeger(gecmis, 0.75, maks_dt=1.0)
    assert poz_75 is not None
    assert abs(poz_75[2] - math.radians(-179.5)) < 1e-4

    # 4. Ters yön: -179° -> 179° (-2° dönüş)
    gecmis_ters = [
        (0.0, 0.0, 0.0, yaw_eksi_179),
        (1.0, 0.0, 0.0, yaw_179),
    ]
    poz_ters_orta = poz_aradeger(gecmis_ters, 0.5, maks_dt=1.0)
    assert poz_ters_orta is not None
    assert abs(abs(poz_ters_orta[2]) - math.pi) < 1e-4

    # 5. maks_dt (0.2 s) sınır kontrolleri
    # 0.1 s dışarıda -> en yakın ucu kullanır
    poz_yakin_son = poz_aradeger(gecmis, 1.10, maks_dt=0.2)
    assert poz_yakin_son is not None
    assert abs(poz_yakin_son[0] - 3.0) < 1e-4

    poz_yakin_ilk = poz_aradeger(gecmis, -0.10, maks_dt=0.2)
    assert poz_yakin_ilk is not None
    assert abs(poz_yakin_ilk[0] - 1.0) < 1e-4

    # 0.25 s dışarıda (> maks_dt 0.2s) -> None döner
    assert poz_aradeger(gecmis, 1.25, maks_dt=0.2) is None
    assert poz_aradeger(gecmis, -0.25, maks_dt=0.2) is None
    assert poz_aradeger([], 0.5) is None


def test_yon_tutarliligi() -> None:
    """Hız yön tutarlılığı fonksiyonunun doğrulanması."""
    # Aynı yönde 5 ölçüm -> tutarlılık 1.0
    hizlar_duz = [(0.5, 0.0)] * 5
    assert abs(yon_tutarliligi(hizlar_duz) - 1.0) < 1e-4

    # Zıt yönde gidip gelen ölçümler -> tutarlılık ~0
    hizlar_titrek = [(0.5, 0.0), (-0.5, 0.0)]
    assert yon_tutarliligi(hizlar_titrek) < 0.1
    # son_n=4 için 4 ölçüm
    assert yon_tutarliligi([(0.5, 0.0), (-0.5, 0.0), (0.5, 0.0), (-0.5, 0.0)], son_n=4) < 0.1

    # Boş ölçüm -> 0.0
    assert yon_tutarliligi([]) == 0.0


def test_tarama_noktalari_lidar_montaj_yaw() -> None:
    """R1 B1.1: LIMO lidarı (0.137, 0.015, yaw -0.1686); yaw yok sayılırsa 2 m'de ~33 cm yanal hata."""
    yaw = -0.1686
    # Lidar çerçevesinde tam önde (açı 0) 2 m'lik tek nokta, base_link'te ileri-sağa düşmeli
    pts = tarama_noktalari([2.0], 0.0, 0.01, 0.05, 10.0, max_menzil=5.0,
                           lidar_x=0.137, lidar_y=0.015, lidar_yaw=yaw)
    assert pts.shape == (1, 2)
    assert pts[0, 0] == pytest.approx(0.137 + 2.0 * math.cos(yaw), abs=1e-6)
    assert pts[0, 1] == pytest.approx(0.015 + 2.0 * math.sin(yaw), abs=1e-6)
    assert pts[0, 1] < -0.3


def test_iz_yasi_ayni_damga_buyumez() -> None:
    """R1 B1.6: aynı damgalı tekrar güncellemeler iz yaşını yapay büyütmemeli."""
    takipci = IzTakipci()
    for _ in range(6):
        takipci.guncelle([(1.0, 0.0)], [0.1], 5.0)
    izler = takipci.izler()
    assert izler and all(iz.iz_yasi_s == 0.0 for iz in izler)


def test_b1_3_lidar_6hz_tanima_ve_tetikleme_gecikmesi() -> None:
    """B1.3: 6 Hz (dt ≈ 0.167 s) lidar frekansında süre temelli min_iz_yasi_s (0.35 s)
    ve min_gorulme (3) ile 0.8 m/s yaklaşan engelin ~0.50 s içinde DUR tetiklemesi."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.6)

    # 6 Hz lidar adımları: dt = 1/6 s ≈ 0.1667 s
    # 0.8 m/s hızla robota (0, 0) doğru yaklaşan engel (başlangıç x=1.8 m)
    dt = 1.0 / 6.0
    x_baslangic = 1.80
    v_engel = 0.80

    tetiklenme_t: float | None = None
    tetiklenme_adimi: int | None = None

    for adim in range(6):  # 0 to 5 (0.0 s - 0.833 s)
        t = adim * dt
        x = x_baslangic - v_engel * t
        merkezler = [(x, 0.0)]
        takipci.guncelle(merkezler, [0.20], t, robot_wz=0.0)

        # Yeni varsayılanlar: min_gorulme=3, min_iz_yasi_s=0.35, yon_tutarlilik=0.7 (son_n=3)
        karar = kalkan_karari(
            izler=takipci.izler(),
            robot_v_odom=(0.0, 0.0),
            robot_p_odom=(0.0, 0.0),
            robot_yaricap=0.20,
            ttc_dur=1.5,
            ttc_yavas=3.0,
        )

        if karar.karar == 'dur':
            tetiklenme_t = t
            tetiklenme_adimi = adim
            break

    # 6 Hz'de min_gorulme=3 ve min_iz_yasi_s=0.35 s ile 3. adımda (t ≈ 0.50 s) tetiklenmeli
    # (Eski min_gorulme=5 kuralında en az 4. adım / 0.67 s veya gecikmeyle 0.8 s sürüyordu)
    assert tetiklenme_adimi is not None and tetiklenme_adimi <= 3, (
        f"6 Hz lidarda DUR kararı {tetiklenme_adimi}. adımda ({tetiklenme_t:.3f} s) geldi; <= 3. adım olmalı!"
    )
    assert tetiklenme_t is not None and tetiklenme_t <= 0.55

    # İzin hız ve yön tutarlılığı doğrulaması
    iz = takipci.izler()[0]
    assert len(iz.ham_hizlar) >= 2
    # Ham hızların dt ile doğru hesaplandığı kontrolü (v ≈ -0.80 m/s)
    for vx, vy in iz.ham_hizlar:
        assert abs(vx - (-v_engel)) < 0.05
        assert abs(vy) < 0.01

    assert yon_tutarliligi(iz.ham_hizlar, son_n=3) >= 0.95


def test_b1_4_hizli_donuste_gorulme_artmaz_sahte_dur_engellenir() -> None:
    """B1.4: Robot hızlı dönerken (|wz| > maks_donus_hizi) hız güncellemesi atlandığında
    görülme sayısının artmaması (iz.gorulme sabit kalması) ve sahte DUR üretilmemesi."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.6, maks_donus_hizi=0.8)

    # 1. Adım: Durağan nesne ilk kez görülüyor (gorulme = 1)
    t0 = 0.0
    takipci.guncelle([(1.50, 0.0)], [0.15], t0, robot_wz=0.0)
    assert len(takipci.izler()) == 1
    assert takipci.izler()[0].gorulme == 1

    # 2. Adım: Gürültü veya ani hareketle tek bir hız ölçümü (v = 0.25 m/s)
    t1 = 0.167
    takipci.guncelle([(1.542, 0.0)], [0.15], t1, robot_wz=0.0)
    iz = takipci.izler()[0]
    assert iz.gorulme == 2
    assert iz.v >= 0.20
    assert len(iz.ham_hizlar) == 1

    # 3. Adımlar: Robot hızlı dönmeye başlıyor (wz = 1.0 rad/s > maks_donus_hizi = 0.8)
    for step in range(2, 6):
        t = step * 0.167
        # Nesne dönen robotun görüş alanında eşleşiyor
        takipci.guncelle([(1.542, 0.0)], [0.15], t, robot_wz=1.0)
        iz = takipci.izler()[0]
        # Hız güncellemesi atlandığı için görülme sayısı ARTMAYIP 2'de kalmalı!
        assert iz.gorulme == 2, (
            f"Hızlı dönüş sırasında {step}. adımda iz.gorulme artmamalı (beklenen 2, bulunan {iz.gorulme})"
        )
        assert iz.yas == step + 1

        # Kalkan kararı çağrıldığında gorulme < min_gorulme (2 < 3) olduğu için 'serbest' kalmalı!
        karar = kalkan_karari(
            izler=takipci.izler(),
            robot_v_odom=(0.0, 0.0),
            robot_p_odom=(0.0, 0.0),
            min_gorulme=3,
            min_iz_yasi_s=0.35,
        )
        assert karar.karar == 'serbest', (
            f"Hızlı dönüşte sahte DUR/YAVAS üretilmemeli! Alınan: {karar.karar} ({karar.gerekce})"
        )


def test_b2_2_odom_kesintisinde_kalkan_kilitli_kalmaz_serbest_ve_odom_yok() -> None:
    """B2.2: Odom geçmişi boşaldığında veya eskidiğinde (> 0.5 s) kalkanın kilitli kalmayıp
    izleri temizlemesi, 'serbest' karara geçmesi ve 'ODOM_YOK' durumunu yayınlaması."""
    # 1. Saf fonksiyon odom_zaman_asimi_kontrolu testi
    # a) Robot pozu geçerli -> zaman aşımı yok
    assert not odom_zaman_asimi_kontrolu(10.0, (1.0, 0.0, 0.0), 9.9, odom_gecmisi_bos=False, odom_zaman_asimi=0.5)
    # b) Robot pozu None ve odom geçmişi boş -> zaman aşımı var
    assert odom_zaman_asimi_kontrolu(10.0, None, None, odom_gecmisi_bos=True, odom_zaman_asimi=0.5)
    # c) Robot pozu None, son poz 0.3 s önce (<= 0.5 s) -> geçici kesinti toleransı
    assert not odom_zaman_asimi_kontrolu(10.3, None, 10.0, odom_gecmisi_bos=False, odom_zaman_asimi=0.5)
    # d) Robot pozu None, son poz 0.6 s önce (> 0.5 s) -> zaman aşımı!
    assert odom_zaman_asimi_kontrolu(10.6, None, 10.0, odom_gecmisi_bos=False, odom_zaman_asimi=0.5)

    # 2. HareketliKalkan düğüm mantığı testi (mock ile)
    from unittest.mock import MagicMock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header

    node = object.__new__(HareketliKalkan)
    node.etkin = True
    node.takipci = IzTakipci()
    node.odom_zaman_asimi = 0.5
    node.tarama_zaman_asimi = 0.5
    node.eski_tarama_tutma_s = 0.5
    node.robot_yaricap = 0.20
    node.min_engel_hizi = 0.20
    node.min_gorulme = 3
    node.min_iz_yasi_s = 0.35
    node.yon_tutarlilik = 0.7
    node.ttc_dur = 1.5
    node.ttc_yavas = 3.0
    node.max_menzil = 5.0
    node.lidar_x = 0.0
    node.lidar_y = 0.0
    node.lidar_yaw = 0.0
    node.kume_esik = 0.10
    node.kume_min_nokta = 3
    node.max_iz_yaricap = 0.35
    node._lidar_tf_alindi = True
    node._marker_yayinla = False
    node._robot_pose_odom = (0.0, 0.0, 0.0)
    node._robot_v_odom = (0.0, 0.0)
    node._robot_wz = 0.0
    node._odom_gecmisi = []
    node._son_scan_t = None
    node._son_scan_duvar_t = 10.0
    node._son_gecerli_pose_t = 10.0
    node._son_karar = 'dur'  # Odom kesilmeden önce kalkan 'dur' durumundaydı!
    node._son_olcek = 0.0
    node._son_gerekce = 'DUR ttc=1.1s'
    node._onceki_gerekce = 'DUR ttc=1.1s'
    node._son_kritik_iz = None
    node._zaman_asimi_uyarildi = False
    node._tarama_yok_uyarildi = False
    node._odom_yok_uyarildi = False

    node.p_cmd = MagicMock()
    node.p_durum = MagicMock()
    node.get_logger = MagicMock()

    # t = 10.6 s'de (> 0.5 s odom kesintisi) tarama geliyor
    scan_msg = LaserScan()
    scan_msg.header = Header()
    scan_msg.header.stamp.sec = 10
    scan_msg.header.stamp.nanosec = int(0.6 * 1e9)
    scan_msg.angle_min = -1.0
    scan_msg.angle_increment = 0.1
    scan_msg.range_min = 0.1
    scan_msg.range_max = 5.0
    scan_msg.ranges = [1.0] * 20

    node._simdi_s = lambda: 10.6
    node._scan_cb(scan_msg)

    # Sonuç: Kalkan 'dur'da kilitli kalmamalı, izler silinmeli, serbest + ODOM_YOK olmalı!
    assert node._son_karar == 'serbest', f"Karar 'serbest' olmalı, bulunan: {node._son_karar}"
    assert node._son_olcek == 1.0
    assert node._son_gerekce == 'ODOM_YOK'
    assert len(node.takipci.izler()) == 0
    assert node.p_durum.publish.call_count == 1
    assert node.p_durum.publish.call_args[0][0].data == 'ODOM_YOK'

    # İkinci taramada "ODOM_YOK" spam yapılmamalı (bir kez yayınla)
    node._scan_cb(scan_msg)
    assert node.p_durum.publish.call_count == 1


def test_b2_3_tarama_zaman_asimi_dur_tutma_ve_tarama_yok() -> None:
    """B2.3: Tarama eskidiğinde son karar 'dur' ise eski_tarama_tutma_s (0.5 s) boyunca
    dur kararını tutma, ardından serbest + 'TARAMA_YOK' durumuna geçme."""
    # 1. Saf fonksiyon tarama_zaman_asimi_yonetimi testi
    # a) Taze tarama (dt = 0.3 s <= 0.5 s)
    eylem, karar, olcek = tarama_zaman_asimi_yonetimi(10.3, 10.0, 'dur', 0.5, 0.5)
    assert eylem == 'normal'
    assert karar == 'dur'

    # b) Tarama zaman aşımına girdi ancak tutma süresi (0.5 s) içinde (dt = 0.7 s -> dt_gecen = 0.2 s < 0.5 s)
    eylem_tut, karar_tut, olcek_tut = tarama_zaman_asimi_yonetimi(10.7, 10.0, 'dur', 0.5, 0.5)
    assert eylem_tut == 'dur_tut'
    assert karar_tut == 'dur'
    assert olcek_tut == 0.0

    # c) Tutma süresi doldu (dt = 1.1 s -> dt_gecen = 0.6 s >= 0.5 s)
    eylem_bit, karar_bit, olcek_bit = tarama_zaman_asimi_yonetimi(11.1, 10.0, 'dur', 0.5, 0.5)
    assert eylem_bit == 'serbest_birak'
    assert karar_bit == 'serbest'
    assert olcek_bit == 1.0

    # d) Son karar 'serbest' iken tarama zaman aşımı (tutma süresi beklenmez, doğrudan serbest)
    eylem_s, karar_s, _ = tarama_zaman_asimi_yonetimi(10.7, 10.0, 'serbest', 0.5, 0.5)
    assert eylem_s == 'serbest_birak'
    assert karar_s == 'serbest'

    # 2. HareketliKalkan._cmd_cb testi (mock ile)
    from unittest.mock import MagicMock
    from geometry_msgs.msg import Twist

    node = object.__new__(HareketliKalkan)
    node.etkin = True
    node.takipci = IzTakipci()
    node.tarama_zaman_asimi = 0.5
    node.eski_tarama_tutma_s = 0.5
    node._son_scan_duvar_t = 10.0
    node._son_karar = 'dur'
    node._son_olcek = 0.0
    node._son_gerekce = 'DUR'
    node._son_kritik_iz = None
    node._robot_pose_odom = (0.0, 0.0, 0.0)
    node._zaman_asimi_uyarildi = False
    node._tarama_yok_uyarildi = False
    node.p_cmd = MagicMock()
    node.p_durum = MagicMock()
    node.get_logger = MagicMock()

    cmd_in = Twist()
    cmd_in.linear.x = 0.40  # İleri hareket isteği

    # t = 10.7 s'de (tutma süresi içinde, dt_gecen = 0.2 s < 0.5 s) cmd_vel geldi
    node._simdi_s = lambda: 10.7
    node._cmd_cb(cmd_in)

    # İleri hız sıfırlanmış olmalı (DUR tutuluyor)
    assert node.p_cmd.publish.call_count == 1
    cikis_twist = node.p_cmd.publish.call_args[0][0]
    assert cikis_twist.linear.x == 0.0, f"DUR tutma süresinde hız 0 olmalı; çıkan: {cikis_twist.linear.x}"

    # t = 11.2 s'de (tutma süresi doldu, dt_gecen = 0.7 s >= 0.5 s) cmd_vel geldi
    node._simdi_s = lambda: 11.2
    node._cmd_cb(cmd_in)

    # Artık serbest bırakılmış ve TARAMA_YOK yayınlanmış olmalı
    assert node._son_karar == 'serbest'
    assert node._son_gerekce == 'TARAMA_YOK'
    assert node.p_durum.publish.call_count == 1
    assert node.p_durum.publish.call_args[0][0].data == 'TARAMA_YOK'
    cikis_twist2 = node.p_cmd.publish.call_args[0][0]
    assert cikis_twist2.linear.x == 0.40


def test_n25_eslestir_saf_fonksiyon() -> None:
    """N25: eslestir saf fonksiyonunun öngörülen konum, dinamik kapı ve açgözlü eşlemesi."""
    # 1. Durağan iki iz ve iki küme: en yakın olanlar eşleşir
    iz1 = Iz(id=1, x=1.0, y=0.0, r=0.2, vx=0.0, vy=0.0, t_son=0.0)
    iz2 = Iz(id=2, x=2.0, y=0.0, r=0.2, vx=0.0, vy=0.0, t_son=0.0)
    kumeler = [(2.05, 0.0), (1.02, 0.0)]  # kume 0 iz2'ye, kume 1 iz1'e yakın

    sonuc = eslestir([iz1, iz2], kumeler, t=0.0, kapi=0.35)
    assert len(sonuc) == 2
    # iz1 -> kume 1 (mesafe 0.02), iz2 -> kume 0 (mesafe 0.05)
    eslesme_dict = dict(sonuc)
    assert eslesme_dict[0] == 1
    assert eslesme_dict[1] == 0
    assert len(sonuc.eslesmeyen_izler) == 0
    assert len(sonuc.eslesmeyen_kumeler) == 0

    # 2. Hareketli iz: öngörülen konum (x + vx*dt) kullanılır
    # iz_hizli: x=1.0'da ama vx=0.5 m/s. t=0.2 s'de öngörülen konumu 1.10 m
    iz_hizli = Iz(id=3, x=1.0, y=0.0, r=0.2, vx=0.5, vy=0.0, t_son=0.0)
    # Küme 1.12'de (öngörülen konuma 0.02 m, eski konuma 0.12 m)
    sonuc_hizli = eslestir([iz_hizli], [(1.12, 0.0)], t=0.2, kapi=0.35)
    assert len(sonuc_hizli) == 1
    assert sonuc_hizli[0] == (0, 0)

    # 3. Dinamik kapı büyümesi: kapi = eslesme_kapi + |v|*dt
    # dt = 0.3 s, v = 0.5 m/s -> dinamik kapı = 0.35 + 0.15 = 0.50 m
    # Eski konuma uzaklık 0.45 m (> taban kapı 0.35 m, ama < dinamik kapı 0.50 m)
    iz_kapi = Iz(id=4, x=1.0, y=0.0, r=0.2, vx=0.5, vy=0.0, t_son=0.0)
    # t=0.3'te öngörü 1.15 m, küme 1.45 m'de (öngörüye 0.30 m < 0.50 m)
    sonuc_kapi = eslestir([iz_kapi], [(1.45, 0.0)], t=0.3, kapi=0.35)
    assert len(sonuc_kapi) == 1

    # 4. Kapı dışındaki küme eşleşmez
    sonuc_bos = eslestir([iz1], [(5.0, 5.0)], t=0.0, kapi=0.35)
    assert len(sonuc_bos) == 0
    assert sonuc_bos.eslesmeyen_izler == [0]
    assert sonuc_bos.eslesmeyen_kumeler == [0]


def test_n25_belirsizlik_daraltilmis() -> None:
    """N25: Belirsizlik sıfırlamasının daraltılması (d2 - d1 < belirsizlik_payi VE d1 > 0.10 m)."""
    # a) d2 - d1 < 0.05 VE d1 > 0.10 m -> BELİRSİZ (hız sıfırlanır, görülme yarıya indirilir)
    # İz (1.0, 0.0)'da, iki aday: (1.12, 0.0) [d1=0.12] ve (1.15, 0.0) [d2=0.15]
    # d1 = 0.12 > 0.10, d2 - d1 = 0.03 < 0.05
    iz_belirsiz = Iz(id=1, x=1.0, y=0.0, r=0.2, vx=0.0, vy=0.0, gorulme=4, yas=5, t_son=0.0)
    # (vx=0: takipçi öngörülen konumu kullanır; hızlı izde öngörü d1<0.10 yapıp belirsizliği zaten çözer)
    kumeler_belirsiz = [(1.12, 0.0), (1.15, 0.0)]
    belirsiz = belirsiz_izler_kumesi([iz_belirsiz], kumeler_belirsiz, t=0.0, kapi=0.35, belirsizlik_payi=0.05)
    assert 0 in belirsiz

    # Takipçide güncelleme ile hızın sıfırlandığı ve görülmenin SIFIRLANMAYIP yarıya indiği (4 -> 2)
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5, belirsizlik_payi=0.05)
    takipci._izler = [iz_belirsiz]
    takipci.guncelle(kumeler_belirsiz, [0.2, 0.2], t=0.1)
    iz_sonuc = [iz for iz in takipci.izler() if iz.id == 1][0]
    assert iz_sonuc.vx == 0.0 and iz_sonuc.vy == 0.0
    assert iz_sonuc.gorulme == 2, f"Görülme 4'ten 2'ye inmeli (sıfırlanmamalı), bulunan: {iz_sonuc.gorulme}"

    # b) d1 <= 0.10 m -> BELİRSİZ DEĞİL (adaylardan biri çok yakın, net eşleşme)
    # d1 = 0.04, d2 = 0.06 (d2 - d1 = 0.02 < 0.05 ama d1 <= 0.10)
    iz_net = Iz(id=2, x=1.0, y=0.0, r=0.2, vx=0.0, vy=0.0, gorulme=2, yas=2, t_son=0.0)
    kumeler_net = [(1.04, 0.0), (1.06, 0.0)]
    belirsiz_net = belirsiz_izler_kumesi([iz_net], kumeler_net, t=0.0, kapi=0.35, belirsizlik_payi=0.05)
    assert len(belirsiz_net) == 0

    # c) d2 - d1 >= 0.05 m -> BELİRSİZ DEĞİL (en iyi aday belirgin biçimde daha yakın)
    # d1 = 0.12, d2 = 0.22 (d2 - d1 = 0.10 >= 0.05)
    iz_ayrik = Iz(id=3, x=1.0, y=0.0, r=0.2, vx=0.0, vy=0.0, gorulme=2, yas=2, t_son=0.0)
    kumeler_ayrik = [(1.12, 0.0), (1.22, 0.0)]
    belirsiz_ayrik = belirsiz_izler_kumesi([iz_ayrik], kumeler_ayrik, t=0.0, kapi=0.35, belirsizlik_payi=0.05)
    assert len(belirsiz_ayrik) == 0


def test_n25_daginik_ortamda_insan_onaylanir_ve_ttc_uretir() -> None:
    """N25 Test (a): 0.4 m arayla 3 durağan küme + aralarından 0.5 m/s geçen insan (6 Hz, gürültü σ 1 cm):
    İnsan izi en geç 0.7 s'de onaylanır (görülme >= 3, |v| ≈ 0.5) ve TTC kararı üretilir."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5, belirsizlik_payi=0.05)
    dt = 1.0 / 6.0  # 6 Hz
    rng = np.random.RandomState(42)

    # 0.4 m arayla 3 durağan küme (x = 1.5 m doğrultusunda)
    duragan_kumeler = [
        (1.50, -0.40),
        (1.50, 0.00),
        (1.50, 0.40),
    ]

    # İnsan: x=1.8'den robota (0,0) doğru vx=-0.5 m/s hızla yaklaşıyor, y=0.20'de (K2 ile K3 arasından geçiyor)
    x_insan_baslangic = 1.80
    y_insan = 0.20
    v_insan = -0.50

    onaylandi = False
    onay_zamani: float | None = None
    ttc_karari_uretildi = False

    # 0.0 s'den 0.70 s'ye kadar (5 adım: step 0 to 4; step 4 t ≈ 0.667 s <= 0.7 s)
    toplam_adim = 5
    for adim in range(toplam_adim):
        t = adim * dt
        x_insan = x_insan_baslangic + v_insan * t

        # Ölçümler (sigma = 0.01 m gürültü ile)
        olcumler: list[tuple[float, float]] = []
        for gx, gy in duragan_kumeler:
            gurultu = rng.normal(0.0, 0.01, size=2)
            olcumler.append((gx + float(gurultu[0]), gy + float(gurultu[1])))

        gurultu_insan = rng.normal(0.0, 0.01, size=2)
        olcumler.append((x_insan + float(gurultu_insan[0]), y_insan + float(gurultu_insan[1])))

        yaricaplar = [0.15] * len(olcumler)
        takipci.guncelle(olcumler, yaricaplar, t, robot_wz=0.0)

        karar = kalkan_karari(
            izler=takipci.izler(),
            robot_v_odom=(0.0, 0.0),
            robot_p_odom=(0.0, 0.0),
            robot_yaricap=0.20,
            min_engel_hizi=0.20,
            min_gorulme=3,
            min_iz_yasi_s=0.35,
            yon_tutarlilik=0.7,
            ttc_dur=1.5,
            ttc_yavas=3.0,
        )

        # İnsan izini tespit et (x ~ x_insan, y ~ y_insan)
        for iz in takipci.izler():
            dx = iz.x - x_insan
            dy = iz.y - y_insan
            if math.hypot(dx, dy) < 0.25:
                # İnsan izi kontrolleri
                if iz.gorulme >= 3 and abs(iz.v - 0.50) <= 0.10:
                    onaylandi = True
                    if onay_zamani is None:
                        onay_zamani = t
                    if karar.karar in ('dur', 'yavas'):
                        ttc_karari_uretildi = True

    assert onaylandi is True, "İnsan izi t <= 0.7 s içinde onaylanmalı (gorulme >= 3, |v| ≈ 0.5)!"
    assert onay_zamani is not None and onay_zamani <= 0.70 + 1e-4, f"Onay zamanı {onay_zamani:.3f} s > 0.70 s!"
    assert ttc_karari_uretildi is True, "TTC kararı ('dur' veya 'yavas') üretilmeli!"


def test_n25_duragan_kumeler_hareketli_sayilmaz() -> None:
    """N25 Test (b): 0.4 m arayla durağan kümeler dağınık ortamda hareketli SAYILMAZ (yanlış alarm yok)."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5, belirsizlik_payi=0.05)
    rng = np.random.RandomState(42)

    duragan_kumeler = [
        (1.50, -0.40),
        (1.50, 0.00),
        (1.50, 0.40),
    ]

    # 30 adım (5 saniye) boyunca gürültülü durağan ölçümler
    for adim in range(30):
        t = adim * (1.0 / 6.0)
        olcumler: list[tuple[float, float]] = []
        for gx, gy in duragan_kumeler:
            gurultu = rng.normal(0.0, 0.01, size=2)
            olcumler.append((gx + float(gurultu[0]), gy + float(gurultu[1])))

        takipci.guncelle(olcumler, [0.15, 0.15, 0.15], t, robot_wz=0.0)

        karar = kalkan_karari(
            izler=takipci.izler(),
            robot_v_odom=(0.0, 0.0),
            robot_p_odom=(0.0, 0.0),
            min_engel_hizi=0.20,
            min_gorulme=3,
            min_iz_yasi_s=0.35,
            yon_tutarlilik=0.7,
            ttc_dur=1.5,
            ttc_yavas=3.0,
        )

        assert karar.karar == 'serbest', (
            f"Adım {adim}'de durağan kümeler için sahte alarm üretilmemeli! Alınan: {karar.karar} ({karar.gerekce})"
        )
        assert karar.olcek == 1.0

        for iz in takipci.izler():
            assert iz.v < 0.20, f"Durağan izin hızı {iz.v:.3f} >= min_engel_hizi 0.20 m/s!"


def test_n25_bir_tarama_kacirilan_insan_ayni_kimlikle_devam_eder() -> None:
    """N25 Test (c): 1 tarama kaçırılan insan izi dinamik kapı ve sabit hız öngörüsüyle aynı kimlikle devam eder."""
    takipci = IzTakipci(kapi=0.35, alfa=0.4, silme_suresi=0.5, belirsizlik_payi=0.05)
    dt = 1.0 / 6.0  # ~0.1667 s
    v_insan = 0.50  # m/s

    # Adım 0 (t = 0.0 s): İnsan ilk kez (1.0, 0.0)'da görülüyor
    t0 = 0.0
    x0 = 1.0
    takipci.guncelle([(x0, 0.0)], [0.20], t0)
    assert len(takipci.izler()) == 1
    insan_id = takipci.izler()[0].id

    # Adım 1 (t = 0.167 s): İnsan (1.0833, 0.0)'da görülüyor -> hız oturuyor
    t1 = dt
    x1 = x0 + v_insan * t1
    takipci.guncelle([(x1, 0.0)], [0.20], t1)
    izler1 = takipci.izler()
    assert len(izler1) == 1
    assert izler1[0].id == insan_id
    assert abs(izler1[0].v - v_insan) < 0.05

    # Adım 2 (t = 0.333 s): 1 TARAMA KAÇIRILDI (sensör boş veya küme tespit edilemedi)
    t2 = 2 * dt
    takipci.guncelle([], [], t2)
    izler2 = takipci.izler()
    # 0.5 s silme süresi dolmadığı için iz silinmemeli ve hızı korunmalı
    assert len(izler2) == 1
    assert izler2[0].id == insan_id
    assert abs(izler2[0].v - v_insan) < 0.05

    # Adım 3 (t = 0.500 s): İnsan tekrar görülüyor (x3 = 1.0 + 0.5 * 0.5 = 1.25 m)
    # Öngörülen konum (1.0833 + 0.5 * 0.333 = 1.25 m) ve genişleyen kapı (0.35 + 0.5*0.333 = 0.516 m)
    t3 = 3 * dt
    x3 = x0 + v_insan * t3
    takipci.guncelle([(x3, 0.0)], [0.20], t3)
    izler3 = takipci.izler()

    # İz kimliği KOPMAMALI, aynı ID ile devam etmeli
    assert len(izler3) == 1, f"Tek iz beklenirken {len(izler3)} iz bulundu (kimlik kopmuş olabilir)"
    assert izler3[0].id == insan_id, f"İz ID'si değişti: beklenen {insan_id}, bulunan {izler3[0].id}"
    assert abs(izler3[0].v - v_insan) <= 0.10, f"Hız kestirimi bozuldu: {izler3[0].v:.3f} m/s"
    assert izler3[0].gorulme >= 2
