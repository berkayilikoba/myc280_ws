#!/usr/bin/env python3
"""mesh_karsilastir.py: renkli mesh'leri gerçek kamera pozlarından görüntüleyip gerçek kareyle yan yana koyar ve sayısal ölçütler üretir.

  python3 mesh_karsilastir.py <cikti.png> <sahne> <mesh.ply>... [--kareler 0,300,700 --aralik 100 --dx 0.09 --dy 0.01 --dyaw -1.0]

Mesh yüzeyinden yoğun nokta örneklenir, z-tamponlu izdüşümle 640x360 görüntü ve derinlik haritası oluşturulur.
Satır = kare; sütun = gerçek, mesh_1, mesh_2, ...
Sayısal ölçütler <cikti>.yaml dosyasına yazılır ve terminale özet tablosu basılır.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import yaml

from oda_tsdf import duzeltme_matrisi, poz_matrisi


def ciz(noktalar, renkler, T_wc, K, w=640, h=360):
    T_cw = np.linalg.inv(T_wc)
    p = noktalar @ T_cw[:3, :3].T + T_cw[:3, 3]
    on = p[:, 2] > 0.05
    p, c = p[on], renkler[on]
    u = np.round(K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2]).astype(int)
    v = np.round(K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]).astype(int)
    ic = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z, c = u[ic], v[ic], p[ic, 2], c[ic]
    sira = np.argsort(-z)  # uzaktan yakına: yakın olan en son yazılır
    img = np.zeros((h, w, 3), np.uint8)
    depth = np.zeros((h, w), np.float32)
    if len(sira) > 0:
        c_sira = c[sira]
        z_sira = z[sira].astype(np.float32)
        for du in (0, 1):  # 2x2 kaplama, delikleri azaltır
            for dv in (0, 1):
                uu, vv = np.clip(u[sira] + du, 0, w - 1), np.clip(v[sira] + dv, 0, h - 1)
                img[vv, uu] = c_sira
                depth[vv, uu] = z_sira
    return img, depth


def olculeri_hesapla(gercek_rgb, gercek_d, mesh_rgb, mesh_d):
    """Kare ve mesh başına ölçütleri hesaplar.

    Yalnızca iki derinliğin de geçerli olduğu ve gerçek derinlik <= 3.0 m pikseller dikkate alınır.
    """
    # Gerçek geçerli pikseller (<= 3.0 m)
    gt_gecerli = (gercek_d > 0.0) & (gercek_d <= 3.0)
    gt_gecerli_sayisi = int(np.count_nonzero(gt_gecerli))

    # Kapsama: gerçek geçerli piksellerin mesh'te dolu oranı
    mesh_dolu_gt = gt_gecerli & (mesh_d > 0.0)
    if gt_gecerli_sayisi > 0:
        kapsama = float(np.count_nonzero(mesh_dolu_gt) / gt_gecerli_sayisi)
    else:
        kapsama = 0.0

    # Ortak geçerli maske
    mask = mesh_dolu_gt
    mask_sayisi = int(np.count_nonzero(mask))

    if mask_sayisi > 0:
        # 1. Derinlik mutlak medyan farkı (cm)
        d_fark_cm = np.abs(mesh_d[mask] - gercek_d[mask]) * 100.0
        derinlik_medyan_fark_cm = float(np.median(d_fark_cm))

        # 2. |fark| < 3 cm oranı
        fark_alti_3cm_orani = float(np.mean(d_fark_cm < 3.0))

        # 3. Renk farkı: Lab uzayında ΔE medyanı (cv2.cvtColor ile)
        lab_gt = cv2.cvtColor(gercek_rgb, cv2.COLOR_BGR2LAB).astype(np.float32)
        lab_mesh = cv2.cvtColor(mesh_rgb, cv2.COLOR_BGR2LAB).astype(np.float32)
        diff_lab = lab_mesh[mask] - lab_gt[mask]
        delta_e = np.linalg.norm(diff_lab, axis=1)
        renk_delta_e_medyan = float(np.median(delta_e))
    else:
        derinlik_medyan_fark_cm = 0.0
        fark_alti_3cm_orani = 0.0
        renk_delta_e_medyan = 0.0

    # 4. Keskinlik: mesh görüntüsünün Laplacian varyansı / gerçek görüntününki
    gray_gt = cv2.cvtColor(gercek_rgb, cv2.COLOR_BGR2GRAY)
    gray_mesh = cv2.cvtColor(mesh_rgb, cv2.COLOR_BGR2GRAY)
    lap_gt = cv2.Laplacian(gray_gt, cv2.CV_64F)
    lap_mesh = cv2.Laplacian(gray_mesh, cv2.CV_64F)

    if mask_sayisi > 1:
        var_gt = float(np.var(lap_gt[mask]))
        var_mesh = float(np.var(lap_mesh[mask]))
        keskinlik = float(var_mesh / var_gt) if var_gt > 1e-12 else 0.0
    else:
        keskinlik = 0.0

    # 5. Kenar uyumu: Canny kenarları (gerçek ve mesh) arasında 3 px tolerans ile F1
    canny_gt = cv2.Canny(gray_gt, 100, 200) > 0
    canny_mesh = cv2.Canny(gray_mesh, 100, 200) > 0

    edges_gt = canny_gt & mask
    edges_mesh = canny_mesh & mask

    n_gt_edges = int(np.count_nonzero(edges_gt))
    n_mesh_edges = int(np.count_nonzero(edges_mesh))

    if n_gt_edges > 0 and n_mesh_edges > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        dilated_gt = cv2.dilate(edges_gt.astype(np.uint8), kernel) > 0
        dilated_mesh = cv2.dilate(edges_mesh.astype(np.uint8), kernel) > 0

        tp_mesh = np.count_nonzero(edges_mesh & dilated_gt)
        tp_gt = np.count_nonzero(edges_gt & dilated_mesh)

        precision = float(tp_mesh / n_mesh_edges)
        recall = float(tp_gt / n_gt_edges)

        if precision + recall > 0.0:
            kenar_f1 = float(2.0 * precision * recall / (precision + recall))
        else:
            kenar_f1 = 0.0
    elif n_gt_edges == 0 and n_mesh_edges == 0:
        kenar_f1 = 1.0
    else:
        kenar_f1 = 0.0

    return {
        "derinlik_medyan_fark_cm": round(derinlik_medyan_fark_cm, 4),
        "fark_alti_3cm_orani": round(fark_alti_3cm_orani, 4),
        "kapsama": round(kapsama, 4),
        "renk_delta_e_medyan": round(renk_delta_e_medyan, 4),
        "keskinlik": round(keskinlik, 4),
        "kenar_uyumu_f1": round(kenar_f1, 4),
    }


def main():
    a = argparse.ArgumentParser()
    a.add_argument("cikti", type=Path)
    a.add_argument("sahne", type=Path)
    a.add_argument("mesh", type=Path, nargs="+")
    a.add_argument("--kareler", default=None, help="Virgülle ayrılmış kare listesi (örn: 0,300,700,1100)")
    a.add_argument("--aralik", type=int, default=100, help="--kareler verilmezse örnekleme aralığı (varsayılan: 100)")
    a.add_argument("--dx", type=float, default=0.09)
    a.add_argument("--dy", type=float, default=0.01)
    a.add_argument("--dyaw", type=float, default=-1.0)
    a.add_argument("--nokta", type=int, default=6_000_000)
    g = a.parse_args()

    cfg = yaml.safe_load(open(g.sahne / "vlmaps_config.yaml"))
    K = np.array(cfg["cam_calib_mat"], float).reshape(3, 3)
    govde = np.atleast_2d(np.loadtxt(g.sahne / "poses.txt"))
    kamera = np.atleast_2d(np.loadtxt(g.sahne / "poses_kamera.txt"))
    dT = duzeltme_matrisi(g.dx, g.dy, g.dyaw)

    bulutlar = []
    for m in g.mesh:
        mesh = o3d.io.read_triangle_mesh(str(m))
        pc = mesh.sample_points_uniformly(g.nokta)
        bulutlar.append((np.asarray(pc.points), (np.asarray(pc.colors)[:, ::-1] * 255).astype(np.uint8)))

    # Mesh isimleri
    parent_names = [m.parent.name for m in g.mesh]
    if len(set(parent_names)) == len(g.mesh) and all(p and p != "." for p in parent_names):
        mesh_isimleri = parent_names
    else:
        mesh_isimleri = [m.stem if not m.parent.name or m.parent.name == "." else f"{m.parent.name}_{m.stem}" for m in g.mesh]

    etiketler = ["gercek"] + mesh_isimleri

    # Kare listesini belirle
    toplam_kare = min(len(govde), len(kamera))
    if g.kareler is not None:
        kare_listesi = [int(s.strip()) for s in g.kareler.split(",") if s.strip()]
        gorsel_kare_listesi = kare_listesi
    else:
        kare_listesi = list(range(0, toplam_kare, g.aralik))
        gorsel_kare_listesi = kare_listesi[:4]

    satirlar = []
    kareler_dict = {}
    mesh_kareler_dict = {m: {} for m in mesh_isimleri}

    for i in kare_listesi:
        if i >= toplam_kare:
            continue
        T_wb, T_wc = poz_matrisi(govde[i]), poz_matrisi(kamera[i])
        T = T_wb @ dT @ np.linalg.inv(T_wb) @ T_wc

        # Gerçek RGB oku
        gercek_yolu = g.sahne / "rgb" / f"{i:06d}.png"
        if not gercek_yolu.exists():
            gercek_yolu = g.sahne / "rgb" / f"{i:06d}.jpg"
        if not gercek_yolu.exists():
            continue
        gercek = cv2.imread(str(gercek_yolu))
        if gercek is None:
            continue
        gercek_res = cv2.resize(gercek, (640, 360))

        # Gerçek Derinlik oku: <sahne>/depth/NNNNNN.npy
        depth_yolu = g.sahne / "depth" / f"{i:06d}.npy"
        if not depth_yolu.exists():
            continue
        gercek_depth = np.load(str(depth_yolu)).astype(np.float32)
        if gercek_depth.shape != (360, 640):
            gercek_depth = cv2.resize(gercek_depth, (640, 360), interpolation=cv2.INTER_NEAREST)

        # Mesh'leri çiz ve ölçütleri hesapla
        cizimler = [ciz(p, c, T, K) for p, c in bulutlar]  # [(img, depth), ...]

        kareler_dict[i] = {}
        for m_idx, (m_img, m_depth) in enumerate(cizimler):
            m_isim = mesh_isimleri[m_idx]
            olculer = olculeri_hesapla(gercek_res, gercek_depth, m_img, m_depth)
            kareler_dict[i][m_isim] = olculer
            mesh_kareler_dict[m_isim][i] = olculer

        # Görsel PNG panelleri (yalnızca görsel kareler için)
        if i in gorsel_kare_listesi:
            paneller = [gercek_res.copy()] + [m_img.copy() for m_img, _ in cizimler]
            for p, e in zip(paneller, etiketler):
                cv2.putText(p, f"{e} #{i}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            satirlar.append(np.hstack(paneller))

    # Görsel PNG kaydet
    if satirlar:
        g.cikti.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(g.cikti), np.vstack(satirlar))
        print("yazildi", g.cikti)

    # Mesh başına ortalama hesapla
    metrik_anahtarlari = [
        "derinlik_medyan_fark_cm",
        "fark_alti_3cm_orani",
        "kapsama",
        "renk_delta_e_medyan",
        "keskinlik",
        "kenar_uyumu_f1",
    ]
    ortalama_dict = {m: {} for m in mesh_isimleri}
    for m in mesh_isimleri:
        if mesh_kareler_dict[m]:
            for key in metrik_anahtarlari:
                degerler = [mesh_kareler_dict[m][k][key] for k in mesh_kareler_dict[m]]
                ortalama_dict[m][key] = round(float(np.mean(degerler)), 4)
        else:
            for key in metrik_anahtarlari:
                ortalama_dict[m][key] = 0.0

    # YAML çıktısını hazırla ve kaydet
    yaml_cikti = {
        "ortalama": ortalama_dict,
        "kareler": kareler_dict,
        "meshler": {
            m: {
                "ortalama": ortalama_dict[m],
                "kareler": mesh_kareler_dict[m],
            }
            for m in mesh_isimleri
        },
    }
    for m in mesh_isimleri:
        if m not in yaml_cikti:
            yaml_cikti[m] = {
                "ortalama": ortalama_dict[m],
                "kareler": mesh_kareler_dict[m],
            }

    yaml_yolu = g.cikti.with_suffix(".yaml")
    with open(yaml_yolu, "w", encoding="utf-8") as f:
        yaml.safe_dump(yaml_cikti, f, sort_keys=False, allow_unicode=True)
    print("yazildi", yaml_yolu)

    alt_yaml_yolu = Path(str(g.cikti) + ".yaml")
    if alt_yaml_yolu != yaml_yolu:
        try:
            with open(alt_yaml_yolu, "w", encoding="utf-8") as f:
                yaml.safe_dump(yaml_cikti, f, sort_keys=False, allow_unicode=True)
        except Exception:
            pass

    # Terminale mesh başına özet tablosu
    baslik = f"{'Mesh':<20} {'Med|Δd| (cm)':<14} {'|Δd|<3cm':<10} {'Kapsama':<10} {'Renk ΔE':<10} {'Keskinlik':<11} {'Kenar F1':<10}"
    cizgi = "=" * len(baslik)
    print("\n" + cizgi)
    print("MESH KARSILASTIRMA OZETI (ORTALAMA)")
    print(cizgi)
    print(baslik)
    print("-" * len(baslik))
    for m_isim in mesh_isimleri:
        ort = ortalama_dict[m_isim]
        satir = (
            f"{m_isim:<20} "
            f"{ort['derinlik_medyan_fark_cm']:<14.2f} "
            f"{ort['fark_alti_3cm_orani']:<10.3f} "
            f"{ort['kapsama']:<10.3f} "
            f"{ort['renk_delta_e_medyan']:<10.2f} "
            f"{ort['keskinlik']:<11.3f} "
            f"{ort['kenar_uyumu_f1']:<10.3f}"
        )
        print(satir)
    print(cizgi + "\n")


if __name__ == "__main__":
    main()
