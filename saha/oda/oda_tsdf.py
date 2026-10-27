#!/usr/bin/env python3
"""Kamera RGB-D sahnelerinden TSDF füzyonu ile renkli 3B oda mesh'i üretir."""

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import open3d as o3d
import yaml

try:
    from scipy.spatial import cKDTree as _cKDTree
except ImportError:
    _cKDTree = None

try:
    import torch
except ImportError:
    torch = None

try:
    from ultralytics import YOLO

    ULTRALYTICS_MEVCUT = True
except ImportError:
    YOLO = None
    ULTRALYTICS_MEVCUT = False


def poz_matrisi(satir: np.ndarray) -> np.ndarray:
    """[x, y, z, qx, qy, qz, qw] satırını 4x4 homojen matrise çevirir."""
    x, y, z, qx, qy, qz, qw = satir
    norm = float(np.linalg.norm([qx, qy, qz, qw]))
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    T = np.eye(4, dtype=np.float64)
    T[0, 0] = 1.0 - 2.0 * (qy * qy + qz * qz)
    T[0, 1] = 2.0 * (qx * qy - qz * qw)
    T[0, 2] = 2.0 * (qx * qz + qy * qw)
    T[1, 0] = 2.0 * (qx * qy + qz * qw)
    T[1, 1] = 1.0 - 2.0 * (qx * qx + qz * qz)
    T[1, 2] = 2.0 * (qy * qz - qx * qw)
    T[2, 0] = 2.0 * (qx * qz - qy * qw)
    T[2, 1] = 2.0 * (qy * qz + qx * qw)
    T[2, 2] = 1.0 - 2.0 * (qx * qx + qy * qy)
    T[:3, 3] = [x, y, z]
    return T


def duzeltme_matrisi(dx: float, dy: float, dyaw_deg: float) -> np.ndarray:
    """Gövde çerçevesinde (dx, dy, dyaw) kalibrasyon düzeltme matrisi."""
    rad = np.deg2rad(dyaw_deg)
    c, s = float(np.cos(rad)), float(np.sin(rad))
    return np.array(
        [
            [c, -s, 0.0, dx],
            [s, c, 0.0, dy],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def obj_ve_mtl_kaydet(
    cikti_dizin: Path,
    koseler: np.ndarray,
    normaller: np.ndarray,
    ucgenler: np.ndarray,
    etiketler: np.ndarray,
    merkezler: np.ndarray,
) -> None:
    """Gazebo Classic için renk kümeli OBJ ve MTL dosyalarını yazar."""
    mtl_yolu = cikti_dizin / "oda_mesh.mtl"
    obj_yolu = cikti_dizin / "oda_mesh.obj"

    mtl_satirlar = []
    for k, kd in enumerate(merkezler):
        ka = 0.6 * kd
        mtl_satirlar.append(f"newmtl m{k:02d}")
        mtl_satirlar.append(f"Ka {ka[0]:.4f} {ka[1]:.4f} {ka[2]:.4f}")
        mtl_satirlar.append(f"Kd {kd[0]:.4f} {kd[1]:.4f} {kd[2]:.4f}")
        mtl_satirlar.append("Ks 0.0000 0.0000 0.0000")
        mtl_satirlar.append("d 1.0")
        mtl_satirlar.append("illum 1\n")
    mtl_yolu.write_text("\n".join(mtl_satirlar), encoding="utf-8")

    with obj_yolu.open("w", encoding="utf-8") as f:
        f.write("mtllib oda_mesh.mtl\n")
        for v in koseler:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for vn in normaller:
            f.write(f"vn {vn[0]:.6f} {vn[1]:.6f} {vn[2]:.6f}\n")
        ucgen_1 = ucgenler + 1
        for k in range(len(merkezler)):
            grup = ucgen_1[etiketler == k]
            if len(grup) == 0:
                continue
            f.write(f"usemtl m{k:02d}\n")
            for a, b, c in grup:
                f.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")


@dataclass
class KareBilgisi:
    """Tek bir RGB-D karesinin dosya yolları ve poz bilgileri."""

    indis: int
    rgb_yolu: Path
    depth_yolu: Path
    T_wc_ilk: np.ndarray
    T_wc_sahne: np.ndarray
    T_wc_son: np.ndarray
    laplacian_var: float = 0.0
    insan_maskesi: np.ndarray | None = None


@dataclass
class SahneVerisi:
    """Bir sahnenin kamera parametreleri, seçilen kareleri ve hizalama verileri."""

    ad: str
    dizin: Path
    fx: float
    fy: float
    cx: float
    cy: float
    intrinsic: o3d.camera.PinholeCameraIntrinsic
    kareler: list[KareBilgisi] = field(default_factory=list)
    toplam_ham_kare: int = 0
    max_w_atilan: int = 0
    aday_kare: int = 0
    bulanik_atilan: int = 0
    esik_laplacian_var: float = 0.0
    T_sahne: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))
    yuzey_noktalari: np.ndarray = field(
        default_factory=lambda: np.empty((0, 3), dtype=np.float64)
    )


def rotasyon_acisi_deg(R: np.ndarray) -> float:
    """3x3 rotasyon matrisinin toplam dönme açısını (derece) döndürür."""
    iz = float(np.clip((np.trace(R[:3, :3]) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.rad2deg(np.arccos(iz)))


def yaw_ve_oteleme_kisitla(T: np.ndarray, dz_sinir: float = 0.05) -> tuple[np.ndarray, float, float, float, float]:
    """4x4 dönüşümden roll/pitch'i atıp yalnızca yaw + x,y + (küçük z) bırakır."""
    yaw = float(np.arctan2(T[1, 0], T[0, 0]))
    yaw_deg = float(np.rad2deg(yaw))
    dx = float(T[0, 3])
    dy = float(T[1, 3])
    dz = float(np.clip(T[2, 3], -dz_sinir, dz_sinir))
    c, s = float(np.cos(yaw)), float(np.sin(yaw))
    T_kisi = np.array(
        [
            [c, -s, 0.0, dx],
            [s, c, 0.0, dy],
            [0.0, 0.0, 1.0, dz],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return T_kisi, dx, dy, dz, yaw_deg


def kare_rgbd_yukle(
    rgb_yolu: Path,
    depth_yolu: Path,
    zmax: float,
    maske: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray] | None:
    """RGB (uint8 HxWx3) ve temizlenmiş Depth (float32 HxW) dizilerini yükler."""
    bgr = cv2.imread(str(rgb_yolu), cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    depth = np.load(str(depth_yolu)).astype(np.float32)
    depth[(depth > zmax) | (depth < 0.0) | ~np.isfinite(depth)] = 0.0
    if maske is not None:
        depth[maske] = 0.0
    depth = np.ascontiguousarray(depth)
    return rgb, depth


def sahne_karelerini_hazirla(
    sahne_yolu: Path,
    dT: np.ndarray,
    her: int,
    max_w: float,
    bulanik_yuzde: float,
    bulanik_aktif: bool,
) -> SahneVerisi:
    """Sahne dosyalarını okur, açısal hız ve Laplacian bulanıklık süzgecini uygular."""
    sahne = sahne_yolu.expanduser().resolve()
    with (sahne / "vlmaps_config.yaml").open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    K = cfg["cam_calib_mat"]
    fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 360, fx, fy, cx, cy)

    poses = np.atleast_2d(np.loadtxt(sahne / "poses.txt", dtype=np.float64))
    poses_kam = np.atleast_2d(np.loadtxt(sahne / "poses_kamera.txt", dtype=np.float64))
    zaman = np.atleast_2d(np.loadtxt(sahne / "zaman.txt", dtype=np.float64))

    qx, qy, qz, qw = poses[:, 3], poses[:, 4], poses[:, 5], poses[:, 6]
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    if len(poses) >= 2:
        w = np.gradient(np.unwrap(yaw), zaman[:, 1])
    else:
        w = np.zeros(len(poses), dtype=np.float64)

    n_kare = min(len(poses), len(poses_kam), len(zaman))
    adaylar: list[KareBilgisi] = []
    max_w_atilan = 0

    for i in range(0, n_kare, her):
        if abs(w[i]) > max_w:
            max_w_atilan += 1
            continue
        rgb_yolu = sahne / "rgb" / f"{i:06d}.png"
        depth_yolu = sahne / "depth" / f"{i:06d}.npy"
        if not rgb_yolu.exists() or not depth_yolu.exists():
            continue

        gri = cv2.imread(str(rgb_yolu), cv2.IMREAD_GRAYSCALE)
        if gri is None:
            continue
        lap_var = float(cv2.Laplacian(gri, cv2.CV_64F).var())

        T_wb = poz_matrisi(poses[i])
        T_wc = poz_matrisi(poses_kam[i])
        T_bc = np.linalg.inv(T_wb) @ T_wc
        T_wc_duz = T_wb @ dT @ T_bc

        adaylar.append(
            KareBilgisi(
                indis=i,
                rgb_yolu=rgb_yolu,
                depth_yolu=depth_yolu,
                T_wc_ilk=T_wc_duz.copy(),
                T_wc_sahne=T_wc_duz.copy(),
                T_wc_son=T_wc_duz.copy(),
                laplacian_var=lap_var,
            )
        )

    bulanik_atilan = 0
    esik_var = 0.0
    if bulanik_aktif and bulanik_yuzde > 0.0 and len(adaylar) > 1:
        oran = float(np.clip(bulanik_yuzde, 0.0, 95.0)) / 100.0
        at_sayisi = int(round(len(adaylar) * oran))
        if 0 < at_sayisi < len(adaylar):
            varyanslar = np.array([k.laplacian_var for k in adaylar], dtype=np.float64)
            sirali_indis = np.argsort(varyanslar, kind="mergesort")
            atilan_kume = set(sirali_indis[:at_sayisi].tolist())
            esik_var = float(varyanslar[sirali_indis[at_sayisi]])
            secilenler = [k for idx, k in enumerate(adaylar) if idx not in atilan_kume]
            bulanik_atilan = len(adaylar) - len(secilenler)
        else:
            secilenler = adaylar
    else:
        secilenler = adaylar

    print(
        f"[BULANIKLIK] {sahne.name}: aday={len(adaylar)} (max_w atılan={max_w_atilan}), "
        f"bulanık atılan={bulanik_atilan} (eşik Laplacian var={esik_var:.2f}), "
        f"kalan={len(secilenler)}"
    )

    return SahneVerisi(
        ad=sahne.name,
        dizin=sahne,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        intrinsic=intrinsic,
        kareler=secilenler,
        toplam_ham_kare=n_kare,
        max_w_atilan=max_w_atilan,
        aday_kare=len(adaylar),
        bulanik_atilan=bulanik_atilan,
        esik_laplacian_var=esik_var,
    )


def sinif_listesi_coz(siniflar_str: str) -> list[int]:
    """Virgülle ayrılmış sınıf ID stringini tamsayı listesine çevirir."""
    siniflar = []
    for parca in str(siniflar_str).split(","):
        parca = parca.strip()
        if parca:
            try:
                siniflar.append(int(parca))
            except ValueError:
                pass
    return siniflar if siniflar else [0]


def insan_maskeleri_uygula(
    sahneler: list[SahneVerisi],
    model: Any,
    device: Any,
    guven: float,
    siniflar: list[int],
    genislet_px: int,
    batch_boyutu: int = 16,
) -> dict[str, Any]:
    """Füzyona girecek karelerde YOLO segmentasyonu ile insan maskeleri üretir ve KareBilgisi'ne atar."""
    sahne_istatistik: dict[str, Any] = {}
    toplam_kare_hepsi = 0
    toplam_maskeli_hepsi = 0
    tum_kare_oranlari: list[float] = []

    for s in sahneler:
        s_toplam = len(s.kareler)
        toplam_kare_hepsi += s_toplam
        s_maskeli = 0
        s_oranlar: list[float] = []

        if s_toplam == 0:
            sahne_istatistik[s.ad] = {
                "toplam_kare": 0,
                "maskelenen_kare": 0,
                "maskelenen_kare_orani": 0.0,
                "ortalama_maskeli_piksel_orani": 0.0,
            }
            continue

        for b_start in range(0, s_toplam, batch_boyutu):
            batch_kareler = s.kareler[b_start : b_start + batch_boyutu]
            batch_bgr = []
            for k in batch_kareler:
                bgr = cv2.imread(str(k.rgb_yolu), cv2.IMREAD_COLOR)
                if bgr is None:
                    bgr = np.zeros((360, 640, 3), dtype=np.uint8)
                batch_bgr.append(bgr)

            try:
                results = model.predict(
                    source=batch_bgr,
                    batch=len(batch_bgr),
                    conf=guven,
                    device=device,
                    classes=siniflar,
                    retina_masks=True,
                    verbose=False,
                )
            except Exception as e:
                if device != "cpu":
                    print(
                        f"[İNSAN MASKE] UYARI: Cihaz {device} üzerinde tahmin başarısız ({e}), "
                        f"CPU deneniyor..."
                    )
                    device = "cpu"
                    results = model.predict(
                        source=batch_bgr,
                        batch=len(batch_bgr),
                        conf=guven,
                        device="cpu",
                        classes=siniflar,
                        retina_masks=True,
                        verbose=False,
                    )
                else:
                    raise

            for kare, res, bgr_img in zip(batch_kareler, results, batch_bgr):
                h, w = bgr_img.shape[:2]
                birlesik_maske = np.zeros((h, w), dtype=np.uint8)

                if (
                    res.masks is not None
                    and res.masks.data is not None
                    and len(res.masks.data) > 0
                ):
                    masks_data = res.masks.data
                    if hasattr(masks_data, "cpu"):
                        masks_np = masks_data.cpu().numpy()
                    else:
                        masks_np = np.asarray(masks_data)

                    if res.boxes is not None and res.boxes.cls is not None:
                        cls_data = res.boxes.cls
                        if hasattr(cls_data, "cpu"):
                            det_cls = cls_data.int().cpu().tolist()
                        else:
                            det_cls = [int(x) for x in cls_data]
                    else:
                        det_cls = [siniflar[0]] * len(masks_np)

                    for m_idx, inst_mask in enumerate(masks_np):
                        cls_id = det_cls[m_idx] if m_idx < len(det_cls) else -1
                        if cls_id in siniflar:
                            inst_2d = np.squeeze(inst_mask)
                            if inst_2d.ndim != 2:
                                continue
                            m_bin = (inst_2d > 0.5).astype(np.uint8)
                            if m_bin.shape != (h, w):
                                m_bin = cv2.resize(
                                    m_bin, (w, h), interpolation=cv2.INTER_NEAREST
                                )
                            birlesik_maske = np.maximum(birlesik_maske, m_bin)

                if genislet_px > 0 and np.any(birlesik_maske > 0):
                    k_boyut = 2 * genislet_px + 1
                    cekirdek = cv2.getStructuringElement(
                        cv2.MORPH_ELLIPSE, (k_boyut, k_boyut)
                    )
                    birlesik_maske = cv2.dilate(birlesik_maske, cekirdek)

                maske_bool = birlesik_maske > 0
                kare.insan_maskesi = maske_bool

                piksel_orani = float(np.mean(maske_bool))
                s_oranlar.append(piksel_orani)
                tum_kare_oranlari.append(piksel_orani)
                if np.any(maske_bool):
                    s_maskeli += 1
                    toplam_maskeli_hepsi += 1

        s_ort_piksel_orani = float(np.mean(s_oranlar)) if s_oranlar else 0.0
        s_kare_orani = float(s_maskeli / s_toplam) if s_toplam > 0 else 0.0
        print(
            f"[İNSAN MASKE] {s.ad}: {s_maskeli}/{s_toplam} kare maskelendi "
            f"(%{s_kare_orani * 100.0:.1f}), ortalama maskeli piksel oranı=%{s_ort_piksel_orani * 100.0:.2f}"
        )

        sahne_istatistik[s.ad] = {
            "toplam_kare": int(s_toplam),
            "maskelenen_kare": int(s_maskeli),
            "maskelenen_kare_orani": round(s_kare_orani, 4),
            "ortalama_maskeli_piksel_orani": round(s_ort_piksel_orani, 5),
        }

    genel_ort_piksel = float(np.mean(tum_kare_oranlari)) if tum_kare_oranlari else 0.0
    genel_kare_orani = (
        float(toplam_maskeli_hepsi / toplam_kare_hepsi)
        if toplam_kare_hepsi > 0
        else 0.0
    )
    if sahneler:
        print(
            f"[İNSAN MASKE] Toplam: {toplam_maskeli_hepsi}/{toplam_kare_hepsi} kare maskelendi "
            f"(%{genel_kare_orani * 100.0:.1f}), ortalama maskeli piksel oranı=%{genel_ort_piksel * 100.0:.2f}"
        )

    return {
        "toplam_kare": int(toplam_kare_hepsi),
        "maskelenen_kare": int(toplam_maskeli_hepsi),
        "maskelenen_kare_orani": round(genel_kare_orani, 4),
        "ortalama_maskeli_piksel_orani": round(genel_ort_piksel, 5),
        "sahneler": sahne_istatistik,
    }


def sahneleri_hizala_ve_kaba_tsdf(
    sahneler: list[SahneVerisi],
    zmax: float,
    sahne_hizala_aktif: bool,
    tutarlilik_aktif: bool,
) -> dict[str, Any]:
    """Her sahne için 0.04 m kaba TSDF oluşturur; sahneleri referansa ICP ile hizalar."""
    rapor: dict[str, Any] = {
        "aktif": bool(sahne_hizala_aktif),
        "referans_sahne": sahneler[0].ad if sahneler else "",
        "sahneler": {},
    }
    if not sahneler:
        return rapor

    if not sahne_hizala_aktif and not tutarlilik_aktif:
        for s in sahneler:
            rapor["sahneler"][s.ad] = {
                "referans": s is sahneler[0],
                "uygulandi": False,
                "dx": 0.0,
                "dy": 0.0,
                "dz": 0.0,
                "dyaw_deg": 0.0,
                "oteleme_m": 0.0,
                "fitness": 1.0 if s is sahneler[0] else 0.0,
                "inlier_rmse": 0.0,
            }
        return rapor

    kaba_voksel = 0.04
    down_voksel = 0.03
    referans_pcd: o3d.geometry.PointCloud | None = None

    for idx, s in enumerate(sahneler):
        kaba_hacim = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=kaba_voksel,
            sdf_trunc=4.0 * kaba_voksel,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        )
        for kare in s.kareler:
            yuklenen = kare_rgbd_yukle(
                kare.rgb_yolu, kare.depth_yolu, zmax, kare.insan_maskesi
            )
            if yuklenen is None:
                continue
            rgb, depth = yuklenen
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(rgb),
                o3d.geometry.Image(depth),
                depth_scale=1.0,
                depth_trunc=zmax,
                convert_rgb_to_intensity=False,
            )
            kaba_hacim.integrate(rgbd, s.intrinsic, np.linalg.inv(kare.T_wc_ilk))

        pcd_tam = kaba_hacim.extract_point_cloud()
        pcd_down = pcd_tam.voxel_down_sample(down_voksel)
        if len(pcd_down.points) > 0:
            if not pcd_down.has_normals():
                pcd_down.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=0.10, max_nn=30)
                )
            else:
                pcd_down.normalize_normals()

        if idx == 0:
            referans_pcd = pcd_down
            s.T_sahne = np.eye(4, dtype=np.float64)
            s.yuzey_noktalari = np.asarray(pcd_tam.points, dtype=np.float64).copy()
            rapor["sahneler"][s.ad] = {
                "referans": True,
                "uygulandi": True,
                "dx": 0.0,
                "dy": 0.0,
                "dz": 0.0,
                "dyaw_deg": 0.0,
                "oteleme_m": 0.0,
                "fitness": 1.0,
                "inlier_rmse": 0.0,
                "matris": s.T_sahne.tolist(),
            }
            print(
                f"[SAHNE HİZALA] Referans sahne: {s.ad} "
                f"({len(pcd_down.points)} nokta)"
            )
            continue

        if not sahne_hizala_aktif or referans_pcd is None or len(pcd_down.points) < 30 or len(referans_pcd.points) < 30:
            s.T_sahne = np.eye(4, dtype=np.float64)
            s.yuzey_noktalari = np.asarray(pcd_tam.points, dtype=np.float64).copy()
            rapor["sahneler"][s.ad] = {
                "referans": False,
                "uygulandi": False,
                "dx": 0.0,
                "dy": 0.0,
                "dz": 0.0,
                "dyaw_deg": 0.0,
                "oteleme_m": 0.0,
                "fitness": 0.0,
                "inlier_rmse": 0.0,
                "matris": s.T_sahne.tolist(),
            }
            continue

        reg1 = o3d.pipelines.registration.registration_icp(
            pcd_down,
            referans_pcd,
            0.10,
            np.eye(4, dtype=np.float64),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
        )
        reg2 = o3d.pipelines.registration.registration_icp(
            pcd_down,
            referans_pcd,
            0.05,
            reg1.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
        )

        T_ham = np.asarray(reg2.transformation, dtype=np.float64)
        T_kisi, dx, dy, dz, dyaw_deg = yaw_ve_oteleme_kisitla(T_ham, dz_sinir=0.05)
        ham_oteleme = float(np.linalg.norm(T_ham[:3, 3]))
        kisi_oteleme = float(np.linalg.norm([dx, dy, dz]))
        oteleme_m = max(ham_oteleme, kisi_oteleme)
        fitness = float(reg2.fitness)
        rmse = float(reg2.inlier_rmse)

        if oteleme_m > 0.15 or abs(dyaw_deg) > 5.0:
            print(
                f"[SAHNE HİZALA] UYARI: {s.ad} düzeltmesi sınırı aştı "
                f"(öteleme={oteleme_m:.4f} m > 0.15 m veya |dyaw|={abs(dyaw_deg):.2f}° > 5°), "
                f"fitness={fitness:.4f}, rmse={rmse:.4f} -> UYGULANMADI"
            )
            uygulandi = False
            s.T_sahne = np.eye(4, dtype=np.float64)
        else:
            print(
                f"[SAHNE HİZALA] {s.ad}: dx={dx:+.4f} m, dy={dy:+.4f} m, dz={dz:+.4f} m, "
                f"dyaw={dyaw_deg:+.2f}°, |d|={kisi_oteleme:.4f} m, "
                f"fitness={fitness:.4f}, rmse={rmse:.4f} -> UYGULANDI"
            )
            uygulandi = True
            s.T_sahne = T_kisi

        for kare in s.kareler:
            kare.T_wc_sahne = s.T_sahne @ kare.T_wc_ilk
            kare.T_wc_son = kare.T_wc_sahne.copy()

        pcd_tam_hizali = o3d.geometry.PointCloud(pcd_tam)
        pcd_tam_hizali.transform(s.T_sahne)
        s.yuzey_noktalari = np.asarray(pcd_tam_hizali.points, dtype=np.float64).copy()

        rapor["sahneler"][s.ad] = {
            "referans": False,
            "uygulandi": bool(uygulandi),
            "dx": round(dx, 5),
            "dy": round(dy, 5),
            "dz": round(dz, 5),
            "dyaw_deg": round(dyaw_deg, 4),
            "oteleme_m": round(kisi_oteleme, 5),
            "ham_oteleme_m": round(ham_oteleme, 5),
            "fitness": round(fitness, 5),
            "inlier_rmse": round(rmse, 5),
            "matris": [[round(float(x), 6) for x in satir] for satir in s.T_sahne],
        }

    return rapor


def kare_ince_hizala(
    rgb: np.ndarray,
    depth: np.ndarray,
    intrinsic: o3d.camera.PinholeCameraIntrinsic,
    T_baz: np.ndarray,
    onbellek_pcd: o3d.geometry.PointCloud,
    onbellek_noktalar: np.ndarray,
) -> tuple[bool, np.ndarray, float, float, float]:
    """Kare nokta bulutunu (<=2.0m, voksel 0.02) önbellek TSDF bulutuna colored ICP ile hizalar."""
    if len(onbellek_noktalar) < 100:
        return False, np.eye(4, dtype=np.float64), 0.0, 0.0, 0.0

    rgbd_kisa = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb),
        o3d.geometry.Image(depth),
        depth_scale=1.0,
        depth_trunc=2.0,
        convert_rgb_to_intensity=False,
    )
    pcd_kare = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_kisa, intrinsic)
    if len(pcd_kare.points) < 100:
        return False, np.eye(4, dtype=np.float64), 0.0, 0.0, 0.0

    pcd_kare.transform(T_baz)
    kaynak_02 = pcd_kare.voxel_down_sample(0.02)
    kaynak_noktalar = np.asarray(kaynak_02.points, dtype=np.float64)
    if len(kaynak_noktalar) < 80:
        return False, np.eye(4, dtype=np.float64), 0.0, 0.0, 0.0

    min_kutu = kaynak_noktalar.min(axis=0) - 0.15
    max_kutu = kaynak_noktalar.max(axis=0) + 0.15
    ilgili_maske = np.all(
        (onbellek_noktalar >= min_kutu) & (onbellek_noktalar <= max_kutu), axis=1
    )
    ilgili_indisler = np.flatnonzero(ilgili_maske).tolist()
    if len(ilgili_indisler) < 100:
        return False, np.eye(4, dtype=np.float64), 0.0, 0.0, 0.0

    hedef_02 = onbellek_pcd.select_by_index(ilgili_indisler)
    kaynak_04 = kaynak_02.voxel_down_sample(0.04)
    hedef_04 = hedef_02.voxel_down_sample(0.04)
    if len(kaynak_04.points) < 50 or len(hedef_04.points) < 50:
        return False, np.eye(4, dtype=np.float64), 0.0, 0.0, 0.0

    try:
        kaynak_04.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30)
        )
        hedef_04.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30)
        )
        reg_04 = o3d.pipelines.registration.registration_colored_icp(
            kaynak_04,
            hedef_04,
            0.04,
            np.eye(4, dtype=np.float64),
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=25
            ),
        )

        kaynak_02.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.04, max_nn=30)
        )
        hedef_02.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.04, max_nn=30)
        )
        reg_02 = o3d.pipelines.registration.registration_colored_icp(
            kaynak_02,
            hedef_02,
            0.02,
            reg_04.transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=15
            ),
        )
    except Exception:
        return False, np.eye(4, dtype=np.float64), 0.0, 0.0, 0.0

    T_aday = np.asarray(reg_02.transformation, dtype=np.float64)
    if not np.all(np.isfinite(T_aday)):
        return False, np.eye(4, dtype=np.float64), 0.0, 0.0, 0.0

    fitness = float(reg_02.fitness)
    merkez = kaynak_noktalar.mean(axis=0)
    merkez_yeni = T_aday[:3, :3] @ merkez + T_aday[:3, 3]
    oteleme_merkez = float(np.linalg.norm(merkez_yeni - merkez))
    kam_konum_yeni = (T_aday @ T_baz)[:3, 3]
    oteleme_kam = float(np.linalg.norm(kam_konum_yeni - T_baz[:3, 3]))
    oteleme_m = max(oteleme_merkez, oteleme_kam)
    aci_deg = rotasyon_acisi_deg(T_aday[:3, :3])

    kabul = (fitness > 0.5) and (oteleme_m < 0.05) and (aci_deg < 2.0)
    if not kabul:
        return False, np.eye(4, dtype=np.float64), fitness, oteleme_m, aci_deg

    # Göreli ofset (T_wc_son = T_baz @ T_ofset_kam) olarak sakla
    T_ofset_kam = np.linalg.inv(T_baz) @ T_aday @ T_baz
    return True, T_ofset_kam, fitness, oteleme_m, aci_deg


def tutarlilik_temizle(
    mesh: o3d.geometry.TriangleMesh,
    sahneler: list[SahneVerisi],
    voxel: float,
    zmax: float,
    max_ornek_kare: int = 200,
) -> dict[str, Any]:
    """Sahne başına TSDF yüzeyleri ve derinlik geçişleriyle yeri değişmiş nesneleri temizler."""
    koseler = np.asarray(mesh.vertices, dtype=np.float64)
    ucgenler = np.asarray(mesh.triangles, dtype=np.int32)
    n_kose = len(koseler)
    n_ucgen = len(ucgenler)

    if n_kose == 0 or n_ucgen == 0 or len(sahneler) < 2:
        return {
            "aktif": True,
            "onceki_ucgen": int(n_ucgen),
            "atilan_ucgen": 0,
            "kalan_ucgen": int(n_ucgen),
            "atilan_ucgen_orani": 0.0,
            "atilan_kose": 0,
        }

    # Adım 2'deki kaba TSDF voksel boyutu 0.04 m; 1.5 voksel yarıçapı
    yuzey_esik = max(1.5 * voxel, 1.5 * 0.04)
    bos_marj = max(0.10, 4.0 * voxel)
    erode_cekirdek = np.ones((3, 3), dtype=np.uint8)

    yuzey_goren_sahne = np.zeros(n_kose, dtype=np.int32)
    bos_bulan_sahne = np.zeros(n_kose, dtype=np.int32)

    for s in sahneler:
        # 1) Bu sahnenin TSDF yüzeyinde (1.5 voksel içinde) karşılığı olan köşeler
        if len(s.yuzey_noktalari) > 0:
            if _cKDTree is not None:
                agac = _cKDTree(s.yuzey_noktalari)
                mesafeler, _ = agac.query(koseler, k=1)
                yuzey_goren_sahne += (mesafeler <= yuzey_esik).astype(np.int32)
            else:
                pcd_s = o3d.geometry.PointCloud()
                pcd_s.points = o3d.utility.Vector3dVector(s.yuzey_noktalari)
                kdtree = o3d.geometry.KDTreeFlann(pcd_s)
                yuzey_maske = np.zeros(n_kose, dtype=bool)
                r2 = yuzey_esik * yuzey_esik
                for v_idx, pt in enumerate(koseler):
                    k_bul, _, d2 = kdtree.search_knn_vector_3d(pt, 1)
                    if k_bul > 0 and d2[0] <= r2:
                        yuzey_maske[v_idx] = True
                yuzey_goren_sahne += yuzey_maske.astype(np.int32)

        # 2) Bu sahnede kamera konisi içinde olup derinlik ölçümü arkaya geçen (boş) köşeler
        if len(s.kareler) == 0:
            continue
        if len(s.kareler) > max_ornek_kare:
            ornek_indisler = np.linspace(0, len(s.kareler) - 1, max_ornek_kare, dtype=int)
            ornek_kareler = [s.kareler[i] for i in ornek_indisler]
        else:
            ornek_kareler = s.kareler

        bos_sayac = np.zeros(n_kose, dtype=np.int32)
        fx, fy, cx, cy = s.fx, s.fy, s.cx, s.cy

        for kare in ornek_kareler:
            depth = np.load(str(kare.depth_yolu)).astype(np.float32)
            depth[(depth > zmax) | (depth < 0.0) | ~np.isfinite(depth)] = 0.0
            if kare.insan_maskesi is not None:
                depth[kare.insan_maskesi] = 0.0
            # Kenar piksellerdeki derinlik sıçramalarını bastırmak için 3x3 minimum
            depth_min = cv2.erode(depth, erode_cekirdek)
            yukseklik, genislik = depth_min.shape

            T_cw = np.linalg.inv(kare.T_wc_son)
            R_cw = T_cw[:3, :3]
            t_cw = T_cw[:3, 3]

            z_cam = koseler @ R_cw[2, :] + t_cw[2]
            aday_indis = np.flatnonzero((z_cam > 0.25) & (z_cam < zmax - bos_marj))
            if len(aday_indis) == 0:
                continue

            kose_aday = koseler[aday_indis]
            z_a = z_cam[aday_indis]
            x_a = kose_aday @ R_cw[0, :] + t_cw[0]
            y_a = kose_aday @ R_cw[1, :] + t_cw[1]

            u = np.rint(fx * (x_a / z_a) + cx).astype(np.int32)
            v = np.rint(fy * (y_a / z_a) + cy).astype(np.int32)

            icerde = (u >= 2) & (u < genislik - 2) & (v >= 2) & (v < yukseklik - 2)
            if not np.any(icerde):
                continue

            alt_indis = aday_indis[icerde]
            u_ic = u[icerde]
            v_ic = v[icerde]
            z_ic = z_a[icerde]

            d_olcum = depth_min[v_ic, u_ic]
            bos_maske = (d_olcum > 0.0) & (d_olcum > z_ic + bos_marj)
            if np.any(bos_maske):
                bos_sayac[alt_indis[bos_maske]] += 1

        # En az 2 örnek karede açıkça içinden geçilmiş köşeleri bu sahnede "boş" say
        bos_bulan_sahne += (bos_sayac >= 2).astype(np.int32)

    at_kose_maske = bos_bulan_sahne > yuzey_goren_sahne
    at_ucgen_maske = np.any(at_kose_maske[ucgenler], axis=1)
    atilan_ucgen = int(np.sum(at_ucgen_maske))
    atilan_kose = int(np.sum(at_kose_maske))
    oran = float(atilan_ucgen / n_ucgen) if n_ucgen > 0 else 0.0

    if atilan_ucgen > 0:
        mesh.remove_triangles_by_mask(at_ucgen_maske)
        mesh.remove_unreferenced_vertices()

    print(
        f"[TUTARLILIK] {atilan_ucgen}/{n_ucgen} üçgen atıldı "
        f"(%{oran * 100.0:.2f}, {atilan_kose} tutarsız köşe)"
    )

    return {
        "aktif": True,
        "onceki_ucgen": int(n_ucgen),
        "atilan_ucgen": int(atilan_ucgen),
        "kalan_ucgen": int(len(mesh.triangles)),
        "atilan_ucgen_orani": round(oran, 5),
        "atilan_kose": int(atilan_kose),
    }


def kose_renklerini_yeniden_hesapla(
    mesh: o3d.geometry.TriangleMesh,
    sahneler: list[SahneVerisi],
    zmax: float,
    en_iyi_k: int = 5,
    max_kare_sahne: int = 250,
) -> dict[str, Any]:
    """Mesh köşe renklerini en yakından ve en dik açıyla gören en iyi K kareden medyanla hesaplar."""
    koseler = np.asarray(mesh.vertices, dtype=np.float64)
    normaller = np.asarray(mesh.vertex_normals, dtype=np.float64)
    n_kose = len(koseler)
    if n_kose == 0:
        return {"aktif": True, "guncellenen_kose": 0, "toplam_kose": 0}

    en_iyi_skor = np.full((n_kose, en_iyi_k), -1.0, dtype=np.float32)
    en_iyi_renk = np.zeros((n_kose, en_iyi_k, 3), dtype=np.float32)

    for s in sahneler:
        if len(s.kareler) == 0:
            continue
        if len(s.kareler) > max_kare_sahne:
            indisler = np.linspace(0, len(s.kareler) - 1, max_kare_sahne, dtype=int)
            ornekler = [s.kareler[i] for i in indisler]
        else:
            ornekler = s.kareler

        fx, fy, cx, cy = s.fx, s.fy, s.cx, s.cy
        for kare in ornekler:
            yuklenen = kare_rgbd_yukle(
                kare.rgb_yolu, kare.depth_yolu, zmax, kare.insan_maskesi
            )
            if yuklenen is None:
                continue
            rgb, depth = yuklenen
            yukseklik, genislik = depth.shape

            T_wc = kare.T_wc_son
            T_cw = np.linalg.inv(T_wc)
            R_cw = T_cw[:3, :3]
            t_cw = T_cw[:3, 3]
            kam_merkez = T_wc[:3, 3]

            z_cam = koseler @ R_cw[2, :] + t_cw[2]
            aday = np.flatnonzero((z_cam > 0.25) & (z_cam < zmax))
            if len(aday) == 0:
                continue

            k_a = koseler[aday]
            z_a = z_cam[aday]
            x_a = k_a @ R_cw[0, :] + t_cw[0]
            y_a = k_a @ R_cw[1, :] + t_cw[1]

            u = np.rint(fx * (x_a / z_a) + cx).astype(np.int32)
            v = np.rint(fy * (y_a / z_a) + cy).astype(np.int32)
            icerde = (u >= 0) & (u < genislik) & (v >= 0) & (v < yukseklik)
            if not np.any(icerde):
                continue

            aday = aday[icerde]
            u = u[icerde]
            v = v[icerde]
            z_a = z_a[icerde]

            d_olcum = depth[v, u]
            gorunur = (d_olcum > 0.0) & (np.abs(d_olcum - z_a) <= 0.06)
            if not np.any(gorunur):
                continue

            aday = aday[gorunur]
            u = u[gorunur]
            v = v[gorunur]
            z_a = z_a[gorunur]

            bakis_vek = kam_merkez - koseler[aday]
            mesafe = np.linalg.norm(bakis_vek, axis=1)
            mesafe = np.maximum(mesafe, 1e-4)
            cos_aci = np.einsum("ij,ij->i", normaller[aday], bakis_vek) / mesafe
            dik_maske = cos_aci > 0.15
            if not np.any(dik_maske):
                continue

            aday = aday[dik_maske]
            u = u[dik_maske]
            v = v[dik_maske]
            skor = (cos_aci[dik_maske] / np.maximum(mesafe[dik_maske], 0.3)).astype(
                np.float32
            )

            mevcut_en_dusuk_ind = np.argmin(en_iyi_skor[aday], axis=1)
            mevcut_en_dusuk = en_iyi_skor[aday, mevcut_en_dusuk_ind]
            guncelle = skor > mevcut_en_dusuk
            if not np.any(guncelle):
                continue

            g_aday = aday[guncelle]
            g_slot = mevcut_en_dusuk_ind[guncelle]
            en_iyi_skor[g_aday, g_slot] = skor[guncelle]
            en_iyi_renk[g_aday, g_slot] = (
                rgb[v[guncelle], u[guncelle]].astype(np.float32) / 255.0
            )

    mevcut_renkler = np.asarray(mesh.vertex_colors, dtype=np.float64).copy()
    gecerli_sayisi = np.sum(en_iyi_skor > 0.0, axis=1)
    guncellenen = 0

    for k in range(1, en_iyi_k + 1):
        indisler = np.flatnonzero(gecerli_sayisi == k)
        if len(indisler) == 0:
            continue
        if k == en_iyi_k:
            mevcut_renkler[indisler] = np.median(en_iyi_renk[indisler], axis=1)
        else:
            for idx in indisler:
                secili = en_iyi_renk[idx, en_iyi_skor[idx] > 0.0]
                mevcut_renkler[idx] = np.median(secili, axis=0)
        guncellenen += len(indisler)

    mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(mevcut_renkler, 0.0, 1.0))
    print(f"[RENK YENİDEN] {guncellenen}/{n_kose} köşe rengi en iyi {en_iyi_k} kareden yenilendi")
    return {
        "aktif": True,
        "guncellenen_kose": int(guncellenen),
        "toplam_kose": int(n_kose),
    }


def argumanlari_coz() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Kamera sahnelerinden TSDF 3B oda mesh üretimi")
    p.add_argument("cikti_dizini", type=Path, help="Çıktı dosyalarının yazılacağı dizin")
    p.add_argument("sahne", type=Path, nargs="+", help="Bir ya da birden çok sahne dizini")
    p.add_argument("--dx", type=float, default=0.09, help="Gövde x öteleme düzeltmesi (m)")
    p.add_argument("--dy", type=float, default=0.01, help="Gövde y öteleme düzeltmesi (m)")
    p.add_argument("--dyaw", type=float, default=-1.0, help="Gövde yaw düzeltmesi (derece)")
    p.add_argument("--her", type=int, default=2, help="Her N. kareyi işle")
    p.add_argument("--max_w", type=float, default=0.25, help="Maksimum gövde açısal hızı (rad/s)")
    p.add_argument("--voxel", type=float, default=0.02, help="TSDF voksel boyutu (m)")
    p.add_argument("--zmax", type=float, default=2.5, help="Maksimum derinlik mesafesi (m)")
    p.add_argument("--zemin_kes", type=float, default=0.02, help="Zemin kesme yüksekliği (m)")
    p.add_argument("--tavan_kes", type=float, default=2.0, help="Tavan kesme yüksekliği (m)")
    p.add_argument("--min_bilesen", type=int, default=500, help="Minimum bileşen üçgen sayısı")
    p.add_argument("--ucgen", type=int, default=300000, help="Hedef üçgen sayısı (decimation)")
    p.add_argument("--renk", type=int, default=32, help="K-means renk kümesi sayısı")

    # Görev I1: Bulanıklık süzgeci, sahneler arası hizalama, kare başına ince hizalama, tutarlılık, renk
    p.add_argument(
        "--bulanik_yuzde",
        type=float,
        default=20.0,
        help="Sahne içi Laplacian varyansı en düşük %%N kareyi at (0=kapalı)",
    )
    p.add_argument(
        "--bulanik",
        dest="bulanik_aktif",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Bulanıklık süzgecini aç/kapat (varsayılan: açık)",
    )
    p.add_argument(
        "--no_bulanik",
        dest="bulanik_aktif",
        action="store_false",
        help=argparse.SUPPRESS,
    )
    p.add_argument(
        "--sahne_hizala",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Sahneler arası kaba TSDF + point-to-plane ICP hizalama (varsayılan: açık)",
    )
    p.add_argument(
        "--no_sahne_hizala",
        dest="sahne_hizala",
        action="store_false",
        help=argparse.SUPPRESS,
    )
    p.add_argument(
        "--ince",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Kare başına colored ICP ince hizalama (varsayılan: açık)",
    )
    p.add_argument(
        "--no_ince",
        dest="ince",
        action="store_false",
        help=argparse.SUPPRESS,
    )
    p.add_argument(
        "--ince_her",
        type=int,
        default=5,
        help="Her N. karede ince hizalama yap (varsayılan: 5)",
    )
    p.add_argument(
        "--tutarlilik",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Sahneler arası tutarsız (yeri değişmiş) yüzeyleri temizle (varsayılan: açık)",
    )
    p.add_argument(
        "--no_tutarlilik",
        dest="tutarlilik",
        action="store_false",
        help=argparse.SUPPRESS,
    )
    p.add_argument(
        "--renk_yeniden",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Mesh köşe renklerini en iyi 5 kareden medyan ile yeniden hesapla (varsayılan: kapalı)",
    )
    p.add_argument(
        "--no_renk_yeniden",
        dest="renk_yeniden",
        action="store_false",
        help=argparse.SUPPRESS,
    )

    # Görev I2: YOLO insan segmentasyon maskesi
    p.add_argument(
        "--insan_maske",
        dest="insan_maske",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="YOLO segmentasyon ile insanları füzyondan çıkar (varsayılan: açık)",
    )
    p.add_argument(
        "--no_insan_maske",
        dest="insan_maske",
        action="store_false",
        help=argparse.SUPPRESS,
    )
    p.add_argument(
        "--yolo_model",
        type=Path,
        default=Path(
            "/home/gunes/limo_robot/robot_ws_ref/koray_agile_ws/src/ultralytics_ros/models/yolov8m-seg.pt"
        ),
        help="YOLO segmentasyon model dosyası yolu",
    )
    p.add_argument(
        "--maske_genislet",
        type=int,
        default=15,
        help="Segmentasyon maskesi genişletme miktarı (piksel, varsayılan: 15)",
    )
    p.add_argument(
        "--yolo_guven",
        type=float,
        default=0.35,
        help="YOLO tespit güven eşiği (varsayılan: 0.35)",
    )
    p.add_argument(
        "--maske_siniflar",
        type=str,
        default="0",
        help="Maskelenecek virgülle ayrılmış COCO sınıf no listesi (varsayılan: '0' = person)",
    )
    return p.parse_args()


def main() -> None:
    args = argumanlari_coz()
    cikti_dizin = args.cikti_dizini.expanduser().resolve()
    cikti_dizin.mkdir(parents=True, exist_ok=True)

    dT = duzeltme_matrisi(args.dx, args.dy, args.dyaw)

    # 1) Sahneleri yükle ve bulanıklık süzgecini uygula
    sahneler: list[SahneVerisi] = []
    for sahne_yolu in args.sahne:
        s_veri = sahne_karelerini_hazirla(
            sahne_yolu=sahne_yolu,
            dT=dT,
            her=args.her,
            max_w=args.max_w,
            bulanik_yuzde=args.bulanik_yuzde,
            bulanik_aktif=args.bulanik_aktif,
        )
        sahneler.append(s_veri)

    bulanik_ozet = {
        "aktif": bool(args.bulanik_aktif and args.bulanik_yuzde > 0.0),
        "bulanik_yuzde": float(args.bulanik_yuzde),
        "toplam_aday_kare": int(sum(s.aday_kare for s in sahneler)),
        "toplam_max_w_atilan": int(sum(s.max_w_atilan for s in sahneler)),
        "toplam_bulanik_atilan": int(sum(s.bulanik_atilan for s in sahneler)),
        "toplam_secilen_kare": int(sum(len(s.kareler) for s in sahneler)),
        "sahneler": {
            s.ad: {
                "ham_kare": int(s.toplam_ham_kare),
                "max_w_atilan": int(s.max_w_atilan),
                "aday_kare": int(s.aday_kare),
                "bulanik_atilan": int(s.bulanik_atilan),
                "secilen_kare": int(len(s.kareler)),
                "esik_laplacian_var": round(float(s.esik_laplacian_var), 4),
            }
            for s in sahneler
        },
    }

    # 2) Sahneler arası hizalama ve sahne başına kaba TSDF
    # Önce füzyona girecek kareler için YOLO insan segmentasyonu çalıştırılır
    insan_maske_aktif = bool(args.insan_maske)
    model = None
    device = "cpu"
    siniflar = sinif_listesi_coz(args.maske_siniflar)

    if insan_maske_aktif:
        if not ULTRALYTICS_MEVCUT or YOLO is None:
            print(
                "[İNSAN MASKE] UYARI: 'ultralytics' kütüphanesi yüklenemedi, "
                "insan maskeleme devre dışı."
            )
            insan_maske_aktif = False
        else:
            model_yolu = Path(args.yolo_model).expanduser().resolve()
            if not model_yolu.is_file():
                print(
                    f"[İNSAN MASKE] UYARI: YOLO model dosyası bulunamadı ({model_yolu}), "
                    f"insan maskeleme devre dışı."
                )
                insan_maske_aktif = False
            else:
                device = 0 if (torch is not None and torch.cuda.is_available()) else "cpu"
                try:
                    model = YOLO(str(model_yolu))
                    print(
                        f"[İNSAN MASKE] YOLO modeli yüklendi: {model_yolu.name} "
                        f"(cihaz={device}, güven={args.yolo_guven}, genişlet={args.maske_genislet}px, sınıflar={siniflar})"
                    )
                except Exception as e:
                    print(
                        f"[İNSAN MASKE] UYARI: YOLO modeli yüklenemedi ({e}), "
                        f"insan maskeleme devre dışı."
                    )
                    insan_maske_aktif = False

    if insan_maske_aktif and model is not None:
        insan_maske_sonuc = insan_maskeleri_uygula(
            sahneler=sahneler,
            model=model,
            device=device,
            guven=args.yolo_guven,
            siniflar=siniflar,
            genislet_px=args.maske_genislet,
            batch_boyutu=16,
        )
        insan_maske_ozet = {
            "aktif": True,
            "model": str(args.yolo_model),
            "guven_esigi": float(args.yolo_guven),
            "genisletme_piksel": int(args.maske_genislet),
            "siniflar": siniflar,
            **insan_maske_sonuc,
        }
    else:
        for s in sahneler:
            for k in s.kareler:
                k.insan_maskesi = None
        toplam_kare_sayisi = sum(len(s.kareler) for s in sahneler)
        insan_maske_ozet = {
            "aktif": False,
            "model": str(args.yolo_model),
            "guven_esigi": float(args.yolo_guven),
            "genisletme_piksel": int(args.maske_genislet),
            "siniflar": siniflar,
            "toplam_kare": int(toplam_kare_sayisi),
            "maskelenen_kare": 0,
            "maskelenen_kare_orani": 0.0,
            "ortalama_maskeli_piksel_orani": 0.0,
            "sahneler": {
                s.ad: {
                    "toplam_kare": int(len(s.kareler)),
                    "maskelenen_kare": 0,
                    "maskelenen_kare_orani": 0.0,
                    "ortalama_maskeli_piksel_orani": 0.0,
                }
                for s in sahneler
            },
        }

    # 3) Sahneler arası hizalama ve sahne başına kaba TSDF
    sahne_hizalama_ozet = sahneleri_hizala_ve_kaba_tsdf(
        sahneler=sahneler,
        zmax=args.zmax,
        sahne_hizala_aktif=args.sahne_hizala,
        tutarlilik_aktif=args.tutarlilik,
    )

    # 4) Ana TSDF entegrasyonu + kare başına ince hizalama (--ince)
    hacim = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel,
        sdf_trunc=4.0 * args.voxel,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    kullanilan_kare = 0
    onbellek_pcd: o3d.geometry.PointCloud | None = None
    onbellek_noktalar = np.empty((0, 3), dtype=np.float64)
    son_onbellek_karesi = -999

    toplam_ince_denenen = 0
    toplam_ince_kabul = 0
    kabul_otelemeler: list[float] = []
    kabul_acilar: list[float] = []
    kabul_fitnesslar: list[float] = []
    ince_sahne_istatistik: dict[str, Any] = {}

    ince_adim = max(1, int(args.ince_her))

    for s in sahneler:
        T_ince_ofset = np.eye(4, dtype=np.float64)
        s_denenen = 0
        s_kabul = 0
        s_otelemeler: list[float] = []
        s_acilar: list[float] = []

        for s_kare_idx, kare in enumerate(s.kareler):
            yuklenen = kare_rgbd_yukle(
                kare.rgb_yolu, kare.depth_yolu, args.zmax, kare.insan_maskesi
            )
            if yuklenen is None:
                continue
            rgb, depth = yuklenen

            if args.ince and kullanilan_kare >= 30 and (s_kare_idx % ince_adim == 0):
                if (
                    onbellek_pcd is None
                    or (kullanilan_kare - son_onbellek_karesi) >= 50
                ):
                    ham_pcd = hacim.extract_point_cloud()
                    if len(ham_pcd.points) > 0:
                        onbellek_pcd = ham_pcd.voxel_down_sample(0.02)
                        onbellek_noktalar = np.asarray(
                            onbellek_pcd.points, dtype=np.float64
                        )
                    else:
                        onbellek_pcd = ham_pcd
                        onbellek_noktalar = np.empty((0, 3), dtype=np.float64)
                    son_onbellek_karesi = kullanilan_kare

                if onbellek_pcd is not None and len(onbellek_noktalar) >= 100:
                    s_denenen += 1
                    toplam_ince_denenen += 1
                    kabul, T_yeni_ofset, fit, d_m, d_deg = kare_ince_hizala(
                        rgb=rgb,
                        depth=depth,
                        intrinsic=s.intrinsic,
                        T_baz=kare.T_wc_sahne,
                        onbellek_pcd=onbellek_pcd,
                        onbellek_noktalar=onbellek_noktalar,
                    )
                    if kabul:
                        # Birikimli değil; sahne baz pozuna göre mutlak ofset olarak yenile
                        T_ince_ofset = T_yeni_ofset
                        s_kabul += 1
                        toplam_ince_kabul += 1
                        s_otelemeler.append(d_m)
                        s_acilar.append(d_deg)
                        kabul_otelemeler.append(d_m)
                        kabul_acilar.append(d_deg)
                        kabul_fitnesslar.append(fit)

            kare.T_wc_son = kare.T_wc_sahne @ T_ince_ofset
            extrinsic = np.linalg.inv(kare.T_wc_son)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(rgb),
                o3d.geometry.Image(depth),
                depth_scale=1.0,
                depth_trunc=args.zmax,
                convert_rgb_to_intensity=False,
            )
            hacim.integrate(rgbd, s.intrinsic, extrinsic)
            kullanilan_kare += 1
            if kullanilan_kare % 100 == 0:
                print(f"[TSDF] {kullanilan_kare} kare eklendi ({s.ad}/{kare.indis:06d})")

        s_oran = float(s_kabul / s_denenen) if s_denenen > 0 else 0.0
        s_ort_d = float(np.mean(s_otelemeler)) if s_otelemeler else 0.0
        s_ort_a = float(np.mean(s_acilar)) if s_acilar else 0.0
        if args.ince:
            print(
                f"[İNCE HİZALA] {s.ad}: kabul={s_kabul}/{s_denenen} (%{s_oran * 100.0:.1f}), "
                f"ort. düzeltme={s_ort_d * 100.0:.2f} cm / {s_ort_a:.2f}°"
            )
        ince_sahne_istatistik[s.ad] = {
            "denenen": int(s_denenen),
            "kabul": int(s_kabul),
            "kabul_orani": round(s_oran, 4),
            "ortalama_oteleme_m": round(s_ort_d, 5),
            "ortalama_aci_deg": round(s_ort_a, 4),
        }

    genel_kabul_orani = (
        float(toplam_ince_kabul / toplam_ince_denenen)
        if toplam_ince_denenen > 0
        else 0.0
    )
    genel_ort_oteleme = float(np.mean(kabul_otelemeler)) if kabul_otelemeler else 0.0
    genel_ort_aci = float(np.mean(kabul_acilar)) if kabul_acilar else 0.0
    genel_ort_fitness = float(np.mean(kabul_fitnesslar)) if kabul_fitnesslar else 0.0
    if args.ince:
        print(
            f"[İNCE HİZALA] Toplam kabul: {toplam_ince_kabul}/{toplam_ince_denenen} "
            f"(%{genel_kabul_orani * 100.0:.1f}), "
            f"ortalama düzeltme: {genel_ort_oteleme * 100.0:.2f} cm, {genel_ort_aci:.2f}°"
        )

    ince_ozet = {
        "aktif": bool(args.ince),
        "ince_her": int(ince_adim),
        "atlanan_ilk_kare": 30,
        "toplam_denenen": int(toplam_ince_denenen),
        "toplam_kabul": int(toplam_ince_kabul),
        "kabul_orani": round(genel_kabul_orani, 4),
        "ortalama_oteleme_m": round(genel_ort_oteleme, 5),
        "ortalama_aci_deg": round(genel_ort_aci, 4),
        "ortalama_fitness": round(genel_ort_fitness, 4),
        "sahneler": ince_sahne_istatistik,
    }

    print(f"[TSDF] Toplam {kullanilan_kare} kare birleştirildi, mesh çıkarılıyor...")
    mesh = hacim.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # Küçük bağlı bileşenleri temizle
    kume_indis, kume_sayilari, _ = mesh.cluster_connected_triangles()
    kume_indis = np.asarray(kume_indis)
    kume_sayilari = np.asarray(kume_sayilari)
    if len(kume_indis) > 0:
        mesh.remove_triangles_by_mask(kume_sayilari[kume_indis] < args.min_bilesen)
        mesh.remove_unreferenced_vertices()

    # Zemin ve tavan üçgenlerini kes (zemin rengi tahminini kesmeden önce al)
    koseler_ham = np.asarray(mesh.vertices)
    ucgenler_ham = np.asarray(mesh.triangles)
    renkler_ham = np.asarray(mesh.vertex_colors)
    z_ucgen = koseler_ham[ucgenler_ham, 2].mean(axis=1)
    zemin_maske = z_ucgen < args.zemin_kes

    if np.any(zemin_maske) and len(renkler_ham) > 0:
        zemin_ucgen_renk = renkler_ham[ucgenler_ham[zemin_maske]].mean(axis=1)
        zemin_rengi = np.median(zemin_ucgen_renk, axis=0).tolist()
    else:
        zemin_rengi = [0.5, 0.5, 0.5]

    kes_maske = zemin_maske | (z_ucgen > args.tavan_kes)
    mesh.remove_triangles_by_mask(kes_maske)
    mesh.remove_unreferenced_vertices()

    # Quadric decimation ve topoloji temizliği
    eski_kose = np.asarray(mesh.vertices).copy()
    eski_renk = np.asarray(mesh.vertex_colors).copy()
    if len(mesh.triangles) > args.ucgen:
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=args.ucgen)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()

    if not mesh.has_vertex_colors() and len(eski_kose) > 0 and len(eski_renk) > 0:
        if _cKDTree is not None:
            agac = _cKDTree(eski_kose)
            _, ind = agac.query(np.asarray(mesh.vertices), k=1)
            mesh.vertex_colors = o3d.utility.Vector3dVector(eski_renk[ind])
        else:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(eski_kose)
            kdtree = o3d.geometry.KDTreeFlann(pcd)
            yeni_renk = np.zeros((len(mesh.vertices), 3), dtype=np.float64)
            for idx, pt in enumerate(mesh.vertices):
                _, ind, _ = kdtree.search_knn_vector_3d(pt, 1)
                yeni_renk[idx] = eski_renk[ind[0]]
            mesh.vertex_colors = o3d.utility.Vector3dVector(yeni_renk)

    # 4) Tutarsızlık temizliği (--tutarlilik)
    if args.tutarlilik:
        tutarlilik_ozet = tutarlilik_temizle(
            mesh=mesh,
            sahneler=sahneler,
            voxel=args.voxel,
            zmax=args.zmax,
            max_ornek_kare=200,
        )
        # Tutarsız üçgenler silindikten sonra kalan küçük kopuk parçaları temizle
        kume_indis2, kume_sayilari2, _ = mesh.cluster_connected_triangles()
        kume_indis2 = np.asarray(kume_indis2)
        kume_sayilari2 = np.asarray(kume_sayilari2)
        if len(kume_indis2) > 0:
            mesh.remove_triangles_by_mask(
                kume_sayilari2[kume_indis2] < max(50, args.min_bilesen // 4)
            )
            mesh.remove_unreferenced_vertices()
    else:
        tutarlilik_ozet = {
            "aktif": False,
            "onceki_ucgen": int(len(mesh.triangles)),
            "atilan_ucgen": 0,
            "kalan_ucgen": int(len(mesh.triangles)),
            "atilan_ucgen_orani": 0.0,
            "atilan_kose": 0,
        }

    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()

    # 5) İsteğe bağlı renk yeniden hesaplama (--renk_yeniden)
    if args.renk_yeniden:
        renk_yeniden_ozet = kose_renklerini_yeniden_hesapla(
            mesh=mesh,
            sahneler=sahneler,
            zmax=args.zmax,
            en_iyi_k=5,
        )
    else:
        renk_yeniden_ozet = {
            "aktif": False,
            "guncellenen_kose": 0,
            "toplam_kose": int(len(mesh.vertices)),
        }

    # PLY kaydet
    ply_yolu = cikti_dizin / "oda_mesh.ply"
    o3d.io.write_triangle_mesh(str(ply_yolu), mesh)

    # Üçgen renklerini hesapla ve k-means ile kümele
    koseler = np.asarray(mesh.vertices, dtype=np.float64)
    normaller = np.asarray(mesh.vertex_normals, dtype=np.float64)
    ucgenler = np.asarray(mesh.triangles, dtype=np.int32)
    kose_renkleri = np.asarray(mesh.vertex_colors, dtype=np.float32)
    ucgen_renkleri = kose_renkleri[ucgenler].mean(axis=1).astype(np.float32)

    k_sayisi = max(1, min(args.renk, len(ucgen_renkleri)))
    kriter = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    _, etiketler, merkezler = cv2.kmeans(
        ucgen_renkleri,
        k_sayisi,
        None,
        kriter,
        3,
        cv2.KMEANS_PP_CENTERS,
    )
    etiketler = etiketler.ravel()
    merkezler = np.clip(merkezler, 0.0, 1.0)

    obj_ve_mtl_kaydet(cikti_dizin, koseler, normaller, ucgenler, etiketler, merkezler)

    # Duvar rengi tahmini: dikey yüzeylerin (|normal z| < 0.3) medyan rengi
    ucgen_normalleri = np.asarray(mesh.triangle_normals)
    dikey_maske = np.abs(ucgen_normalleri[:, 2]) < 0.3
    if np.any(dikey_maske):
        duvar_rengi = np.median(ucgen_renkleri[dikey_maske], axis=0).astype(float).tolist()
    else:
        duvar_rengi = np.median(ucgen_renkleri, axis=0).astype(float).tolist()

    sinirlar = [
        [round(float(x), 4) for x in koseler.min(axis=0)],
        [round(float(x), 4) for x in koseler.max(axis=0)],
    ]
    ozet = {
        "ucgen": int(len(ucgenler)),
        "kose": int(len(koseler)),
        "sinirlar": sinirlar,
        "kare": int(kullanilan_kare),
        "renkler": [[round(float(c), 4) for c in m] for m in merkezler],
        "duvar_rengi_tahmini": [round(float(c), 4) for c in duvar_rengi],
        "zemin_rengi_tahmini": [round(float(c), 4) for c in zemin_rengi],
        "duzeltme": {"dx": float(args.dx), "dy": float(args.dy), "dyaw": float(args.dyaw)},
    }
    (cikti_dizin / "oda_mesh.json").write_text(
        json.dumps(ozet, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    hizalama_raporu = {
        "parametreler": {
            "dx": float(args.dx),
            "dy": float(args.dy),
            "dyaw": float(args.dyaw),
            "her": int(args.her),
            "max_w": float(args.max_w),
            "voxel": float(args.voxel),
            "ucgen_hedef": int(args.ucgen),
            "bulanik_yuzde": float(args.bulanik_yuzde),
            "bulanik_aktif": bool(args.bulanik_aktif),
            "sahne_hizala": bool(args.sahne_hizala),
            "ince": bool(args.ince),
            "ince_her": int(args.ince_her),
            "tutarlilik": bool(args.tutarlilik),
            "renk_yeniden": bool(args.renk_yeniden),
            "insan_maske": bool(args.insan_maske),
            "yolo_model": str(args.yolo_model),
            "maske_genislet": int(args.maske_genislet),
            "yolo_guven": float(args.yolo_guven),
            "maske_siniflar": str(args.maske_siniflar),
        },
        "insan_maske": insan_maske_ozet,
        "bulanik_suzgeci": bulanik_ozet,
        "sahne_hizalama": sahne_hizalama_ozet,
        "ince_hizalama": ince_ozet,
        "tutarlilik_temizligi": tutarlilik_ozet,
        "renk_yeniden": renk_yeniden_ozet,
    }
    with (cikti_dizin / "hizalama.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(hizalama_raporu, f, sort_keys=False, allow_unicode=True)

    print(f"[TSDF] Tamamlandı: {len(ucgenler)} üçgen, {len(koseler)} köşe -> {cikti_dizin}")


if __name__ == "__main__":
    main()
