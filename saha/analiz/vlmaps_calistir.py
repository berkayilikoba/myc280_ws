#!/usr/bin/env python3
"""bag2vlmaps.py sahnesinden gerçek VLMaps haritası (LSeg + CLIP) ve kategori sorguları.

vlmaps sanal ortamında çalışır (~/vlmaps_env, depo ~/vlmaps, LSeg ağırlığı vlmaps/lseg/checkpoints/demo_e200.ckpt):
  ~/vlmaps_env/bin/python vlmaps_calistir.py <sahne> <cikti_dizini> [--her 2] [--ornek 50] [--kategoriler ...]

  1) <cikti>/veri: rgb, depth (sahneye sembolik bağ) + poses.txt = renk kamerası optik pozu (poses_kamera.txt),
     dünya Habitat eksenine çevrilmiş (VLMaps kamera modu y-yukarı varsayar: x_h = x, y_h = z, z_h = -y).
  2) VLMapBuilderCam ile 3B harita (vlmap_cam/vlmaps_cam.h5df), hücre 5 cm.
  3) Kategori skorları (CLIP metin ↔ LSeg piksel özelliği): kuşbakışı renk haritası, etiket haritası ve her kategori
     için kaç hücre; <cikti>/ozet.yaml + PNG'ler.
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

VLMAPS = Path.home() / 'vlmaps'
sys.path.insert(0, str(VLMAPS))

KATEGORI = ['wall', 'floor', 'door', 'chair', 'table', 'cabinet', 'shelf', 'box', 'window', 'trash can', 'robot']
RENK = [(150, 150, 150), (230, 220, 200), (40, 90, 160), (0, 170, 255), (30, 140, 30), (160, 60, 160),
        (200, 120, 60), (60, 200, 200), (255, 200, 120), (90, 90, 220), (0, 0, 255), (0, 0, 0)]


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def hazirla(sahne, veri):
    veri.mkdir(parents=True, exist_ok=True)
    for alt in ('rgb', 'depth'):
        hedef = veri / alt
        if not hedef.exists():
            hedef.symlink_to((sahne / alt).resolve())
    Rh = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)   # ROS (z yukarı) -> Habitat (y yukarı)
    with open(veri / 'poses.txt', 'w') as f:
        for satir in np.loadtxt(sahne / 'poses_kamera.txt'):
            p, R = satir[:3], q2R(*satir[3:])
            f.write(' '.join(f'{v:.6f}' for v in (*(Rh @ p), *R2q(Rh @ R))) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sahne')
    ap.add_argument('cikti')
    ap.add_argument('--her', type=int, default=2, help='VLMaps skip_frame')
    ap.add_argument('--ornek', type=int, default=50, help='depth_sample_rate: karede H*W/ornek nokta')
    ap.add_argument('--kategoriler', nargs='+', default=KATEGORI)
    ap.add_argument('--zmax', type=float, default=3.0,
                    help='haritaya katılan en uzak derinlik (m); VLMaps varsayılanı 10 m uzak gürültüyü haritaya yayıyordu')
    a = ap.parse_args()
    from omegaconf import OmegaConf
    from vlmaps.map.vlmap import VLMap
    from vlmaps.map.vlmap_builder_cam import VLMapBuilderCam
    _bp = VLMapBuilderCam._backproject_depth

    def bp_sinirli(self, depth, calib_mat, rate, min_depth=0.1, max_depth=10):
        return _bp(self, depth, calib_mat, rate, min_depth=min_depth, max_depth=min(max_depth, a.zmax))
    VLMapBuilderCam._backproject_depth = bp_sinirli

    sahne, cikti = Path(a.sahne).resolve(), Path(a.cikti).resolve()
    veri = cikti / 'veri'
    hazirla(sahne, veri)
    sc = yaml.safe_load(open(sahne / 'vlmaps_config.yaml'))
    cfg = OmegaConf.load(VLMAPS / 'config/map_config/vlmaps.yaml')
    cfg.grid_size, cfg.cell_size = 1000, 0.05
    cfg.pose_info.pose_type, cfg.pose_info.rot_type = 'camera_base', 'quat'
    cfg.cam_calib_mat = sc['cam_calib_mat']
    cfg.skip_frame, cfg.depth_sample_rate = a.her, a.ornek

    vm = VLMap(cfg)
    h5 = veri / 'vlmap_cam' / 'vlmaps_cam.h5df'
    if not h5.exists():
        vm.create_map(veri)
    vm = VLMap(cfg, data_dir=veri)
    vm.load_map(veri)
    vm._init_clip()
    skor = vm.init_categories(a.kategoriler)            # (N hücre, K+1): son sütun "other"
    etiket = np.argmax(skor, axis=1)
    gp = vm.grid_pos.astype(int)                         # [satır (x), yükseklik (y_h), sütun (z_h)]
    r0, r1, c0, c1 = gp[:, 0].min(), gp[:, 0].max(), gp[:, 2].min(), gp[:, 2].max()
    H, W = r1 - r0 + 1, c1 - c0 + 1
    rgb2, lab2, yuk = np.zeros((H, W, 3), np.uint8), np.full((H, W), -1), np.full((H, W), -1e9)
    sira = np.argsort(gp[:, 1])                          # alçaktan yükseğe: en üst hücre kalır
    for i in sira:
        r, h, c = gp[i, 0] - r0, gp[i, 1], gp[i, 2] - c0
        rgb2[r, c], lab2[r, c], yuk[r, c] = vm.grid_rgb[i][::-1], etiket[i], h
    renkli = np.zeros((H, W, 3), np.uint8)
    adlar = list(a.kategoriler) + ['other']
    for k in range(len(adlar)):
        renkli[lab2 == k] = RENK[k % len(RENK)]
    olcek = 4
    for ad, img in (('kusbakisi_rgb.png', rgb2), ('kusbakisi_etiket.png', renkli)):
        b = cv2.resize(img, (W * olcek, H * olcek), interpolation=cv2.INTER_NEAREST)
        if ad == 'kusbakisi_etiket.png':
            for k, n in enumerate(adlar):
                cv2.rectangle(b, (5, 5 + 16 * k), (17, 17 + 16 * k), RENK[k % len(RENK)], -1)
                cv2.putText(b, n, (22, 16 + 16 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imwrite(str(cikti / ad), b)
    sayim = {n: int((etiket == k).sum()) for k, n in enumerate(adlar)}
    ozet = {'sahne': str(sahne), 'hucre_sayisi': int(len(gp)), 'hucre_m': 0.05,
            'kapsama_m2_kusbakisi': round(float((lab2 >= 0).sum()) * 0.0025, 2),
            'kategori_hucre': sayim, 'zmax_m': a.zmax, 'skip_frame': a.her, 'depth_sample_rate': a.ornek}
    yaml.safe_dump(ozet, open(cikti / 'ozet.yaml', 'w'), allow_unicode=True, sort_keys=False)
    print(yaml.safe_dump(ozet, allow_unicode=True, sort_keys=False))


if __name__ == '__main__':
    main()
