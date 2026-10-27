#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Görev E ve H2: oda_dogrula.py — Sim odası ile gerçek odanın kamera ve lidar karşılaştırması
(ROS 2 Humble + Gazebo Classic)

Sim çalışırken (robot modeli adı "limo_mycobot", use_sim_time), gerçek sahnedeki
kare pozlarına robotu ışınlayıp sim renk, derinlik ve lidar taramasını alır,
gerçek kare ve 2B doluluk haritasıyla karşılaştırır.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
import yaml

# bag2vlmaps kütüphanesinden derinlik_hizala (kaplamalı / splat izdüşüm)
sys.path.insert(0, "/home/gunes/limo_robot/limo_ws/saha/analiz")
from bag2vlmaps import derinlik_hizala

try:
  from gazebo_msgs.srv import GetEntityState, SetEntityState
except ImportError:
  GetEntityState = None
  SetEntityState = None


# Hedef karşılaştırma çözünürlüğü (gerçek sahne 640x360)
TARGET_W = 640
TARGET_H = 360

# Derinlik optik -> renk optik dönüşümü:
# Renk kamerası derinliğin +y'sinde (0.0148 m) => optik çerçevede x'te -0.0148 m
DEPTH_TO_COLOR_OPTICAL_OFFSET = np.array([-0.0148, 0.0, 0.0], dtype=np.float64)
T_DEPTH_TO_COLOR_OPTICAL = np.eye(4, dtype=np.float64)
T_DEPTH_TO_COLOR_OPTICAL[:3, 3] = DEPTH_TO_COLOR_OPTICAL_OFFSET

# Satır bantları (360 satır yüksekliği için)
ROW_BANDS: List[Tuple[int, int, str]] = [
    (0, 120, "0-120"),
    (120, 200, "120-200"),
    (200, 260, "200-260"),
    (260, 360, "260-360"),
]


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
  """Kuaterniyonu (qx, qy, qz, qw) 3x3 dönüşüm matrisine çevirir."""
  norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
  if norm < 1e-12:
    return np.eye(3, dtype=np.float64)
  qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

  xx, yy, zz = qx * qx, qy * qy, qz * qz
  xy, xz, yz = qx * qy, qx * qz, qy * qz
  wx, wy, wz = qw * qx, qw * qy, qw * qz

  return np.array(
      [
          [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
          [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
          [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
      ],
      dtype=np.float64,
  )


def rot_matrix_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
  """3x3 dönüşüm matrisini (qx, qy, qz, qw) kuaterniyonuna çevirir."""
  trace = float(R[0, 0] + R[1, 1] + R[2, 2])
  if trace > 0.0:
    s = 0.5 / math.sqrt(trace + 1.0)
    qw = 0.25 / s
    qx = (R[2, 1] - R[1, 2]) * s
    qy = (R[0, 2] - R[2, 0]) * s
    qz = (R[1, 0] - R[0, 1]) * s
  elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
    s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
    qw = (R[2, 1] - R[1, 2]) / s
    qx = 0.25 * s
    qy = (R[0, 1] + R[1, 0]) / s
    qz = (R[0, 2] + R[2, 0]) / s
  elif R[1, 1] > R[2, 2]:
    s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
    qw = (R[0, 2] - R[2, 0]) / s
    qx = (R[0, 1] + R[1, 0]) / s
    qy = 0.25 * s
    qz = (R[1, 2] + R[2, 1]) / s
  else:
    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    qw = (R[1, 0] - R[0, 1]) / s
    qx = (R[0, 2] + R[2, 0]) / s
    qy = (R[1, 2] + R[2, 1]) / s
    qz = 0.25 * s

  norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
  if norm > 1e-12:
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
  return float(qx), float(qy), float(qz), float(qw)


def rot_matrix_to_yaw(R: np.ndarray) -> float:
  """3x3 dönüşüm matrisinden yaw açısını (radyan) döndürür."""
  return math.atan2(float(R[1, 0]), float(R[0, 0]))


def parse_pose_line(line: str) -> Optional[np.ndarray]:
  """poses.txt içindeki tek bir satırı 4x4 homojen T_wb matrisine çevirir."""
  cleaned = line.strip()
  if not cleaned or cleaned.startswith("#"):
    return None
  parts = cleaned.replace(",", " ").split()
  try:
    vals = [float(v) for v in parts]
  except ValueError:
    return None

  T = np.eye(4, dtype=np.float64)
  n = len(vals)
  if n == 16:
    T = np.array(vals, dtype=np.float64).reshape(4, 4)
  elif n == 12:
    T[:3, :] = np.array(vals, dtype=np.float64).reshape(3, 4)
  elif n == 7:
    x, y, z, qx, qy, qz, qw = vals
    T[:3, :3] = quat_to_rot_matrix(qx, qy, qz, qw)
    T[:3, 3] = [x, y, z]
  elif n == 8:
    # [timestamp/index, x, y, z, qx, qy, qz, qw]
    _, x, y, z, qx, qy, qz, qw = vals
    T[:3, :3] = quat_to_rot_matrix(qx, qy, qz, qw)
    T[:3, 3] = [x, y, z]
  elif n == 3:
    x, y, yaw = vals
    c, s = math.cos(yaw), math.sin(yaw)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    T[0, 3], T[1, 3] = x, y
  elif n == 4:
    _, x, y, yaw = vals
    c, s = math.cos(yaw), math.sin(yaw)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    T[0, 3], T[1, 3] = x, y
  else:
    raise ValueError(f"poses.txt satır biçimi tanınmadı ({n} değer): {cleaned}")
  return T


def load_poses(sahne_dir: Path) -> List[np.ndarray]:
  """<sahne>/poses.txt dosyasından gövde pozlarını (T_wb) yükler."""
  poses_path = sahne_dir / "poses.txt"
  if not poses_path.is_file():
    raise FileNotFoundError(f"Poz dosyası bulunamadı: {poses_path}")

  poses: List[np.ndarray] = []
  with open(poses_path, "r", encoding="utf-8") as f:
    for line in f:
      T = parse_pose_line(line)
      if T is not None:
        poses.append(T)

  if not poses:
    raise RuntimeError(f"Poz dosyası boş veya geçerli poz içermiyor: {poses_path}")
  return poses


def _extract_cam_calib_mat(data: Any) -> Optional[np.ndarray]:
  """YAML verisi içinde cam_calib_mat anahtarını özyinelemeli arar."""
  if isinstance(data, dict):
    if "cam_calib_mat" in data:
      arr = np.array(data["cam_calib_mat"], dtype=np.float64).reshape(3, 3)
      return arr
    for v in data.values():
      res = _extract_cam_calib_mat(v)
      if res is not None:
        return res
  return None


def load_real_color_K(sahne_dir: Path) -> Optional[np.ndarray]:
  """Gerçek sahnenin 640x360 K matrisini (vlmaps_config cam_calib_mat) yükler."""
  candidates = [
      sahne_dir / "vlmaps_config.yaml",
      sahne_dir / "vlmaps_config.yml",
      sahne_dir / "vlmaps.yaml",
      sahne_dir / "config.yaml",
      sahne_dir / "cam_calib.yaml",
      sahne_dir.parent / "vlmaps_config.yaml",
  ]
  for p in sorted(sahne_dir.glob("*.y*ml")):
    if p not in candidates:
      candidates.append(p)

  for path in candidates:
    if path.is_file():
      try:
        with open(path, "r", encoding="utf-8") as f:
          content = yaml.safe_load(f)
        K = _extract_cam_calib_mat(content)
        if K is not None:
          return K
      except Exception:
        continue
  return None


def find_frame_file(folder: Path, idx: int, extensions: Tuple[str, ...]) -> Path:
  """rgb/<i>.png veya sıfır dolgulu (000000.png vb.) kare dosyasını bulur."""
  name_patterns = [
      f"{idx}",
      f"{idx:06d}",
      f"{idx:05d}",
      f"{idx:04d}",
      f"{idx:03d}",
  ]
  for pat in name_patterns:
    for ext in extensions:
      candidate = folder / f"{pat}{ext}"
      if candidate.is_file():
        return candidate
  raise FileNotFoundError(
      f"Kare {idx} için dosya bulunamadı: {folder}/{idx}{extensions[0]}"
  )


def load_real_frame(sahne_dir: Path, idx: int) -> Tuple[np.ndarray, np.ndarray]:
  """Gerçek sahnedeki rgb/<i>.png (BGR) ve depth/<i>.npy (metre) verisini okur."""
  rgb_path = find_frame_file(sahne_dir / "rgb", idx, (".png", ".jpg", ".jpeg"))
  depth_path = find_frame_file(sahne_dir / "depth", idx, (".npy",))

  real_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
  if real_bgr is None:
    raise RuntimeError(f"Gerçek renk görüntüsü okunamadı: {rgb_path}")
  if real_bgr.shape[1] != TARGET_W or real_bgr.shape[0] != TARGET_H:
    real_bgr = cv2.resize(
        real_bgr, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA
    )

  real_depth = np.load(str(depth_path))
  if real_depth.ndim == 3:
    real_depth = real_depth.squeeze(-1)
  if real_depth.dtype == np.uint16:
    real_depth = real_depth.astype(np.float32) / 1000.0
  else:
    real_depth = real_depth.astype(np.float32)
    valid_vals = real_depth[np.isfinite(real_depth) & (real_depth > 0)]
    if valid_vals.size > 0 and float(np.median(valid_vals)) > 50.0:
      real_depth = real_depth / 1000.0

  if real_depth.shape[1] != TARGET_W or real_depth.shape[0] != TARGET_H:
    real_depth = cv2.resize(
        real_depth, (TARGET_W, TARGET_H), interpolation=cv2.INTER_NEAREST
    )

  return real_bgr, real_depth


def image_msg_to_bgr(msg: Image) -> np.ndarray:
  """sensor_msgs/Image mesajını OpenCV BGR (uint8) dizisine çevirir."""
  h, w, step = msg.height, msg.width, msg.step
  buf = np.frombuffer(msg.data, dtype=np.uint8)
  enc = msg.encoding.lower()

  if enc in ("rgb8", "bgr8"):
    arr = buf.reshape((h, step))[:, : w * 3].reshape((h, w, 3))
    if enc == "rgb8":
      return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return arr.copy()
  if enc in ("rgba8", "bgra8"):
    arr = buf.reshape((h, step))[:, : w * 4].reshape((h, w, 4))
    if enc == "rgba8":
      return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
  if enc in ("mono8", "8uc1"):
    arr = buf.reshape((h, step))[:, :w]
    return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

  raise ValueError(f"Desteklenmeyen renk kodlaması: {msg.encoding}")


def depth_msg_to_uint16_mm(msg: Image) -> np.ndarray:
  """sensor_msgs/Image derinlik mesajını (16UC1 mm veya 32FC1 m) uint16 mm dizisine çevirir."""
  h, w, step = msg.height, msg.width, msg.step
  enc = msg.encoding.upper()
  byteorder = ">" if msg.is_bigendian else "<"

  if enc in ("16UC1", "MONO16"):
    dtype = np.dtype(np.uint16).newbyteorder(byteorder)
    buf = np.frombuffer(msg.data, dtype=dtype)
    row_elems = step // 2
    return buf.reshape((h, row_elems))[:, :w].copy()
  if enc == "32FC1":
    dtype = np.dtype(np.float32).newbyteorder(byteorder)
    buf = np.frombuffer(msg.data, dtype=dtype)
    row_elems = step // 4
    depth_m = buf.reshape((h, row_elems))[:, :w]
    depth_mm = np.clip(
        np.nan_to_num(np.rint(depth_m * 1000.0), nan=0.0, posinf=0.0, neginf=0.0),
        0.0,
        65535.0,
    ).astype(np.uint16)
    return depth_mm

  raise ValueError(f"Desteklenmeyen derinlik kodlaması: {msg.encoding}")


def depth_msg_to_meters(msg: Image) -> np.ndarray:
  """sensor_msgs/Image derinlik mesajını (16UC1 mm veya 32FC1 m) metreye çevirir."""
  h, w, step = msg.height, msg.width, msg.step
  enc = msg.encoding.upper()
  byteorder = ">" if msg.is_bigendian else "<"

  if enc in ("16UC1", "MONO16"):
    dtype = np.dtype(np.uint16).newbyteorder(byteorder)
    buf = np.frombuffer(msg.data, dtype=dtype)
    row_elems = step // 2
    depth_mm = buf.reshape((h, row_elems))[:, :w]
    return depth_mm.astype(np.float32) / 1000.0
  if enc == "32FC1":
    dtype = np.dtype(np.float32).newbyteorder(byteorder)
    buf = np.frombuffer(msg.data, dtype=dtype)
    row_elems = step // 4
    return buf.reshape((h, row_elems))[:, :w].copy()

  raise ValueError(f"Desteklenmeyen derinlik kodlaması: {msg.encoding}")


def align_depth_to_color_zbuffer(
    depth_m: np.ndarray,
    K_depth: np.ndarray,
    K_color_640x360: np.ndarray,
    out_w: int = TARGET_W,
    out_h: int = TARGET_H,
) -> np.ndarray:
  """Sim derinliğini renk kamerasına hizalar ve kaplamalı (splat) izdüşümle aktarır."""
  if depth_m.dtype == np.uint16:
    dmm = depth_m
  else:
    dmm = np.clip(
        np.nan_to_num(np.rint(depth_m * 1000.0), nan=0.0, posinf=0.0, neginf=0.0),
        0.0,
        65535.0,
    ).astype(np.uint16)

  return derinlik_hizala(
      dmm=dmm,
      Kd=K_depth,
      Kc=K_color_640x360,
      T_c_d=T_DEPTH_TO_COLOR_OPTICAL,
      boyut=(out_h, out_w),
      zmin=0.2,
      zmax=8.0,
      doldur=False,
      kapla=True,
  )


def compute_metrics(
    real_bgr: np.ndarray,
    sim_bgr: np.ndarray,
    real_depth: np.ndarray,
    sim_depth: np.ndarray,
) -> Dict[str, Any]:
  """Gerçek ve sim kareleri için derinlik ve renk ölçütlerini hesaplar."""
  real_valid = np.isfinite(real_depth) & (real_depth > 0.001)
  sim_valid = np.isfinite(sim_depth) & (sim_depth > 0.001)
  both_valid = real_valid & sim_valid

  if np.any(both_valid):
    abs_diff = np.abs(sim_depth[both_valid] - real_depth[both_valid])
    median_diff_m = float(np.median(abs_diff))
    lt_10cm_ratio = float(np.mean(abs_diff < 0.10))
  else:
    median_diff_m = 0.0
    lt_10cm_ratio = 0.0

  real_valid_ratio = float(np.mean(real_valid))
  sim_valid_ratio = float(np.mean(sim_valid))
  both_valid_ratio = float(np.mean(both_valid))

  band_metrics: Dict[str, Dict[str, float]] = {}
  for r0, r1, label in ROW_BANDS:
    rv = float(np.mean(real_valid[r0:r1, :]))
    sv = float(np.mean(sim_valid[r0:r1, :]))
    bv = float(np.mean(both_valid[r0:r1, :]))
    band_metrics[label] = {
        "gercek": round(rv, 4),
        "sim": round(sv, 4),
        "ortak": round(bv, 4),
    }

  # Renk 1: CIE L*a*b* ortalama ΔE
  real_lab = cv2.cvtColor(
      real_bgr.astype(np.float32) / 255.0, cv2.COLOR_BGR2LAB
  )
  sim_lab = cv2.cvtColor(sim_bgr.astype(np.float32) / 255.0, cv2.COLOR_BGR2LAB)
  delta_e_map = np.linalg.norm(sim_lab - real_lab, axis=2)
  if np.any(both_valid):
    mean_delta_e = float(np.mean(delta_e_map[both_valid]))
  else:
    mean_delta_e = float(np.mean(delta_e_map))

  # Renk 2: HSV histogram korelasyonu
  real_hsv = cv2.cvtColor(real_bgr, cv2.COLOR_BGR2HSV)
  sim_hsv = cv2.cvtColor(sim_bgr, cv2.COLOR_BGR2HSV)
  hist_real = cv2.calcHist(
      [real_hsv], [0, 1, 2], None, [18, 16, 16], [0, 180, 0, 256, 0, 256]
  )
  hist_sim = cv2.calcHist(
      [sim_hsv], [0, 1, 2], None, [18, 16, 16], [0, 180, 0, 256, 0, 256]
  )
  cv2.normalize(hist_real, hist_real)
  cv2.normalize(hist_sim, hist_sim)
  hsv_correl = float(cv2.compareHist(hist_real, hist_sim, cv2.HISTCMP_CORREL))

  # Renk 3: Canny kenar haritası örtüşmesi (5 px içinde)
  real_gray = cv2.cvtColor(real_bgr, cv2.COLOR_GRAY2BGR if real_bgr.ndim == 2 else cv2.COLOR_BGR2GRAY)
  sim_gray = cv2.cvtColor(sim_bgr, cv2.COLOR_GRAY2BGR if sim_bgr.ndim == 2 else cv2.COLOR_BGR2GRAY)
  edges_real = cv2.Canny(real_gray, 80, 160)
  edges_sim = cv2.Canny(sim_gray, 80, 160)

  sim_edge_mask = edges_sim > 0
  if np.any(sim_edge_mask) and np.any(edges_real > 0):
    inv_real_edges = np.where(edges_real > 0, 0, 1).astype(np.uint8)
    dist_to_real = cv2.distanceTransform(inv_real_edges, cv2.DIST_L2, 5)
    canny_overlap_5px = float(np.mean(dist_to_real[sim_edge_mask] <= 5.0))
  else:
    canny_overlap_5px = 0.0

  return {
      "derinlik": {
          "medyan_fark_m": round(median_diff_m, 4),
          "on_cm_alti_oran": round(lt_10cm_ratio, 4),
          "gecerli_oran_gercek": round(real_valid_ratio, 4),
          "gecerli_oran_sim": round(sim_valid_ratio, 4),
          "gecerli_oran_ortak": round(both_valid_ratio, 4),
          "satir_bantlari_gecerli_oran": band_metrics,
      },
      "renk": {
          "ortalama_delta_e": round(mean_delta_e, 4),
          "hsv_hist_korelasyon": round(hsv_correl, 4),
          "canny_kenar_ortusme_5px": round(canny_overlap_5px, 4),
      },
  }


def colorize_depth_jet(depth_m: np.ndarray, max_depth: float = 5.0) -> np.ndarray:
  """Derinlik görüntüsünü 0-5 m aralığında JET renk haritasına çevirir."""
  valid = np.isfinite(depth_m) & (depth_m > 0.001)
  clipped = np.clip(np.where(valid, depth_m, 0.0), 0.0, max_depth)
  scaled = np.rint((clipped / max_depth) * 255.0).astype(np.uint8)
  jet = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
  jet[~valid] = 0
  return jet


def build_comparison_figure(
    real_bgr: np.ndarray,
    sim_bgr: np.ndarray,
    real_depth: np.ndarray,
    sim_depth: np.ndarray,
) -> np.ndarray:
  """Yan yana kamera karşılaştırma görseli üretir."""
  real_depth_jet = colorize_depth_jet(real_depth, max_depth=5.0)
  sim_depth_jet = colorize_depth_jet(sim_depth, max_depth=5.0)

  top_row = np.hstack([real_bgr, sim_bgr])
  bottom_row = np.hstack([real_depth_jet, sim_depth_jet])
  return np.vstack([top_row, bottom_row])


# ---------- 2B DOLULUK HARİTASI VE LİDAR MODÜLLERİ ----------

class OccupancyMap2D:
  """2B Doluluk haritası ve harita üzerinde ışın izleme (ray casting)."""

  def __init__(
      self,
      image_path: Path,
      resolution: float,
      origin: Tuple[float, float, float],
      negate: int = 0,
      occupied_thresh: float = 0.65,
  ) -> None:
    self.resolution = float(resolution)
    self.origin_x = float(origin[0])
    self.origin_y = float(origin[1])
    self.origin_yaw = float(origin[2])
    self.negate = int(negate)
    self.occupied_thresh = float(occupied_thresh)

    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
      raise FileNotFoundError(f"Harita görüntüsü okunamadı: {image_path}")

    # Doluluk hesabı (ROS map_server standardı)
    p = img.astype(np.float32)
    if self.negate:
      occ = p / 255.0
    else:
      occ = (255.0 - p) / 255.0

    is_occupied = occ > self.occupied_thresh
    # OpenCV'de 0. satır üsttedir; ROS harita orijini sol alttadır.
    # flipud ile satır indeksini (j) artan y yönüyle hizalıyoruz.
    self.occ_grid = np.flipud(is_occupied)
    self.height, self.width = self.occ_grid.shape

  def trace_rays(
      self,
      lidar_x: float,
      lidar_y: float,
      lidar_yaw: float,
      angle_min: float,
      angle_increment: float,
      num_rays: int,
      range_min: float,
      range_max: float,
      step: Optional[float] = None,
  ) -> np.ndarray:
    """
    Her lidar ışını için haritada beklenen menzili hesaplar (vektörize ışın izleme).
    adım = resolution / 2.0.
    Engelle karşılaşmayan veya harita dışına çıkan ışınlar np.nan döner.
    """
    if step is None or step <= 0.0:
      step = self.resolution / 2.0

    s = np.arange(step, range_max + step * 0.5, step, dtype=np.float64)
    if len(s) == 0:
      return np.full((num_rays,), np.nan, dtype=np.float64)

    ray_indices = np.arange(num_rays, dtype=np.float64)
    ray_angles = angle_min + ray_indices * angle_increment
    world_angles = lidar_yaw + ray_angles

    cos_t = np.cos(world_angles)
    sin_t = np.sin(world_angles)

    # Işın koordinatları (adımlar, ışınlar)
    X_w = lidar_x + s[:, None] * cos_t[None, :]
    Y_w = lidar_y + s[:, None] * sin_t[None, :]

    dX = X_w - self.origin_x
    dY = Y_w - self.origin_y

    if abs(self.origin_yaw) > 1e-6:
      cos_o = math.cos(self.origin_yaw)
      sin_o = math.sin(self.origin_yaw)
      X_m = dX * cos_o + dY * sin_o
      Y_m = -dX * sin_o + dY * cos_o
    else:
      X_m = dX
      Y_m = dY

    I = np.floor(X_m / self.resolution).astype(np.int32)
    J = np.floor(Y_m / self.resolution).astype(np.int32)

    valid_cells = (I >= 0) & (I < self.width) & (J >= 0) & (J < self.height)

    hits = np.zeros((len(s), num_rays), dtype=bool)
    hits[valid_cells] = self.occ_grid[J[valid_cells], I[valid_cells]]

    hit_any = np.any(hits, axis=0)
    first_hit_idx = np.argmax(hits, axis=0)

    expected_ranges = np.where(hit_any, s[first_hit_idx], np.nan)
    return expected_ranges


def load_occupancy_map(yaml_path: Path) -> OccupancyMap2D:
  """YAML dosyasından 2B doluluk haritasını okur."""
  yaml_path = yaml_path.expanduser().resolve()
  if not yaml_path.is_file():
    raise FileNotFoundError(f"Harita YAML dosyası bulunamadı: {yaml_path}")
  with open(yaml_path, "r", encoding="utf-8") as f:
    meta = yaml.safe_load(f)

  image_rel = meta.get("image", "")
  image_path = (yaml_path.parent / image_rel).resolve()
  resolution = float(meta["resolution"])
  origin = meta["origin"]
  negate = int(meta.get("negate", 0))
  occupied_thresh = float(meta.get("occupied_thresh", 0.65))

  return OccupancyMap2D(
      image_path=image_path,
      resolution=resolution,
      origin=(float(origin[0]), float(origin[1]), float(origin[2])),
      negate=negate,
      occupied_thresh=occupied_thresh,
  )


def compute_lidar_metrics(
    sim_ranges: np.ndarray,
    map_ranges: np.ndarray,
    range_min: float,
    range_max: float,
) -> Dict[str, float]:
  """
  Kare başına lidar ölçütleri:
  - Yalnız iki tarafın da sonlu ve range_min..range_max içinde olduğu ışınlar
  - lidar_medyan_fark_m (sim - harita, işaretli)
  - lidar_mutlak_medyan_m
  - lidar_5cm_ici_oran
  - lidar_20cm_disi_oran
  - lidar_gecerli_oran (sim taramasında sonlu ışın oranı)
  """
  sim_finite = np.isfinite(sim_ranges)
  gecerli_oran = (
      float(np.mean(sim_finite)) if len(sim_ranges) > 0 else 0.0
  )

  both_valid = (
      sim_finite
      & (sim_ranges >= range_min)
      & (sim_ranges <= range_max)
      & np.isfinite(map_ranges)
      & (map_ranges >= range_min)
      & (map_ranges <= range_max)
  )

  if np.any(both_valid):
    diff = sim_ranges[both_valid] - map_ranges[both_valid]
    abs_diff = np.abs(diff)
    medyan_fark = float(np.median(diff))
    mutlak_medyan = float(np.median(abs_diff))
    oran_5cm = float(np.mean(abs_diff <= 0.05))
    oran_20cm = float(np.mean(abs_diff > 0.20))
  else:
    medyan_fark = 0.0
    mutlak_medyan = 0.0
    oran_5cm = 0.0
    oran_20cm = 0.0

  return {
      "medyan_fark_m": round(medyan_fark, 4),
      "mutlak_medyan_m": round(mutlak_medyan, 4),
      "5cm_ici_oran": round(oran_5cm, 4),
      "20cm_disi_oran": round(oran_20cm, 4),
      "gecerli_oran": round(gecerli_oran, 4),
      "lidar_medyan_fark_m": round(medyan_fark, 4),
      "lidar_mutlak_medyan_m": round(mutlak_medyan, 4),
      "lidar_5cm_ici_oran": round(oran_5cm, 4),
      "lidar_20cm_disi_oran": round(oran_20cm, 4),
      "lidar_gecerli_oran": round(gecerli_oran, 4),
  }


def plot_lidar_comparison(
    angles_rad: np.ndarray,
    sim_ranges: np.ndarray,
    map_ranges: np.ndarray,
    out_path: Path,
    idx: int,
    range_min: float = 0.0,
    range_max: float = 12.0,
) -> None:
  """Açıya göre sim menzili ve harita menzilini çizer ve lidar_<i>.png olarak kaydeder."""
  angles_deg = np.degrees(angles_rad)
  fig, ax = plt.subplots(figsize=(7, 3.5))

  sim_plot = np.where(
      np.isfinite(sim_ranges) & (sim_ranges >= range_min) & (sim_ranges <= range_max),
      sim_ranges,
      np.nan,
  )
  map_plot = np.where(
      np.isfinite(map_ranges) & (map_ranges >= range_min) & (map_ranges <= range_max),
      map_ranges,
      np.nan,
  )

  ax.plot(angles_deg, sim_plot, label="Sim Lidar", color="#1f77b4", linewidth=1.2)
  ax.plot(angles_deg, map_plot, label="Harita Lidar", color="#ff7f0e", linestyle="--", linewidth=1.2)
  ax.set_xlabel("Açı (°)")
  ax.set_ylabel("Menzil (m)")
  ax.set_title(f"Kare {idx} — Lidar Karşılaştırması")
  ax.set_xlim([angles_deg.min(), angles_deg.max()])
  ax.grid(True, linestyle=":", alpha=0.6)
  ax.legend(loc="upper right")
  fig.tight_layout()
  fig.savefig(str(out_path), dpi=100)
  plt.close(fig)


def compute_average_summary(frame_records: List[Dict[str, Any]]) -> Dict[str, Any]:
  """Kare başına ölçütlerin ortalamasını hesaplar."""
  if not frame_records:
    return {}

  def _mean(vals: List[float]) -> float:
    return round(float(np.mean(vals)), 4) if vals else 0.0

  avg_bands: Dict[str, Dict[str, float]] = {}
  for _, _, label in ROW_BANDS:
    avg_bands[label] = {
        "gercek": _mean(
            [
                r["derinlik"]["satir_bantlari_gecerli_oran"][label]["gercek"]
                for r in frame_records
            ]
        ),
        "sim": _mean(
            [
                r["derinlik"]["satir_bantlari_gecerli_oran"][label]["sim"]
                for r in frame_records
            ]
        ),
        "ortak": _mean(
            [
                r["derinlik"]["satir_bantlari_gecerli_oran"][label]["ortak"]
                for r in frame_records
            ]
        ),
    }

  summary: Dict[str, Any] = {
      "kare_sayisi": len(frame_records),
      "derinlik": {
          "medyan_fark_m": _mean(
              [r["derinlik"]["medyan_fark_m"] for r in frame_records]
          ),
          "on_cm_alti_oran": _mean(
              [r["derinlik"]["on_cm_alti_oran"] for r in frame_records]
          ),
          "gecerli_oran_gercek": _mean(
              [r["derinlik"]["gecerli_oran_gercek"] for r in frame_records]
          ),
          "gecerli_oran_sim": _mean(
              [r["derinlik"]["gecerli_oran_sim"] for r in frame_records]
          ),
          "gecerli_oran_ortak": _mean(
              [r["derinlik"]["gecerli_oran_ortak"] for r in frame_records]
          ),
          "satir_bantlari_gecerli_oran": avg_bands,
      },
      "renk": {
          "ortalama_delta_e": _mean(
              [r["renk"]["ortalama_delta_e"] for r in frame_records]
          ),
          "hsv_hist_korelasyon": _mean(
              [r["renk"]["hsv_hist_korelasyon"] for r in frame_records]
          ),
          "canny_kenar_ortusme_5px": _mean(
              [r["renk"]["canny_kenar_ortusme_5px"] for r in frame_records]
          ),
      },
  }

  lidar_records = [
      r["lidar"] for r in frame_records if "lidar" in r and r["lidar"] is not None
  ]
  if lidar_records:
    summary["lidar"] = {
        "medyan_fark_m": _mean([lr.get("medyan_fark_m", 0.0) for lr in lidar_records]),
        "mutlak_medyan_m": _mean([lr.get("mutlak_medyan_m", 0.0) for lr in lidar_records]),
        "5cm_ici_oran": _mean([lr.get("5cm_ici_oran", 0.0) for lr in lidar_records]),
        "20cm_disi_oran": _mean([lr.get("20cm_disi_oran", 0.0) for lr in lidar_records]),
        "gecerli_oran": _mean([lr.get("gecerli_oran", 0.0) for lr in lidar_records]),
        "lidar_medyan_fark_m": _mean([lr.get("lidar_medyan_fark_m", 0.0) for lr in lidar_records]),
        "lidar_mutlak_medyan_m": _mean([lr.get("lidar_mutlak_medyan_m", 0.0) for lr in lidar_records]),
        "lidar_5cm_ici_oran": _mean([lr.get("lidar_5cm_ici_oran", 0.0) for lr in lidar_records]),
        "lidar_20cm_disi_oran": _mean([lr.get("lidar_20cm_disi_oran", 0.0) for lr in lidar_records]),
        "lidar_gecerli_oran": _mean([lr.get("lidar_gecerli_oran", 0.0) for lr in lidar_records]),
    }

  return summary


class TeleportResult(tuple):
  """(color_msg, depth_msg, scan_msg) demeti; 2'li veya 3'lü açılımı dinamik destekler."""

  def __new__(
      cls,
      color_msg: Image,
      depth_msg: Image,
      scan_msg: Optional[LaserScan] = None,
  ):
    return super().__new__(cls, (color_msg, depth_msg, scan_msg))

  @property
  def color(self) -> Image:
    return self[0]

  @property
  def depth(self) -> Image:
    return self[1]

  @property
  def scan(self) -> Optional[LaserScan]:
    return self[2]

  def __iter__(self):
    try:
      frame = sys._getframe(1)
      op = frame.f_code.co_code[frame.f_lasti]
      import dis
      if dis.opname.get(op, "") == "UNPACK_SEQUENCE":
        arg = frame.f_code.co_code[frame.f_lasti + 1]
        if arg == 2:
          return iter((self[0], self[1]))
    except Exception:
      pass
    return super().__iter__()


class OdaDogrulaNode(Node):
  """Gazebo'da robotu ışınlayıp sim kamera görüntülerini ve lidar taramasını toplayan ROS 2 düğümü."""

  def __init__(
      self,
      model_name: str = "limo_mycobot",
      lidar_aktif: bool = True,
  ) -> None:
    super().__init__(
        "oda_dogrula",
        parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)
        ],
    )
    self.model_name = model_name
    self.lidar_aktif = lidar_aktif

    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    self.latest_color_msg: Optional[Image] = None
    self.latest_depth_msg: Optional[Image] = None
    self.latest_scan_msg: Optional[LaserScan] = None
    self.color_info_msg: Optional[CameraInfo] = None
    self.depth_info_msg: Optional[CameraInfo] = None

    self.color_seq: int = 0
    self.depth_seq: int = 0
    self.scan_seq: int = 0

    self.create_subscription(
        Image, "/camera/color/image_raw", self._on_color, sensor_qos
    )
    self.create_subscription(
        Image, "/camera/depth/image_raw", self._on_depth, sensor_qos
    )
    self.create_subscription(
        CameraInfo, "/camera/color/camera_info", self._on_color_info, sensor_qos
    )
    self.create_subscription(
        CameraInfo, "/camera/depth/camera_info", self._on_depth_info, sensor_qos
    )

    if self.lidar_aktif:
      self.create_subscription(
          LaserScan, "/scan", self._on_scan, sensor_qos
      )

    if SetEntityState is None or GetEntityState is None:
      raise RuntimeError(
          "gazebo_msgs paketi bulunamadı (gazebo_msgs/srv/SetEntityState ve "
          "GetEntityState gerekli). Lütfen ROS 2 Humble ortamını kaynaklayın."
      )

    self.set_state_cli = self.create_client(
        SetEntityState, "/gazebo/set_entity_state"
    )
    self.get_state_cli = self.create_client(
        GetEntityState, "/gazebo/get_entity_state"
    )

  def _on_color(self, msg: Image) -> None:
    self.latest_color_msg = msg
    self.color_seq += 1

  def _on_depth(self, msg: Image) -> None:
    self.latest_depth_msg = msg
    self.depth_seq += 1

  def _on_scan(self, msg: LaserScan) -> None:
    self.latest_scan_msg = msg
    self.scan_seq += 1

  def _on_color_info(self, msg: CameraInfo) -> None:
    if self.color_info_msg is None:
      self.color_info_msg = msg

  def _on_depth_info(self, msg: CameraInfo) -> None:
    if self.depth_info_msg is None:
      self.depth_info_msg = msg

  def wait_for_services_and_streams(self, timeout_sec: float = 30.0) -> float:
    """
    Gazebo servislerini ve kamera yayınlarını bekler; modelin doğduğu z
    yüksekliğini (/gazebo/get_entity_state) okur ve döndürür.
    """
    if not self.set_state_cli.wait_for_service(timeout_sec=timeout_sec):
      alt_set = self.create_client(SetEntityState, "/set_entity_state")
      if alt_set.wait_for_service(timeout_sec=2.0):
        self.set_state_cli = alt_set
      else:
        raise RuntimeError(
            "HATA: '/gazebo/set_entity_state' servisi bulunamadı! "
            "Gazebo Classic simülasyonunun ve libgazebo_ros_state eklentisinin "
            "çalıştığından emin olun."
        )

    spawn_z = 0.0
    has_get_srv = self.get_state_cli.wait_for_service(timeout_sec=3.0)
    if not has_get_srv:
      alt_get = self.create_client(GetEntityState, "/get_entity_state")
      if alt_get.wait_for_service(timeout_sec=1.5):
        self.get_state_cli = alt_get
        has_get_srv = True

    if has_get_srv:
      req = GetEntityState.Request()
      req.name = self.model_name
      req.reference_frame = "world"
      fut = self.get_state_cli.call_async(req)
      rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
      resp = fut.result()
      if resp is not None and resp.success:
        spawn_z = float(resp.state.pose.position.z)
      else:
        self.get_logger().warning(
            f"'{self.model_name}' modeli için başlangıç durumu okunamadı; z=0.0 kullanılacak."
        )
    else:
      self.get_logger().warning(
          "'/gazebo/get_entity_state' servisi bulunamadı; z=0.0 kullanılacak."
      )

    # Kamera ve lidar yayınlarını bekle
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec:
      rclpy.spin_once(self, timeout_sec=0.1)
      if (
          self.latest_color_msg is not None
          and self.latest_depth_msg is not None
          and self.depth_info_msg is not None
      ):
        break

    if self.latest_color_msg is None or self.latest_depth_msg is None:
      raise RuntimeError(
          "HATA: '/camera/color/image_raw' veya '/camera/depth/image_raw' "
          "konularından görüntü alınamadı!"
      )

    return spawn_z

  def wait_sim_seconds(self, duration_sec: float = 1.0) -> None:
    """Belirtilen simülasyon süresi kadar bekler."""
    start_clock = self.get_clock().now()
    start_ns = start_clock.nanoseconds
    start_wall = time.monotonic()
    max_wall_wait = max(duration_sec * 8.0, 5.0)

    while True:
      rclpy.spin_once(self, timeout_sec=0.02)
      now_ns = self.get_clock().now().nanoseconds
      if start_ns > 0 and now_ns > start_ns:
        elapsed_sim = (now_ns - start_ns) * 1e-9
        if elapsed_sim >= duration_sec:
          break
      else:
        if time.monotonic() - start_wall >= duration_sec:
          break

      if time.monotonic() - start_wall >= max_wall_wait:
        break

  def teleport_and_capture(
      self,
      T_sim: np.ndarray,
      spawn_z: float,
      wait_sec: float = 1.0,
      capture_lidar: Optional[bool] = None,
  ) -> TeleportResult:
    """
    Robotu T_sim pozuna (z = spawn_z) ışınlar, sim zamanı bekler ve
    ışınlamadan sonra gelen ilk güncel renk, derinlik ve lidar taramasını döndürür.
    """
    if capture_lidar is None:
      capture_lidar = self.lidar_aktif

    qx, qy, qz, qw = rot_matrix_to_quat(T_sim[:3, :3])

    req = SetEntityState.Request()
    req.state.name = self.model_name
    req.state.reference_frame = "world"
    req.state.pose.position.x = float(T_sim[0, 3])
    req.state.pose.position.y = float(T_sim[1, 3])
    req.state.pose.position.z = float(spawn_z)
    req.state.pose.orientation.x = qx
    req.state.pose.orientation.y = qy
    req.state.pose.orientation.z = qz
    req.state.pose.orientation.w = qw

    fut = self.set_state_cli.call_async(req)
    rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
    resp = fut.result()
    if resp is None or not resp.success:
      status_msg = getattr(resp, "status_message", "Yanıt alınamadı")
      raise RuntimeError(
          f"HATA: '{self.model_name}' modeli ışınlanamadı: {status_msg}"
      )

    t_teleport_sim = self.get_clock().now()

    # Belirtilen sim zamanı kadar bekle
    self.wait_sim_seconds(wait_sec)

    # Bekleme sonrasında güncel karelerin geldiğinden emin ol
    seq_c0 = self.color_seq
    seq_d0 = self.depth_seq
    seq_s0 = self.scan_seq
    t_wait = time.monotonic()
    while time.monotonic() - t_wait < 2.0:
      rclpy.spin_once(self, timeout_sec=0.02)
      if self.color_seq > seq_c0 and self.depth_seq > seq_d0:
        break

    if self.latest_color_msg is None or self.latest_depth_msg is None:
      raise RuntimeError("Kamera görüntü mesajı alınamadı.")

    captured_scan: Optional[LaserScan] = None
    if capture_lidar:
      t_teleport_ns = t_teleport_sim.nanoseconds
      # Işınlamadan ve beklemeden sonra gelen ilk yeni taramayı al
      t_wait_scan = time.monotonic()
      while time.monotonic() - t_wait_scan < 2.0:
        if self.latest_scan_msg is not None and self.scan_seq > seq_s0:
          stamp = self.latest_scan_msg.header.stamp
          stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
          if stamp_ns >= t_teleport_ns:
            captured_scan = self.latest_scan_msg
            break
        rclpy.spin_once(self, timeout_sec=0.02)

      if captured_scan is None and self.latest_scan_msg is not None:
        stamp = self.latest_scan_msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns >= t_teleport_ns:
          captured_scan = self.latest_scan_msg

      if captured_scan is None:
        self.get_logger().warning(
            "UYARI: /scan konusu için güncel tarama alınamadı (zaman aşımı)."
        )

    return TeleportResult(self.latest_color_msg, self.latest_depth_msg, captured_scan)

  def get_depth_K(self) -> np.ndarray:
    """/camera/depth/camera_info mesajından 3x3 K matrisini döndürür."""
    if self.depth_info_msg is not None:
      K = np.array(self.depth_info_msg.k, dtype=np.float64).reshape(3, 3)
      if K[0, 0] > 0:
        return K
    return np.array(
        [[424.0, 0.0, 424.0], [0.0, 424.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )

  def get_fallback_color_K_640x360(self) -> np.ndarray:
    """vlmaps_config bulunamazsa /camera/color/camera_info'dan 640x360 K üretir."""
    if self.color_info_msg is not None:
      K = np.array(self.color_info_msg.k, dtype=np.float64).reshape(3, 3)
      w = float(self.color_info_msg.width or 1280)
      h = float(self.color_info_msg.height or 720)
      if K[0, 0] > 0:
        sx = TARGET_W / w
        sy = TARGET_H / h
        return np.array(
            [
                [K[0, 0] * sx, 0.0, K[0, 2] * sx],
                [0.0, K[1, 1] * sy, K[1, 2] * sy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
    return np.array(
        [[462.1, 0.0, 320.0], [0.0, 462.1, 180.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def parse_args(argv: List[str]) -> Tuple[argparse.Namespace, List[str]]:
  parser = argparse.ArgumentParser(
      description=(
          "Sim odası ile gerçek odanın kamera ve lidar karşılaştırması "
          "(ROS 2 Humble + Gazebo Classic)"
      )
  )
  parser.add_argument(
      "sahne",
      type=Path,
      help="Gerçek sahne dizini (poses.txt, rgb/, depth/, vlmaps_config.yaml)",
  )
  parser.add_argument(
      "cikti_dizini",
      type=Path,
      help="Çıktı görsellerinin ve ozet.yaml dosyasının yazılacağı dizin",
  )
  parser.add_argument(
      "--kareler",
      type=str,
      default=None,
      help="Virgülle ayrılmış kare indeksleri (ör. 0,100,200)",
  )
  parser.add_argument(
      "--her",
      type=int,
      default=100,
      help="Kaç karede bir örnekleneceği (varsayılan: 100)",
  )
  parser.add_argument(
      "--dx",
      type=float,
      default=0.09,
      help="İleri (x) poz düzeltmesi (m, varsayılan: 0.09)",
  )
  parser.add_argument(
      "--dy",
      type=float,
      default=0.01,
      help="Yanal (y) poz düzeltmesi (m, varsayılan: 0.01)",
  )
  parser.add_argument(
      "--dyaw",
      type=float,
      default=-1.0,
      help="Yaw açısı düzeltmesi (derece, varsayılan: -1.0)",
  )
  parser.add_argument(
      "--model",
      type=str,
      default="limo_mycobot",
      help="Gazebo model adı (varsayılan: limo_mycobot)",
  )
  parser.add_argument(
      "--harita",
      type=Path,
      default=Path("~/limo_robot/limo_ws/src/limo_bringup/maps/lab_oda.yaml"),
      help="2B doluluk haritası YAML dosyası",
  )
  parser.add_argument(
      "--lidar_x",
      type=float,
      default=0.137,
      help="base_link -> laser x ofseti (m, varsayılan: 0.137)",
  )
  parser.add_argument(
      "--lidar_y",
      type=float,
      default=0.015,
      help="base_link -> laser y ofseti (m, varsayılan: 0.015)",
  )
  parser.add_argument(
      "--lidar_yaw",
      type=float,
      default=-0.1686,
      help="base_link -> laser yaw ofseti (rad, varsayılan: -0.1686)",
  )
  parser.add_argument(
      "--lidar_yok",
      action="store_true",
      default=False,
      help="Lidar karşılaştırmasını tamamen kapatır",
  )
  return parser.parse_known_args(argv)


def select_frame_indices(
    num_poses: int, kareler_arg: Optional[str], her_arg: int
) -> List[int]:
  """İşlenecek kare indekslerini belirler."""
  if kareler_arg:
    indices: List[int] = []
    for part in kareler_arg.split(","):
      part = part.strip()
      if not part:
        continue
      idx = int(part)
      if idx < 0 or idx >= num_poses:
        raise ValueError(
            f"Geçersiz kare indeksi {idx} (toplam poz sayısı: {num_poses})"
        )
      indices.append(idx)
    return indices

  step = max(1, int(her_arg))
  return list(range(0, num_poses, step))


def build_correction_matrix(dx: float, dy: float, dyaw_deg: float) -> np.ndarray:
  """dT (4x4) poz düzeltme matrisini oluşturur."""
  dyaw_rad = math.radians(dyaw_deg)
  c = math.cos(dyaw_rad)
  s = math.sin(dyaw_rad)
  return np.array(
      [
          [c, -s, 0.0, float(dx)],
          [s, c, 0.0, float(dy)],
          [0.0, 0.0, 1.0, 0.0],
          [0.0, 0.0, 0.0, 1.0],
      ],
      dtype=np.float64,
  )


def build_base_to_laser_matrix(lx: float, ly: float, lyaw_rad: float) -> np.ndarray:
  """base_link -> laser (4x4) dönüşüm matrisini oluşturur."""
  c = math.cos(lyaw_rad)
  s = math.sin(lyaw_rad)
  return np.array(
      [
          [c, -s, 0.0, float(lx)],
          [s, c, 0.0, float(ly)],
          [0.0, 0.0, 1.0, 0.0],
          [0.0, 0.0, 0.0, 1.0],
      ],
      dtype=np.float64,
  )


def write_summary_yaml(
    ozet_path: Path,
    params_info: Dict[str, Any],
    frame_records: List[Dict[str, Any]],
) -> None:
  """Her kare sonrasında ozet.yaml dosyasını günceller."""
  summary_data = {
      "parametreler": params_info,
      "ortalama": compute_average_summary(frame_records),
      "kareler": frame_records,
  }
  with open(ozet_path, "w", encoding="utf-8") as f:
    yaml.safe_dump(summary_data, f, allow_unicode=True, sort_keys=False)


def main(argv: Optional[List[str]] = None) -> int:
  args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)

  sahne_dir = args.sahne.expanduser().resolve()
  cikti_dir = args.cikti_dizini.expanduser().resolve()

  if not sahne_dir.is_dir():
    print(f"HATA: Sahne dizini bulunamadı: {sahne_dir}", file=sys.stderr)
    return 1

  try:
    poses = load_poses(sahne_dir)
    frame_indices = select_frame_indices(len(poses), args.kareler, args.her)
  except Exception as exc:
    print(f"HATA: Sahne verileri okunamadı: {exc}", file=sys.stderr)
    return 1

  if not frame_indices:
    print("HATA: İşlenecek kare bulunamadı.", file=sys.stderr)
    return 1

  cikti_dir.mkdir(parents=True, exist_ok=True)
  ozet_path = cikti_dir / "ozet.yaml"

  dT = build_correction_matrix(args.dx, args.dy, args.dyaw)
  T_base_to_laser = build_base_to_laser_matrix(
      args.lidar_x, args.lidar_y, args.lidar_yaw
  )
  real_K_640x360 = load_real_color_K(sahne_dir)

  # 2B Harita yükleme
  occ_map: Optional[OccupancyMap2D] = None
  if not args.lidar_yok:
    try:
      occ_map = load_occupancy_map(args.harita)
    except Exception as exc:
      print(f"UYARI: 2B Harita yüklenemedi: {exc}. Lidar karşılaştırması atlanacak.", file=sys.stderr)

  rclpy.init(args=ros_args)
  node: Optional[OdaDogrulaNode] = None
  try:
    lidar_aktif = (not args.lidar_yok) and (occ_map is not None)
    node = OdaDogrulaNode(model_name=args.model, lidar_aktif=lidar_aktif)
    spawn_z = node.wait_for_services_and_streams(timeout_sec=30.0)

    K_depth = node.get_depth_K()
    K_color_640x360 = (
        real_K_640x360
        if real_K_640x360 is not None
        else node.get_fallback_color_K_640x360()
    )

    params_info: Dict[str, Any] = {
        "sahne": str(sahne_dir),
        "cikti_dizini": str(cikti_dir),
        "model": args.model,
        "dx": float(args.dx),
        "dy": float(args.dy),
        "dyaw": float(args.dyaw),
        "spawn_z": round(float(spawn_z), 4),
        "secilen_kareler": [int(i) for i in frame_indices],
        "lidar_yok": bool(args.lidar_yok),
        "harita": str(args.harita),
        "lidar_x": float(args.lidar_x),
        "lidar_y": float(args.lidar_y),
        "lidar_yaw": float(args.lidar_yaw),
    }

    frame_records: List[Dict[str, Any]] = []

    for idx in frame_indices:
      T_wb = poses[idx]
      T_sim = T_wb @ dT
      T_laser = T_sim @ T_base_to_laser

      real_bgr, real_depth = load_real_frame(sahne_dir, idx)
      capture_res = node.teleport_and_capture(
          T_sim=T_sim, spawn_z=spawn_z, wait_sec=1.0, capture_lidar=lidar_aktif
      )
      color_msg = capture_res.color
      depth_msg = capture_res.depth
      scan_msg = capture_res.scan

      sim_bgr_full = image_msg_to_bgr(color_msg)
      sim_bgr_640x360 = cv2.resize(
          sim_bgr_full, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA
      )

      sim_depth_mm = depth_msg_to_uint16_mm(depth_msg)
      sim_depth_aligned = derinlik_hizala(
          dmm=sim_depth_mm,
          Kd=K_depth,
          Kc=K_color_640x360,
          T_c_d=T_DEPTH_TO_COLOR_OPTICAL,
          boyut=(TARGET_H, TARGET_W),
          zmin=0.2,
          zmax=8.0,
          doldur=False,
          kapla=True,
      )

      metrics = compute_metrics(
          real_bgr=real_bgr,
          sim_bgr=sim_bgr_640x360,
          real_depth=real_depth,
          sim_depth=sim_depth_aligned,
      )

      comp_img = build_comparison_figure(
          real_bgr=real_bgr,
          sim_bgr=sim_bgr_640x360,
          real_depth=real_depth,
          sim_depth=sim_depth_aligned,
      )
      out_img_path = cikti_dir / f"kare_{idx}.png"
      cv2.imwrite(str(out_img_path), comp_img)

      # Lidar ölçütleri ve görseli
      lidar_metrics: Optional[Dict[str, float]] = None
      if lidar_aktif and scan_msg is not None and occ_map is not None:
        lidar_x = float(T_laser[0, 3])
        lidar_y = float(T_laser[1, 3])
        lidar_yaw = rot_matrix_to_yaw(T_laser[:3, :3])

        num_rays = len(scan_msg.ranges)
        sim_ranges = np.array(scan_msg.ranges, dtype=np.float64)
        map_ranges = occ_map.trace_rays(
            lidar_x=lidar_x,
            lidar_y=lidar_y,
            lidar_yaw=lidar_yaw,
            angle_min=scan_msg.angle_min,
            angle_increment=scan_msg.angle_increment,
            num_rays=num_rays,
            range_min=scan_msg.range_min,
            range_max=scan_msg.range_max,
        )

        lidar_metrics = compute_lidar_metrics(
            sim_ranges=sim_ranges,
            map_ranges=map_ranges,
            range_min=scan_msg.range_min,
            range_max=scan_msg.range_max,
        )

        lidar_img_path = cikti_dir / f"lidar_{idx}.png"
        ray_angles = scan_msg.angle_min + np.arange(num_rays) * scan_msg.angle_increment
        plot_lidar_comparison(
            angles_rad=ray_angles,
            sim_ranges=sim_ranges,
            map_ranges=map_ranges,
            out_path=lidar_img_path,
            idx=idx,
            range_min=scan_msg.range_min,
            range_max=scan_msg.range_max,
        )

      record: Dict[str, Any] = {
          "kare": int(idx),
          "poz_gercek": {
              "x": round(float(T_wb[0, 3]), 4),
              "y": round(float(T_wb[1, 3]), 4),
              "yaw_deg": round(
                  math.degrees(rot_matrix_to_yaw(T_wb[:3, :3])), 2
              ),
          },
          "poz_sim": {
              "x": round(float(T_sim[0, 3]), 4),
              "y": round(float(T_sim[1, 3]), 4),
              "z": round(float(spawn_z), 4),
              "yaw_deg": round(
                  math.degrees(rot_matrix_to_yaw(T_sim[:3, :3])), 2
              ),
          },
          "derinlik": metrics["derinlik"],
          "renk": metrics["renk"],
          "gorsel": out_img_path.name,
      }
      if lidar_metrics is not None:
        record["lidar"] = lidar_metrics
        record["lidar_gorsel"] = f"lidar_{idx}.png"

      frame_records.append(record)

      # Her kare sonrasında ozet.yaml'a satır/kayıt yaz
      write_summary_yaml(ozet_path, params_info, frame_records)

      log_msg = (
          f"Kare {idx} tamamlandı: medyan_fark={metrics['derinlik']['medyan_fark_m']:.3f}m, "
          f"<10cm={metrics['derinlik']['on_cm_alti_oran']:.2%}, "
          f"dE={metrics['renk']['ortalama_delta_e']:.2f}, "
          f"HSV={metrics['renk']['hsv_hist_korelasyon']:.3f}, "
          f"Canny5px={metrics['renk']['canny_kenar_ortusme_5px']:.2%}"
      )
      if lidar_metrics is not None:
        log_msg += (
            f" | lidar: medyan_fark={lidar_metrics['medyan_fark_m']:.3f}m, "
            f"mutlak={lidar_metrics['mutlak_medyan_m']:.3f}m, "
            f"<5cm={lidar_metrics['5cm_ici_oran']:.2%}, "
            f">20cm={lidar_metrics['20cm_disi_oran']:.2%}, "
            f"gecerli={lidar_metrics['gecerli_oran']:.2%}"
        )
      node.get_logger().info(log_msg)

  except Exception as exc:
    print(f"{exc}", file=sys.stderr)
    return 1
  finally:
    if node is not None:
      node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()

  return 0


if __name__ == "__main__":
  sys.exit(main())
