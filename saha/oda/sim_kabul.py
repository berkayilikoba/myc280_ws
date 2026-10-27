#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_kabul.py - LIMO Simülasyon Kabul ve Doğrulama Aracı (Görev N1)

Simülasyonun (Gazebo Classic + LIMO + Realsense D435) gerçek robotla aynı
arayüzü, sensör hızlarını, içerik formatlarını ve TF çerçevelerini verdiğini
tek komutla doğrular. Simülasyon çalışıyorken (sim_ac.sh) çalıştırılır.

Kullanım:
    python3 sim_kabul.py [--sure 15] [--cikti rapor.yaml] [--gorsel kare.png]
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.utilities import remove_ros_args
import yaml

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from tf2_ros import Buffer, TransformListener
import tf2_ros


class TopicTracker:
    """Belirli bir ROS 2 konusunun hız ve mesaj istatistiklerini tutar."""

    def __init__(self, name: str, msg_type: str, min_hz: float, max_hz: float):
        self.name = name
        self.msg_type = msg_type
        self.min_hz = min_hz
        self.max_hz = max_hz
        self.count = 0
        self.first_stamp = None
        self.last_stamp = None
        self.first_recv_sim = None
        self.last_recv_sim = None

    def record(self, stamp_sec: float, recv_sim_sec: float):
        self.count += 1
        if self.first_stamp is None:
            self.first_stamp = stamp_sec
            self.first_recv_sim = recv_sim_sec
        self.last_stamp = stamp_sec
        self.last_recv_sim = recv_sim_sec

    def compute_hz(self, elapsed_sim_time: float) -> float:
        if self.count < 2:
            return 0.0

        # Mesaj damgaları arasındaki süre
        if self.first_stamp is not None and self.last_stamp is not None:
            dt_stamp = self.last_stamp - self.first_stamp
            if dt_stamp > 0.0 and dt_stamp >= 0.5 * elapsed_sim_time:
                return (self.count - 1) / dt_stamp

        # Alım simülasyon zamanı arasındaki süre
        if self.first_recv_sim is not None and self.last_recv_sim is not None:
            dt_recv = self.last_recv_sim - self.first_recv_sim
            if dt_recv > 0.0 and dt_recv >= 0.5 * elapsed_sim_time:
                return (self.count - 1) / dt_recv

        # Genel test süresi üzerinden ortalama
        if elapsed_sim_time > 0.0:
            return self.count / elapsed_sim_time

        return 0.0


class SimKabulNode(Node):
    """Simülasyon kabul kontrollerini yürüten ROS 2 düğümü."""

    def __init__(self, sure: float):
        super().__init__(
            'sim_kabul',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ],
        )
        self.sure = sure
        self.start_sim_time = None
        self.elapsed_sim_time = 0.0
        self.latest_clock_sec = None

        # Zaman farkı listesi (|header.stamp - /clock|)
        self.clock_diffs = []

        # Konu Hız Takipçileri
        self.trackers = {
            '/scan': TopicTracker('/scan', 'LaserScan', 5.0, 7.5),  # gerçek YDLIDAR T-mini ~6 Hz (docs/sensor_modeli.md)
            '/imu': TopicTracker('/imu', 'Imu', 70.0, 100.0),
            '/odom': TopicTracker('/odom', 'Odometry', 40.0, 60.0),
            '/camera/color/image_raw': TopicTracker(
                '/camera/color/image_raw', 'Image', 13.0, 16.0
            ),
            '/camera/color/camera_info': TopicTracker(
                '/camera/color/camera_info', 'CameraInfo', 13.0, 16.0
            ),
            '/camera/depth/image_raw': TopicTracker(
                '/camera/depth/image_raw', 'Image', 11.0, 16.0
            ),
            '/camera/depth/camera_info': TopicTracker(
                '/camera/depth/camera_info', 'CameraInfo', 11.0, 16.0
            ),
        }

        # İçerik Ölçümleri
        self.last_color_meta = None
        self.last_color_img = None
        self.color_fx = None

        self.last_depth_meta = None
        self.last_depth_img = None
        self.depth_fx = None
        self.depth_valid_ratios = []

        self.scan_finite_ratios = []
        self.imu_az_values = []

        # TF2 Buffer ve Listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /clock Aboneliği
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_clock = self.create_subscription(
            Clock, '/clock', self._cb_clock, clock_qos
        )

        # Güvenilir Odom Aboneliği
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._cb_odom, odom_qos
        )

        # sensor_data QoS Abonelikleri
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self._cb_scan, qos_profile_sensor_data
        )
        self.sub_imu = self.create_subscription(
            Imu, '/imu', self._cb_imu, qos_profile_sensor_data
        )
        self.sub_color = self.create_subscription(
            Image, '/camera/color/image_raw', self._cb_color_img, qos_profile_sensor_data
        )
        self.sub_color_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._cb_color_info, qos_profile_sensor_data
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/depth/image_raw', self._cb_depth_img, qos_profile_sensor_data
        )
        self.sub_depth_info = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self._cb_depth_info, qos_profile_sensor_data
        )

    def get_sim_time(self) -> float:
        if self.latest_clock_sec is not None:
            return self.latest_clock_sec
        now_ns = self.get_clock().now().nanoseconds
        return now_ns * 1e-9

    def _record_stamp_diff(self, header):
        stamp_sec = header.stamp.sec + header.stamp.nanosec * 1e-9
        curr_sim = self.get_sim_time()

        # Isınma periyodundan sonra (sim başlangıcından 0.2s sonra) kontrol et
        if self.start_sim_time is not None and curr_sim > 0.0 and stamp_sec > 0.0:
            if (curr_sim - self.start_sim_time) >= 0.2:
                diff = abs(curr_sim - stamp_sec)
                self.clock_diffs.append(diff)

        return stamp_sec, curr_sim

    def _cb_clock(self, msg: Clock):
        self.latest_clock_sec = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _cb_scan(self, msg: LaserScan):
        stamp_sec, sim_sec = self._record_stamp_diff(msg.header)
        self.trackers['/scan'].record(stamp_sec, sim_sec)

        # Sonlu ölçüm oranı: np.isfinite
        if len(msg.ranges) > 0:
            arr = np.array(msg.ranges, dtype=np.float32)
            finite_count = int(np.count_nonzero(np.isfinite(arr)))
            ratio = finite_count / float(len(arr))
            self.scan_finite_ratios.append(ratio)

    def _cb_imu(self, msg: Imu):
        stamp_sec, sim_sec = self._record_stamp_diff(msg.header)
        self.trackers['/imu'].record(stamp_sec, sim_sec)
        self.imu_az_values.append(msg.linear_acceleration.z)

    def _cb_odom(self, msg: Odometry):
        stamp_sec, sim_sec = self._record_stamp_diff(msg.header)
        self.trackers['/odom'].record(stamp_sec, sim_sec)

    def _cb_color_img(self, msg: Image):
        stamp_sec, sim_sec = self._record_stamp_diff(msg.header)
        self.trackers['/camera/color/image_raw'].record(stamp_sec, sim_sec)

        self.last_color_meta = {
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
        }

        # Son renk karesini görsel kayıt için BGR formatında sakla
        try:
            if msg.encoding == 'bgr8':
                self.last_color_img = np.frombuffer(
                    msg.data, dtype=np.uint8
                ).reshape((msg.height, msg.width, 3)).copy()
            elif msg.encoding == 'rgb8':
                rgb = np.frombuffer(
                    msg.data, dtype=np.uint8
                ).reshape((msg.height, msg.width, 3))
                self.last_color_img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgra8':
                self.last_color_img = cv2.cvtColor(
                    np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4)),
                    cv2.COLOR_BGRA2BGR,
                )
        except Exception:
            pass

    def _cb_color_info(self, msg: CameraInfo):
        stamp_sec, sim_sec = self._record_stamp_diff(msg.header)
        self.trackers['/camera/color/camera_info'].record(stamp_sec, sim_sec)
        if len(msg.k) >= 1:
            self.color_fx = float(msg.k[0])

    def _cb_depth_img(self, msg: Image):
        stamp_sec, sim_sec = self._record_stamp_diff(msg.header)
        self.trackers['/camera/depth/image_raw'].record(stamp_sec, sim_sec)

        self.last_depth_meta = {
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
        }

        # Geçerli piksel (>0) oranı hesapla
        try:
            if msg.encoding == '16UC1':
                depth_arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
                self.last_depth_img = depth_arr.copy()
                valid_count = int(np.count_nonzero(depth_arr > 0))
                ratio = valid_count / float(depth_arr.size)
                self.depth_valid_ratios.append(ratio)
        except Exception:
            pass

    def _cb_depth_info(self, msg: CameraInfo):
        stamp_sec, sim_sec = self._record_stamp_diff(msg.header)
        self.trackers['/camera/depth/camera_info'].record(stamp_sec, sim_sec)
        if len(msg.k) >= 1:
            self.depth_fx = float(msg.k[0])


def evaluate_tf(tf_buffer: Buffer) -> list:
    """TF ağacı dönüşümlerini ve kalibrasyon ötelemelerini kontrol eder."""
    tf_results = []

    def lookup(target: str, source: str, timeout_sec: float = 2.0):
        try:
            t = tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(), timeout=Duration(seconds=timeout_sec)
            )
            return True, t, ""
        except Exception as e:
            return False, None, str(e)

    # 1. odom -> base_footprint
    ok_odom_bf, _, _ = lookup('odom', 'base_footprint', 2.0)
    tf_results.append({
        'kategori': 'TF',
        'kontrol': 'odom -> base_footprint',
        'olculen': 'Mevcut' if ok_odom_bf else 'Bulunamadı',
        'beklenen': 'Dönüşüm mevcut',
        'durum': 'GEÇTİ' if ok_odom_bf else 'KALDI',
        'detay': {},
    })

    # 2. base_footprint -> base_link (veya doğrudan odom -> base_link)
    ok_bf_bl, _, _ = lookup('base_footprint', 'base_link', 2.0)
    if ok_bf_bl:
        tf_results.append({
            'kategori': 'TF',
            'kontrol': 'base_footprint -> base_link',
            'olculen': 'Mevcut',
            'beklenen': 'Dönüşüm mevcut',
            'durum': 'GEÇTİ',
            'detay': {},
        })
    else:
        # Doğrudan odom -> base_link dene
        ok_odom_bl, _, _ = lookup('odom', 'base_link', 1.0)
        tf_results.append({
            'kategori': 'TF',
            'kontrol': 'base_footprint -> base_link (ya da doğrudan)',
            'olculen': 'Mevcut (doğrudan odom->base_link)' if ok_odom_bl else 'Bulunamadı',
            'beklenen': 'Dönüşüm mevcut',
            'durum': 'GEÇTİ' if ok_odom_bl else 'KALDI',
            'detay': {},
        })

    # 3. base_link -> laser (veya laser_link; ikisini de dene, bulunanı yaz)
    found_laser = None
    ok_laser, _, _ = lookup('base_link', 'laser', 1.0)
    if ok_laser:
        found_laser = 'laser'
    else:
        ok_laser_link, _, _ = lookup('base_link', 'laser_link', 1.0)
        if ok_laser_link:
            found_laser = 'laser_link'

    tf_results.append({
        'kategori': 'TF',
        'kontrol': 'base_link -> laser / laser_link',
        'olculen': f'Bulundu: {found_laser}' if found_laser else 'Bulunamadı',
        'beklenen': 'laser ya da laser_link',
        'durum': 'GEÇTİ' if found_laser else 'KALDI',
        'detay': {'bulunan_cerceve': found_laser},
    })

    # 4. base_link -> camera_link (bağlantı ve öteleme kontrolü)
    ok_cam, t_cam, _ = lookup('base_link', 'camera_link', 2.0)
    tf_results.append({
        'kategori': 'TF',
        'kontrol': 'base_link -> camera_link',
        'olculen': 'Mevcut' if ok_cam else 'Bulunamadı',
        'beklenen': 'Dönüşüm mevcut',
        'durum': 'GEÇTİ' if ok_cam else 'KALDI',
        'detay': {},
    })

    # base_link->camera_link ötelemesi (0.084, 0.0175, 0.028) ±1 cm
    if ok_cam and t_cam is not None:
        trans = t_cam.transform.translation
        tx, ty, tz = trans.x, trans.y, trans.z
        dx = abs(tx - 0.084)
        dy = abs(ty - 0.0175)
        dz = abs(tz - 0.028)
        # 1 cm = 0.01 m tolerans
        offset_ok = (dx <= 0.01) and (dy <= 0.01) and (dz <= 0.01)
        olculen_str = f"({tx:.4f}, {ty:.4f}, {tz:.4f}) m"
        tf_results.append({
            'kategori': 'TF',
            'kontrol': 'base_link->camera_link öteleme',
            'olculen': olculen_str,
            'beklenen': '(0.084, 0.018, 0.028) ±1cm',
            'durum': 'GEÇTİ' if offset_ok else 'KALDI',
            'detay': {'xyz': [tx, ty, tz], 'fark': [dx, dy, dz]},
        })
    else:
        tf_results.append({
            'kategori': 'TF',
            'kontrol': 'base_link->camera_link öteleme',
            'olculen': 'Dönüşüm yok',
            'beklenen': '(0.084, 0.018, 0.028) ±1cm',
            'durum': 'KALDI',
            'detay': {},
        })

    # 5. camera_link -> camera_color_optical_frame
    ok_c_opt, _, _ = lookup('camera_link', 'camera_color_optical_frame', 2.0)
    tf_results.append({
        'kategori': 'TF',
        'kontrol': 'camera_link -> color_optical_frame',
        'olculen': 'Mevcut' if ok_c_opt else 'Bulunamadı',
        'beklenen': 'Dönüşüm mevcut',
        'durum': 'GEÇTİ' if ok_c_opt else 'KALDI',
        'detay': {},
    })

    # 6. camera_link -> camera_depth_optical_frame
    ok_d_opt, _, _ = lookup('camera_link', 'camera_depth_optical_frame', 2.0)
    tf_results.append({
        'kategori': 'TF',
        'kontrol': 'camera_link -> depth_optical_frame',
        'olculen': 'Mevcut' if ok_d_opt else 'Bulunamadı',
        'beklenen': 'Dönüşüm mevcut',
        'durum': 'GEÇTİ' if ok_d_opt else 'KALDI',
        'detay': {},
    })

    # 7. base_link -> imu_link (bulunamazsa UYARI, KALDI değil)
    ok_imu, _, _ = lookup('base_link', 'imu_link', 2.0)
    tf_results.append({
        'kategori': 'TF',
        'kontrol': 'base_link -> imu_link',
        'olculen': 'Mevcut' if ok_imu else 'Bulunamadı (UYARI)',
        'beklenen': 'Dönüşüm (isteğe bağlı)',
        'durum': 'GEÇTİ' if ok_imu else 'UYARI',
        'detay': {},
    })

    return tf_results


def run_checks(node: SimKabulNode) -> tuple:
    """Tüm kontrolleri değerlendirir ve rapor nesnelerini üretir."""
    checks = []
    elapsed = max(node.elapsed_sim_time, 0.1)

    # 1. Konu Hızları
    for topic_name, tracker in node.trackers.items():
        hz = tracker.compute_hz(elapsed)
        hz_str = f"{hz:.1f} Hz ({tracker.count} msg)"
        beklenen_str = f"{tracker.min_hz:.1f} - {tracker.max_hz:.1f} Hz"
        passed = (tracker.min_hz <= hz <= tracker.max_hz) and (tracker.count > 0)
        checks.append({
            'kategori': 'Hız',
            'kontrol': topic_name,
            'olculen': hz_str,
            'beklenen': beklenen_str,
            'durum': 'GEÇTİ' if passed else 'KALDI',
            'detay': {
                'hz': round(hz, 2),
                'mesaj_sayisi': tracker.count,
                'min_hz': tracker.min_hz,
                'max_hz': tracker.max_hz,
            },
        })

    # 2. İçerik Kontrolleri
    # 2.1 Renk görüntüsü 1280x720 rgb8/bgr8
    if node.last_color_meta:
        w = node.last_color_meta['width']
        h = node.last_color_meta['height']
        enc = node.last_color_meta['encoding']
        color_ok = (w == 1280 and h == 720 and enc in ('rgb8', 'bgr8'))
        olculen_str = f"{w}x{h} {enc}"
    else:
        color_ok = False
        olculen_str = "Alınamadı"

    checks.append({
        'kategori': 'İçerik',
        'kontrol': 'Renk çözünürlük / format',
        'olculen': olculen_str,
        'beklenen': '1280x720 rgb8/bgr8',
        'durum': 'GEÇTİ' if color_ok else 'KALDI',
        'detay': node.last_color_meta or {},
    })

    # 2.2 Derinlik 848x480 16UC1
    if node.last_depth_meta:
        dw = node.last_depth_meta['width']
        dh = node.last_depth_meta['height']
        denc = node.last_depth_meta['encoding']
        depth_fmt_ok = (dw == 848 and dh == 480 and denc == '16UC1')
        olculen_str = f"{dw}x{dh} {denc}"
    else:
        depth_fmt_ok = False
        olculen_str = "Alınamadı"

    checks.append({
        'kategori': 'İçerik',
        'kontrol': 'Derinlik çözünürlük / format',
        'olculen': olculen_str,
        'beklenen': '848x480 16UC1',
        'durum': 'GEÇTİ' if depth_fmt_ok else 'KALDI',
        'detay': node.last_depth_meta or {},
    })

    # 2.3 Derinlik geçerli (>0) piksel oranı 0.3–0.95
    if node.depth_valid_ratios:
        mean_depth_ratio = float(np.mean(node.depth_valid_ratios))
        depth_ratio_ok = (0.30 <= mean_depth_ratio <= 0.95)
        olculen_str = f"%{mean_depth_ratio * 100:.1f} ({mean_depth_ratio:.3f})"
    else:
        mean_depth_ratio = 0.0
        depth_ratio_ok = False
        olculen_str = "Alınamadı"

    checks.append({
        'kategori': 'İçerik',
        'kontrol': 'Derinlik geçerli piksel oranı',
        'olculen': olculen_str,
        'beklenen': '0.30 - 0.95',
        'durum': 'GEÇTİ' if depth_ratio_ok else 'KALDI',
        'detay': {'ortalama_oran': round(mean_depth_ratio, 4)},
    })

    # 2.4 /scan sonlu ölçüm oranı 0.85–0.995 (gerçek %96.7)
    if node.scan_finite_ratios:
        mean_scan_ratio = float(np.mean(node.scan_finite_ratios))
        scan_ratio_ok = (0.85 <= mean_scan_ratio <= 0.995)
        olculen_str = f"%{mean_scan_ratio * 100:.1f} ({mean_scan_ratio:.3f})"
    else:
        mean_scan_ratio = 0.0
        scan_ratio_ok = False
        olculen_str = "Alınamadı"

    checks.append({
        'kategori': 'İçerik',
        'kontrol': '/scan sonlu ölçüm oranı',
        'olculen': olculen_str,
        'beklenen': '0.850 - 0.995',
        'durum': 'GEÇTİ' if scan_ratio_ok else 'KALDI',
        'detay': {'ortalama_oran': round(mean_scan_ratio, 4)},
    })

    # 2.5 /imu ivme z ortalaması 9.6–10.3 (gerçek 10.04)
    if node.imu_az_values:
        mean_az = float(np.mean(node.imu_az_values))
        az_ok = (9.6 <= mean_az <= 10.3)
        olculen_str = f"{mean_az:.3f} m/s²"
    else:
        mean_az = 0.0
        az_ok = False
        olculen_str = "Alınamadı"

    checks.append({
        'kategori': 'İçerik',
        'kontrol': '/imu ivme z ortalaması',
        'olculen': olculen_str,
        'beklenen': '9.60 - 10.30 m/s²',
        'durum': 'GEÇTİ' if az_ok else 'KALDI',
        'detay': {'ortalama_az': round(mean_az, 4)},
    })

    # 2.6 camera_info fx: renk 900–918
    if node.color_fx is not None:
        cfx = node.color_fx
        cfx_ok = (900.0 <= cfx <= 918.0)
        olculen_str = f"{cfx:.2f}"
    else:
        cfx = None
        cfx_ok = False
        olculen_str = "Alınamadı"

    checks.append({
        'kategori': 'İçerik',
        'kontrol': 'Kamera renk fx',
        'olculen': olculen_str,
        'beklenen': '900.0 - 918.0',
        'durum': 'GEÇTİ' if cfx_ok else 'KALDI',
        'detay': {'fx': round(cfx, 2) if cfx is not None else None},
    })

    # 2.7 camera_info fx: derinlik 425–435
    if node.depth_fx is not None:
        dfx = node.depth_fx
        dfx_ok = (425.0 <= dfx <= 435.0)
        olculen_str = f"{dfx:.2f}"
    else:
        dfx = None
        dfx_ok = False
        olculen_str = "Alınamadı"

    checks.append({
        'kategori': 'İçerik',
        'kontrol': 'Kamera derinlik fx',
        'olculen': olculen_str,
        'beklenen': '425.0 - 435.0',
        'durum': 'GEÇTİ' if dfx_ok else 'KALDI',
        'detay': {'fx': round(dfx, 2) if dfx is not None else None},
    })

    # 3. TF Kontrolleri
    tf_checks = evaluate_tf(node.tf_buffer)
    checks.extend(tf_checks)

    # 4. Zaman Senkronizasyonu (header.stamp ile /clock tutarlılığı < 0.5 s)
    if node.clock_diffs:
        max_diff = float(np.max(node.clock_diffs))
        mean_diff = float(np.mean(node.clock_diffs))
        time_ok = (max_diff < 0.5)
        olculen_str = f"{max_diff:.3f} s (ort: {mean_diff:.3f} s)"
    else:
        max_diff = None
        mean_diff = None
        time_ok = False
        olculen_str = "Ölçülemedi (mesaj yok)"

    checks.append({
        'kategori': 'Zaman',
        'kontrol': '/clock senkronizasyonu',
        'olculen': olculen_str,
        'beklenen': '< 0.5 s',
        'durum': 'GEÇTİ' if time_ok else 'KALDI',
        'detay': {
            'maks_fark_s': round(max_diff, 4) if max_diff is not None else None,
            'ort_fark_s': round(mean_diff, 4) if mean_diff is not None else None,
        },
    })

    return checks


def print_table_and_summary(checks: list) -> tuple:
    """Sonuç tablosunu terminale hizalı basar ve genel durumu döndürür."""
    header_kat = "Kategori"
    header_kon = "Kontrol / Konu"
    header_olc = "Ölçülen Değer"
    header_bek = "Beklenen Değer"
    header_dur = "Durum"

    col_w = (10, 36, 27, 24, 8)
    line_len = sum(col_w) + len(col_w) - 1

    print("\n" + "=" * line_len)
    row_hdr = (
        f"{header_kat:<{col_w[0]}} "
        f"{header_kon:<{col_w[1]}} "
        f"{header_olc:<{col_w[2]}} "
        f"{header_bek:<{col_w[3]}} "
        f"{header_dur:<{col_w[4]}}"
    )
    print(row_hdr)
    print("=" * line_len)

    current_kat = None
    for c in checks:
        if current_kat is not None and c['kategori'] != current_kat:
            print("-" * line_len)
        current_kat = c['kategori']

        # Kırpma veya hizalama
        kat_s = str(c['kategori'])[:col_w[0]]
        kon_s = str(c['kontrol'])[:col_w[1]]
        olc_s = str(c['olculen'])[:col_w[2]]
        bek_s = str(c['beklenen'])[:col_w[3]]
        dur_s = str(c['durum'])[:col_w[4]]

        row_str = (
            f"{kat_s:<{col_w[0]}} "
            f"{kon_s:<{col_w[1]}} "
            f"{olc_s:<{col_w[2]}} "
            f"{bek_s:<{col_w[3]}} "
            f"{dur_s:<{col_w[4]}}"
        )
        print(row_str)

    print("=" * line_len)

    # Değerlendirme: UYARI başarısız sayılmaz
    mandatory_checks = [c for c in checks if c['durum'] != 'UYARI']
    failed_checks = [c for c in checks if c['durum'] == 'KALDI']
    passed_checks = [c for c in checks if c['durum'] == 'GEÇTİ']

    k_failed = len(failed_checks)
    n_total = len(mandatory_checks)
    n_passed = len(passed_checks)

    if k_failed == 0:
        print(f"SONUÇ: GEÇTİ ({n_passed}/{n_total})\n")
        exit_code = 0
    else:
        print(f"SONUÇ: KALDI ({k_failed} başarısız)\n")
        exit_code = 1

    return exit_code, n_passed, n_total, k_failed


def save_yaml_report(path: str, checks: list, n_passed: int, n_total: int, k_failed: int, sure: float):
    """Sonuçları yapısal YAML formatında dosyaya yazar."""
    try:
        abs_path = os.path.abspath(path)
        os.makedirs(os.path.dirname(abs_path), exist_ok=True)

        rapor = {
            'tarih': datetime.now().isoformat(),
            'sure_saniye': float(sure),
            'sonuc': 'GEÇTİ' if k_failed == 0 else 'KALDI',
            'ozet': {
                'toplam_zorunlu_kontrol': n_total,
                'gecen_kontrol': n_passed,
                'basarisiz_kontrol': k_failed,
            },
            'kontroller': [
                {
                    'kategori': c['kategori'],
                    'kontrol': c['kontrol'],
                    'olculen': c['olculen'],
                    'beklenen': c['beklenen'],
                    'durum': c['durum'],
                    'detay': c.get('detay', {}),
                }
                for c in checks
            ],
        }

        with open(abs_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(rapor, f, allow_unicode=True, sort_keys=False)
        print(f"YAML raporu kaydedildi: {abs_path}")
    except Exception as e:
        print(f"HATA: YAML raporu kaydedilemedi ({path}): {e}", file=sys.stderr)


def save_side_by_side_image(path: str, color_bgr: np.ndarray, depth_raw: np.ndarray):
    """Son renk karesi ve derinlik renkli haritasını yan yana PNG olarak kaydeder."""
    try:
        abs_path = os.path.abspath(path)
        os.makedirs(os.path.dirname(abs_path), exist_ok=True)

        # 1. Derinlik verisini renklendir
        valid_mask = depth_raw > 0
        depth_colormap = np.zeros((depth_raw.shape[0], depth_raw.shape[1], 3), dtype=np.uint8)

        if np.any(valid_mask):
            v_min = float(np.percentile(depth_raw[valid_mask], 2))
            v_max = float(np.percentile(depth_raw[valid_mask], 98))
            if v_max > v_min:
                d_norm = np.clip((depth_raw - v_min) / (v_max - v_min) * 255.0, 0, 255).astype(np.uint8)
            else:
                d_norm = np.zeros_like(depth_raw, dtype=np.uint8)
            colored = cv2.applyColorMap(d_norm, cv2.COLORMAP_JET)
            colored[~valid_mask] = [0, 0, 0]
            depth_colormap = colored

        # 2. Boyutları eşitle: Renk karesinin yüksekliğine uyarla
        target_h, color_w = color_bgr.shape[:2]
        depth_h, depth_w = depth_colormap.shape[:2]
        target_depth_w = int(round(depth_w * (target_h / float(depth_h))))
        depth_resized = cv2.resize(depth_colormap, (target_depth_w, target_h), interpolation=cv2.INTER_NEAREST)

        # 3. Yan yana birleştir ve kaydet
        side_by_side = np.hstack([color_bgr, depth_resized])
        cv2.imwrite(abs_path, side_by_side)
        print(f"Görsel kaydedildi: {abs_path}")
    except Exception as e:
        print(f"HATA: Görsel kaydedilemedi ({path}): {e}", file=sys.stderr)


def main():
    ros_args = remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(
        description="LIMO Simülasyon Kabul ve Doğrulama Aracı (Görev N1)"
    )
    parser.add_argument(
        '--sure',
        type=float,
        default=15.0,
        help="Simülasyon dinleme süresi (saniye, varsayılan: 15)",
    )
    parser.add_argument(
        '--cikti',
        type=str,
        default=None,
        help="YAML formatında rapor çıktı dosyası yolu",
    )
    parser.add_argument(
        '--gorsel',
        type=str,
        default=None,
        help="Renk ve renklendirilmiş derinlik karesi için yan yana PNG çıktı yolu",
    )
    args = parser.parse_args(ros_args[1:])

    rclpy.init(args=None)
    node = SimKabulNode(sure=args.sure)

    print(f"============================================================")
    print(f"  LIMO Simülasyon Kabul Testi Başlatılıyor (Görev N1)")
    print(f"  Hedef Süre: {args.sure:.1f} saniye (simülasyon zamanı)")
    print(f"============================================================")
    print("Simülasyon saati (/clock) bekleniyor...")

    start_sim_time = None
    start_wall_time = time.time()
    max_wall_wait = max(args.sure * 3.5, 45.0)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now_sim = node.get_sim_time()

            if now_sim > 0.0:
                if start_sim_time is None:
                    start_sim_time = now_sim
                    node.start_sim_time = start_sim_time
                    print(f"Simülasyon zamanı aktif (t_sim={start_sim_time:.2f}s). Dinleniyor...")

                elapsed = now_sim - start_sim_time
                if elapsed >= args.sure:
                    node.elapsed_sim_time = elapsed
                    break

            # /clock hiç gelmezse veya simülasyon donarsa zaman aşımı
            wall_elapsed = time.time() - start_wall_time
            if wall_elapsed > max_wall_wait:
                print(f"\nUYARI: Maksimum bekleme süresi ({max_wall_wait:.1f}s) aşıldı.")
                if start_sim_time is not None:
                    node.elapsed_sim_time = now_sim - start_sim_time
                else:
                    node.elapsed_sim_time = 0.0
                break

        print(f"Dinleme tamamlandı ({node.elapsed_sim_time:.1f} sim saniyesi). Sonuçlar değerlendiriliyor...\n")

        # Kontrolleri hesapla
        checks = run_checks(node)

        # Tablo bas ve sonucu belirle
        exit_code, n_passed, n_total, k_failed = print_table_and_summary(checks)

        # YAML çıktısı
        if args.cikti:
            save_yaml_report(args.cikti, checks, n_passed, n_total, k_failed, args.sure)

        # Görsel çıktısı
        if args.gorsel:
            if node.last_color_img is not None and node.last_depth_img is not None:
                save_side_by_side_image(args.gorsel, node.last_color_img, node.last_depth_img)
            else:
                print(
                    "UYARI: Renk veya derinlik görüntüsü alınamadığı için görsel oluşturulamadı.",
                    file=sys.stderr,
                )

        return exit_code

    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu.")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
