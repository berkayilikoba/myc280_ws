"""
RTAB-Map RGB-D görsel odometri (+ isteğe bağlı SLAM) — LIMO + Orbbec DaBai DC1.

Görsel odometri /vo'ya yayınlanır ve TF yayınlamaz; böylece tekerlek odometrisi
(/odom, odom -> base_link) ile yan yana kaydedilip karşılaştırılabilir.
Tekerlek odometrisi TF'i hareket tahmini (guess) olarak kullanılır.

Kamera ayarlarının gerekçesi (2026-10-09 kare incelemesi):
  * kamera 18 cm yüksekte, ufuk görüntünün ortasında; alt yarı parlak, dokusuz zemin
    ve kapı yansıması -> alt %40 özellik aramasından çıkarıldı (Vis/RoiRatios)
  * DC1 derinliği 4 m ötesinde gürültülü -> Vis/MaxDepth 4.0
  * robot düzlemsel -> Reg/Force3DoF

Kullanım (bag üzerinde):
    ros2 launch limo_localization rtabmap_rgbd.launch.py use_sim_time:=true
    ros2 bag play ~/limo_bags/rtab_01 --clock
SLAM'i de açmak için: slam:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    roi = LaunchConfiguration('roi_ratios')

    common = {
        'frame_id': 'base_link',
        'use_sim_time': use_sim_time,
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,
        'wait_for_transform': 0.2,
        'Reg/Force3DoF': 'true',
        'Vis/MaxDepth': '4.0',
        'Vis/MinInliers': '15',
        'Vis/RoiRatios': roi,
    }
    remaps = [('rgb/image', '/camera/color/image_raw'),
              ('rgb/camera_info', '/camera/color/camera_info'),
              ('depth/image', '/camera/depth/image_raw')]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='bag oynatırken true (+ ros2 bag play --clock)'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true: rtabmap SLAM düğümünü de başlat (/vo ile)'),
        DeclareLaunchArgument('roi_ratios', default_value='0 0 0 0.4',
                              description='Vis/RoiRatios: sol sağ üst alt kırpma oranları'),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
            output='screen',
            parameters=[common, {
                'odom_frame_id': 'vo',
                'publish_tf': False,
                'guess_frame_id': 'odom',
                'Odom/Strategy': '0',
                'Odom/ResetCountdown': '1',
            }],
            remappings=remaps + [('odom', 'vo')]),

        Node(
            condition=IfCondition(slam),
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            output='screen',
            parameters=[common, {
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'RGBD/OptimizeMaxError': '3.0',
                'Grid/FromDepth': 'true',
            }],
            remappings=remaps + [('odom', 'vo')],
            arguments=['--delete_db_on_start']),
    ])
