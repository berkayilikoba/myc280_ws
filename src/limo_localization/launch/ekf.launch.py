"""
robot_localization EKF — LIMO odometri füzyonu.

Kullanım:
    # Gerçek robot (limo_base pub_odom_tf:=false ile çalışıyor olmalı)
    ros2 launch limo_localization ekf.launch.py

    # Gazebo simülasyonu: ekf_sim.yaml + sim saati
    ros2 launch limo_localization ekf.launch.py sim:=true

    # Kayıt (bags/LIMO_2026-10-09_*/bag/.../sensor) üzerinde: sim saati, TF yayınlamaz
    # (bag'de odom -> base_link zaten var), eksik base_link -> imu_link'i ekler.
    # Çıktı: /odometry/filtered — /odom_duz ile karşılaştırılabilir.
    ros2 launch limo_localization ekf.launch.py bag:=true
    ros2 bag play <bag>/sensor --clock

    # Başka bir parametre dosyasıyla deneme
    ros2 launch limo_localization ekf.launch.py params_file:=/yol/ekf_deneme.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('limo_localization'), 'config')

    sim = LaunchConfiguration('sim')
    bag = LaunchConfiguration('bag')
    params_file = LaunchConfiguration('params_file')

    default_params = PythonExpression([
        "'", os.path.join(config_dir, 'ekf_sim.yaml'), "' if '", sim, "' == 'true' else '",
        os.path.join(config_dir, 'ekf_robot.yaml'), "'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='true: Gazebo (ekf_sim.yaml, use_sim_time), false: gerçek robot'),
        DeclareLaunchArgument(
            'bag', default_value='false',
            description='true: kayıt oynatma (ekf_robot.yaml, use_sim_time, TF yayınlamaz)'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='EKF parametre dosyası; verilmezse sim argümanına göre seçilir'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': sim}],
            condition=UnlessCondition(bag)),

        # Bag modu: kayıtta odom -> base_link zaten var; ikinci ebeveyn TF ağacını bozar
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': True, 'publish_tf': False}],
            condition=IfCondition(bag)),

        # Kayıtlarda base_link -> imu_link yok (gerçek robotta limo_start.launch.py yayınlıyor)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(bag)),
    ])
