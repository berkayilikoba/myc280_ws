"""LIMO kavrama ve görev katmanı launch dosyası.

Kullanım örnekleri:
  Simülasyonda sahte algı ile:
    ros2 launch limo_autonomy kavrama.launch.py use_sim_time:=true algi:=sahte

  Simülasyonda gerçek renk algısı (renk_algi) ile:
    ros2 launch limo_autonomy kavrama.launch.py use_sim_time:=true algi:=renk hata_ayiklama:=true

  Geriye dönük uyumluluk (sahte_algi:=true):
    ros2 launch limo_autonomy kavrama.launch.py use_sim_time:=true sahte_algi:=true

  Görev isteği:
    ros2 topic pub --once /gorev/istek std_msgs/String "{data: elma}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Kavrama, görev yöneticisi ve algı düğümlerini (sahte veya renk_algi) başlatır."""
    try:
        default_harita = os.path.join(
            get_package_share_directory('limo_bringup'),
            'maps',
            'lab_oda.yaml',
        )
    except Exception:
        default_harita = PathJoinSubstitution(
            [FindPackageShare('limo_bringup'), 'maps', 'lab_oda.yaml']
        )

    try:
        default_nesneler = os.path.join(
            get_package_share_directory('limo_autonomy'),
            'config',
            'nesneler_lab_oda.yaml',
        )
    except Exception:
        default_nesneler = PathJoinSubstitution(
            [FindPackageShare('limo_autonomy'), 'config', 'nesneler_lab_oda.yaml']
        )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Simülasyon (Gazebo) saatini kullan (use_sim_time).',
    )

    harita_arg = DeclareLaunchArgument(
        'harita',
        default_value=default_harita,
        description='Kavrama yaklaşım planlayıcısı için harita YAML dosyası.',
    )

    nesneler_arg = DeclareLaunchArgument(
        'nesneler',
        default_value=default_nesneler,
        description='Görev yöneticisi için nesne ve istasyon konumları YAML yapılandırma dosyası.',
    )

    kol_sahte_arg = DeclareLaunchArgument(
        'kol_sahte',
        default_value='true',
        description='Gerçek robot kolu servisi yoksa sahte kol başarılı kabul edilir.',
    )

    algi_arg = DeclareLaunchArgument(
        'algi',
        default_value='sahte',
        choices=['sahte', 'renk', 'yok'],
        description='Algı kaynağı: sahte (sahte_algi düğümü) | renk (renk_algi düğümü) | yok.',
    )

    sahte_algi_arg = DeclareLaunchArgument(
        'sahte_algi',
        default_value='',
        description='Geriye dönük uyumluluk için: sahte_algi:=true sahte algıyı etkinleştirir.',
    )

    hata_ayiklama_arg = DeclareLaunchArgument(
        'hata_ayiklama',
        default_value='false',
        description='Renk algı düğümü için hata ayıklama görüntüsü yayınla (/kavrama/algi_goruntu).',
    )

    nesne_x_arg = DeclareLaunchArgument(
        'nesne_x',
        default_value='5.62',
        description='Sahte algı için nesnenin dünya (map) x koordinatı (m).',
    )

    nesne_y_arg = DeclareLaunchArgument(
        'nesne_y',
        default_value='0.8',
        description='Sahte algı için nesnenin dünya (map) y koordinatı (m).',
    )

    nesne_z_arg = DeclareLaunchArgument(
        'nesne_z',
        default_value='0.10',
        description='Sahte algı için nesnenin dünya (map) z koordinatı (m).',
    )

    # İkisi aynı anda çalışmasın:
    # algi:=renk açıkça seçilmişse renk_algi çalışır.
    # sahte_algi:=true veya (algi:=sahte ve sahte_algi!=false) iken sahte_algi çalışır.
    sahte_kosul = PythonExpression([
        "'", LaunchConfiguration('algi'), "' != 'renk' and '",
        LaunchConfiguration('algi'), "' != 'yok' and ('",
        LaunchConfiguration('sahte_algi'), "'.lower() in ('true', '1') or ('",
        LaunchConfiguration('algi'), "' == 'sahte' and '",
        LaunchConfiguration('sahte_algi'), "'.lower() not in ('false', '0')))"
    ])

    renk_kosul = PythonExpression([
        "'", LaunchConfiguration('algi'), "' == 'renk' and '",
        LaunchConfiguration('sahte_algi'), "'.lower() not in ('true', '1')"
    ])

    kavrama_node = Node(
        package='limo_autonomy',
        executable='kavrama',
        name='kavrama',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'harita': LaunchConfiguration('harita'),
        }],
    )

    gorev_node = Node(
        package='limo_autonomy',
        executable='gorev',
        name='gorev',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'nesneler_dosyasi': LaunchConfiguration('nesneler'),
            'nesneler': LaunchConfiguration('nesneler'),
            'kol_sahte': ParameterValue(LaunchConfiguration('kol_sahte'), value_type=bool),
        }],
    )

    sahte_algi_node = Node(
        package='limo_autonomy',
        executable='sahte_algi',
        name='sahte_algi',
        output='screen',
        condition=IfCondition(sahte_kosul),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'nesne_x': ParameterValue(LaunchConfiguration('nesne_x'), value_type=float),
            'nesne_y': ParameterValue(LaunchConfiguration('nesne_y'), value_type=float),
            'nesne_z': ParameterValue(LaunchConfiguration('nesne_z'), value_type=float),
        }],
    )

    renk_algi_node = Node(
        package='limo_autonomy',
        executable='renk_algi',
        name='renk_algi',
        output='screen',
        condition=IfCondition(renk_kosul),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'hz': 5.0,
            'hata_ayiklama': ParameterValue(LaunchConfiguration('hata_ayiklama'), value_type=bool),
        }],
    )

    gorsel_arg = DeclareLaunchArgument(
        'gorsel', default_value='true',
        description='Kavrama görselleştirme düğümü (/kavrama/isaretler MarkerArray).')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='RViz2 kavrama.rviz yapılandırmasıyla başlat.')

    gorsel_node = Node(
        package='limo_autonomy',
        executable='kavrama_gorsel',
        name='kavrama_gorsel',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gorsel')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_kavrama',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(get_package_share_directory('limo_autonomy'), 'config', 'kavrama.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        algi_arg,
        sahte_algi_arg,
        hata_ayiklama_arg,
        gorsel_arg,
        rviz_arg,
        use_sim_time_arg,
        harita_arg,
        nesneler_arg,
        kol_sahte_arg,
        nesne_x_arg,
        nesne_y_arg,
        nesne_z_arg,
        kavrama_node,
        gorev_node,
        sahte_algi_node,
        renk_algi_node,
        gorsel_node,
        rviz_node,
    ])
