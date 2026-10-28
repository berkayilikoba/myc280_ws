from glob import glob

from setuptools import setup

package_name = 'limo_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunes',
    description='LIMO otonom gorev katmani',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'patrol = limo_autonomy.patrol_node:main',
        'kavrama = limo_autonomy.kavrama_dugumu:main',
        'sahte_algi = limo_autonomy.sahte_algi:main',
        'gorev = limo_autonomy.gorev_dugumu:main',
        'kavrama_gorsel = limo_autonomy.kavrama_gorsel:main',
        'renk_algi = limo_autonomy.renk_algi:main',
        'hareketli_kalkan = limo_autonomy.hareketli_kalkan:main',
    ]},
)
