from glob import glob

from setuptools import find_packages, setup

package_name = 'limo_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hilal Ates',
    maintainer_email='ateshilal3344@gmail.com',
    description='LIMO odometri füzyonu (robot_localization EKF) ve odometri hata ölçüm araçları',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_pattern = limo_localization.drive_pattern:main',
            'donus_hatasi = limo_localization.donus_hatasi:main',
        ],
    },
)
