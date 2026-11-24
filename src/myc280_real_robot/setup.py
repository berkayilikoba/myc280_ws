import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'myc280_real_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkay',
    maintainer_email='ilikobaberkay@gmail.com',
    description='MyCobot 280 gerçek robot kodları',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_approach_real = myc280_real_robot.cube_approach_real:main',
        ],
    },
)
