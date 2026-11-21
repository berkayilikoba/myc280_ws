import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'myc280_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['myc280_sim', 'myc280_sim.*', 'scripts', 'scripts.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkay',
    maintainer_email='ilikobaberkay@gmail.com',
    description='myCobot 280 simulation and collision test nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'random_collision_test = myc280_sim.random_collision_test:main',
            'cube_approach = scripts.cube_approach:main',
        ],
    },
)
