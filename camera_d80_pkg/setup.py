import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_d80_pkg'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='fantasywilly',
    maintainer_email='bc697522h04@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gcu_ros2_main_node        = camera_d80_pkg.gcu_ros2_main:main',
            'xbox_ros2_controller_node = camera_d80_pkg.xbox_ros2_controller:main',
            'xbox_air_node             = camera_d80_pkg.xbox_air:main',
        ],
    },
)
