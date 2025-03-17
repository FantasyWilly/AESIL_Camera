import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_tt30_pkg'

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
            'camera_feedback_publisher_node = camera_tt30_pkg.camera_feedback_ros2:main',
            'camera_feedback_publisher_gui_node = camera_tt30_pkg.camera_gui_ros2:main',
            'target_position_node = camera_tt30_pkg.camera_laser_dist:main',
            'vio_target_position_node = camera_tt30_pkg.camera_vio_laser_dist:main',
            'xbox_air_node = camera_tt30_pkg.xbox_air:main'
        ],
    },
)
