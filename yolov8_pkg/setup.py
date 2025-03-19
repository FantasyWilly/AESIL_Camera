import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yolov8_pkg'

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
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_ros_node = yolov8_pkg.rtsp_ros2:main',
            'yolo_ros_node = yolov8_pkg.yolov8_test:main',
            'yolo_rtsp_ros_node = yolov8_pkg.yolov8_detect:main',
            'yolov8_mod_node = yolov8_pkg.yolov8_mod:main',
            'yolov8_sv_node = yolov8_pkg.yolov8_sv:main',
            'yolov8_sv_gps_node = yolov8_pkg.yolov8_sv_gps:main',
            'yolov8_sv_gps_save_node = yolov8_pkg.yolov8_sv_gps_save:main',
        ],
    },
)
