import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'target_position_pkg'

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
            'target_gps_node                       = target_position_pkg.target_gps:main',
            'laser_target_gps_node                 = target_position_pkg.laser_target_gps:main',
            'vio_laser_target_gps_node             = target_position_pkg.vio_laser_target_gps:main',
            'vio_laser_target_gps_fix_heading_node = target_position_pkg.vio_laser_target_gps_fix_heading:main',
        ],
    },
)
