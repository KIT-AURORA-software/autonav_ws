from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonav'

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
        ('share/' + package_name + '/description', glob('description/*.xacro')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fuga1129',
    maintainer_email='sakihama1129@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_gps = autonav.nav_gps:main',
            'nav_gps_csv = autonav.nav_gps_csv:main',
            'nav_gps_gate_csv = autonav.nav_gps_gate_csv:main',
        ],
    },
)
