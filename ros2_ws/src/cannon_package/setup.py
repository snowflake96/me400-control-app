#!/usr/bin/env python3
from setuptools import setup, find_packages
import os
from glob import glob

# This file is now redundant. Python scripts are installed in CMakeLists.txt

package_name = 'cannon_package'

setup(
    name=package_name,
    version='0.0.1',
    # packages=find_packages(exclude=['test']),  # Finds the inner cannon_package folder
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'templates'), glob(os.path.join(package_name, 'templates', '*'))),
        (os.path.join('share', package_name, 'static'), glob(os.path.join(package_name, 'static', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jisang You',
    maintainer_email='jisangyou1@gmail.com',
    description='A ROS 2 package with Python and C++ nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_webserver_node = cannon_package.pid_webserver:main',
            'yolo_node = cannon_package.yolo:main',
        ],
    },
)
