# Package setup for the kestrel ROS 2 Python package
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'kestrel'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'kestrel_iris'),
         glob('models/kestrel_iris/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Nieves',
    maintainer_email='andrewjnieves1@gmail.com',
    description='Autonomous inspection drone nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'telemetry_monitor = kestrel.telemetry_monitor:main',
            'flight_commander = kestrel.flight_commander:main',
            'safety_guard = kestrel.safety_guard:main',
        ],
    },
)
