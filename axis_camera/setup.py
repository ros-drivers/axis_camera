from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'axis_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', 'axis_camera.launch'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Iverach-Brereton',
    maintainer_email='civerachb@clearpathrobotics.com',
    description='ROS 2 driver for fixed and PTZ Axis cameras',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis_camera_node = axis_camera.axis_camera_node:main'
        ],
    },
)
