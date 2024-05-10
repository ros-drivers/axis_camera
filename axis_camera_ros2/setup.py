from setuptools import find_packages, setup

package_name = 'axis_camera_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Iverach-Brereton',
    maintainer_email='civerachb@clearpathrobotics.com',
    description='AXIS ROS2 package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis_camera_node = axis_camera_ros2.axis_camera_node:main'
        ],
    },
)
