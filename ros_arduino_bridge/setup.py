from setuptools import setup
import os
from glob import glob

package_name = 'ros_arduino_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),  # ADD THIS
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),          # ADD THIS
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),      # ADD THIS
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 bridge for Arduino-based 4WD robot',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_arduino_bridge = ros_arduino_bridge.ros_arduino_bridge:main',
            'scan_throttle = ros_arduino_bridge.scan_throttle:main',
        ],
    },
)

