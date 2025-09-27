from setuptools import setup
import os
from glob import glob

package_name = 'rpi_camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Allen Kizito Wachio',
    maintainer_email='allenkizitowachio@gmail.com',
    description='Raspberry Pi Camera ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = rpi_camera_package.camera_publisher:main',
            'color_detection_subscriber = rpi_camera_package.color_detection_subscriber:main',
        ],
    },
)
