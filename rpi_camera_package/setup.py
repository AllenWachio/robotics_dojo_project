from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rpi_camera_package'

# Helper function to safely add data files only if they exist
def add_data_files(install_dir, pattern):
    files = glob(pattern)
    if files:
        return [(install_dir, files)]
    return []

# Collect all data files
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add launch files - Pi
data_files.extend(add_data_files(
    os.path.join('share', package_name, 'launch', 'pi'),
    'launch/pi/*.launch.py'))

# Add launch files - Laptop
data_files.extend(add_data_files(
    os.path.join('share', package_name, 'launch', 'laptop'),
    'launch/laptop/*.launch.py'))

# Add legacy launch files (root level - backwards compatibility)
data_files.extend(add_data_files(
    os.path.join('share', package_name, 'launch'),
    'launch/*.launch.py'))

# Add config files - Pi
data_files.extend(add_data_files(
    os.path.join('share', package_name, 'config', 'pi'),
    'config/pi/*.yaml'))

# Add config files - Laptop
data_files.extend(add_data_files(
    os.path.join('share', package_name, 'config', 'laptop'),
    'config/laptop/*.yaml'))

# Add RViz configs
data_files.extend(add_data_files(
    os.path.join('share', package_name, 'rviz'),
    'rviz/*.rviz'))

# Add ML models
data_files.extend(add_data_files(
    os.path.join('share', package_name, 'models'),
    'rpi_camera_package/models/*.pth'))

# Add shell scripts - Pi (install to lib directory for ros2 run)
data_files.extend(add_data_files(
    os.path.join('lib', package_name, 'pi'),
    'scripts/pi/*.sh'))

# Add shell scripts - Laptop (install to lib directory for ros2 run)
data_files.extend(add_data_files(
    os.path.join('lib', package_name, 'laptop'),
    'scripts/laptop/*.sh'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Allen Kizito Wachio',
    maintainer_email='allenkizitowachio@gmail.com',
    description='ROS2 Camera Package - Distributed Pi/Laptop with V4L2, color detection, and potato disease detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Pi camera nodes
            'rpicam_node = rpi_camera_package.rpicam_node:main',
            # Laptop processing nodes
            'color_detection_node = rpi_camera_package.laptop_nodes.color_detection_node:main',
            'disease_detection_node = rpi_camera_package.laptop_nodes.disease_detection_node:main',
        ],
    },
)
