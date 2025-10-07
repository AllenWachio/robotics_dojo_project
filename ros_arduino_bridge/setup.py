from setuptools import setup
import os
from glob import glob

package_name = 'ros_arduino_bridge'

# Helper function to filter out deployment-related launch files from main launch directory
def get_main_launch_files():
    """Get main launch files, excluding those that have deployment equivalents"""
    all_launch_files = glob('launch/*.py')
    # Exclude files that are now in deployment structure
    exclude_files = ['launch/arduino_only.launch.py']  # Add more if needed
    return [f for f in all_launch_files if f not in exclude_files]

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Main launch files (excluding deployment duplicates)
        (os.path.join('share', package_name, 'launch'), get_main_launch_files()),
        # Deployment launch files - properly organized
        (os.path.join('share', package_name, 'deployment/pi/launch'), glob('deployment/pi/launch/*.py')),
        (os.path.join('share', package_name, 'deployment/laptop/launch'), glob('deployment/laptop/launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'deployment/laptop/config'), glob('deployment/laptop/config/*.yaml') + glob('deployment/laptop/config/*.rviz')),
        # URDF and meshes
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.gazebo') + glob('urdf/*.trans')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        # Deployment folders - explicit file selection to avoid __pycache__
        (os.path.join('share', package_name, 'deployment/laptop'), [
            f for f in glob('deployment/laptop/*') 
            if os.path.isfile(f) and not f.endswith('.pyc') and '__pycache__' not in f
        ]),
        (os.path.join('share', package_name, 'deployment/pi'), [
            f for f in glob('deployment/pi/*') 
            if os.path.isfile(f) and not f.endswith('.pyc') and '__pycache__' not in f
        ]),
        (os.path.join('share', package_name, 'deployment/shared'), [
            f for f in glob('deployment/shared/*') 
            if os.path.isfile(f) and not f.endswith('.pyc') and '__pycache__' not in f
        ]),
        # Organized scripts folder
        (os.path.join('share', package_name, 'deployment/scripts'), [
            'deployment/scripts/README.md'
        ]),
        (os.path.join('share', package_name, 'deployment/scripts/laptop'), [
            f for f in glob('deployment/scripts/laptop/*') 
            if os.path.isfile(f) and not f.endswith('.pyc') and '__pycache__' not in f
        ]),
        (os.path.join('share', package_name, 'deployment/scripts/pi'), [
            f for f in glob('deployment/scripts/pi/*') 
            if os.path.isfile(f) and not f.endswith('.pyc') and '__pycache__' not in f
        ]),
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
            'slam_diagnostics = ros_arduino_bridge.slam_diagnostics:main',
        ],
    },
)

