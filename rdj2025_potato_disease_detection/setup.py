from setuptools import find_packages, setup

package_name = 'rdj2025_potato_disease_detection'

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
    maintainer='lenny',
    maintainer_email='codewithlennylen254@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'potato_disease_detection_node = rdj2025_potato_disease_detection.potato_disease_detection_node:main',
            'publish_test_image = rdj2025_potato_disease_detection.test_image_publisher:main',
        ],
    },
)
