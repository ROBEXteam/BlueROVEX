from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'aruco_lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tbelier',
    maintainer_email='titouan.belier@ensta.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "bluerov_ros_camera_node = aruco_lib.bluerov_ros_camera:main",
            "vision_treatment_node = aruco_lib.vision_treatment:main",
            "camera_info_publisher_node = aruco_lib.camera_info_publisher:main",
        ],
    },
)
