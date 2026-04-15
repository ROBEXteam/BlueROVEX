from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'bluerov_control'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*/*.launch.py')), #à sup
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christophe Viel',
    maintainer_email='christophe.viel@ensta.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = bluerov_control.control:main',
            'mavlink_node = bluerov_control.mavlink:main',
            'camera_node = bluerov_control.camera:main',
            'rov_selector_node = bluerov_control.rov_selector:main',
        ],
    },
)





