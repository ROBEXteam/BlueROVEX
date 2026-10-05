from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'bluerov_docking_guaranteed'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='Titouan BELIER',
    maintainer_email='titouan.belier@ensta.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guidance_node = bluerov_docking_guaranteed.guidance:main',
            'navigation_node = bluerov_docking_guaranteed.navigation:main',
            'planner_node = bluerov_docking_guaranteed.planner:main',],
    },
)





