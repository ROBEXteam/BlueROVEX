from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'seatrac'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        ('lib/' + package_name, [package_name + '/seatracLib.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='titouan',
    maintainer_email='titouan.belier@ensta.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points = {
        'console_scripts': [
            'seatrac_test_node = seatrac.seatrac_test_node:main',
            'seatrac_node = seatrac.seatrac:main'
            ]
    }
)
