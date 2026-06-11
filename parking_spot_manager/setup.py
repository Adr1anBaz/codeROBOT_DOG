from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'parking_spot_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Adrian Bazaldua',
    maintainer_email='adrian@todo.com',
    description='Parking spot management for Go2 robot re-localization',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_spot_saver = parking_spot_manager.nodes.parking_spot_saver_node:main',
            'parking_spot_loader = parking_spot_manager.nodes.parking_spot_loader_node:main',
        ],
    },
)
