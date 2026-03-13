from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'd2oc_algorithm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WALL-E Team',
    maintainer_email='walle@example.com',
    description='D2OC exploration algorithm for autonomous navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'd2oc=d2oc_algorithm.d2oc_node:main',
        ],
    },
)
