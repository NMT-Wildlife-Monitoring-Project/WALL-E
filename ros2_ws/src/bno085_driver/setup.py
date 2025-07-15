from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'bno085_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='pi5-walle',
    maintainer_email='user@todo.todo',
    description='ROS 2 driver for bno085 IMU sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno085_node = bno085_driver.bno085_node:main',
        ],
    },
)
