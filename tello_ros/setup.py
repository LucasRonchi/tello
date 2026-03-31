from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tello_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Ronchi',
    maintainer_email='lucascronchi2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = tello_ros.controller_node:main',
            'state_node = tello_ros.state_node:main',
            'camera_node = tello_ros.camera_node:main',
        ],
    },
)
