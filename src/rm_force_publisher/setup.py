from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rm_force_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='六维力传感器重力补偿与基座坐标系变换发布节点',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'force_publisher = rm_force_publisher.force_publisher_node:main',
        ],
    },
)
