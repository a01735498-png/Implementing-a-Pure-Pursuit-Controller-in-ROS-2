from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pure_pursuit_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rebeca_s_e',
    maintainer_email='a01735498@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_recorder = pure_pursuit_controller.path_recorder:main',
            'pure_pursuit_node = pure_pursuit_controller.pure_pursuit_node:main',
        ],
    },
)
