from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot2_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huda',
    maintainer_email='khoirulhuda2793@gmail.com',
    description='Launch files for robot2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_cmd_vel = robot2_launch.joy_to_cmd_vel:main',
        ],
    },
)
