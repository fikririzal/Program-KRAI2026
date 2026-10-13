import os
from setuptools import setup

package_name = 'eira_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.ui'],
    package_dir={package_name: 'scripts'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: [
            'web/index.html',
            'web/emergency.html',
        ],
    },
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paulus Windi Kurniawan | CE 23',
    maintainer_email='pwindikurniawan@gmail.com',
    description='Interface for EIRA 2D field visualization and match configuration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface_node = eira_interface.interface_node:main',
            'reciverweb = eira_interface.reciverweb:main',
        ],
    },
)