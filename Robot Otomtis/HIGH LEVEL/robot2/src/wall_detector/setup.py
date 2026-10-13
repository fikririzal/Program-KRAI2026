from setuptools import setup

package_name = 'wall_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krai',
    maintainer_email='adekkechil31@gmail.com',
    description='RANSAC wall detector: kiri, kanan, belakang dari scan lidar',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wall_node = wall_detector.wall_node:main',
        ],
    },
)
