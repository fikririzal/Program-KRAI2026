from setuptools import setup

package_name = 'bno055_imu'

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
    description='BNO055 IMU via Arduino Nano serial → sensor_msgs/Imu',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_node = bno055_imu.imu_node:main',
        ],
    },
)
