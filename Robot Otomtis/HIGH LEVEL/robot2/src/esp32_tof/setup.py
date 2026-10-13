from setuptools import setup

package_name = 'esp32_tof'

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
    description='ESP32 TOF 8-sensor via USB serial CSV → sensor_tof/TofArray',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tof_node = esp32_tof.tof_node:main',
        ],
    },
)
