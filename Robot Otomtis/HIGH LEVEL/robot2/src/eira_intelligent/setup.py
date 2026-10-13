import os
from glob import glob
from setuptools import setup

package_name = "eira_intelligent"


def find_eira_packages(base_dir="scripts", package_prefix="eira_intelligent"):
    """
    Auto-discover Python packages inside scripts/ and subdirectories.
    """
    packages = [package_prefix]

    for root, dirs, files in os.walk(base_dir):
        if "__init__.py" not in files:
            continue

        rel_path = os.path.relpath(root, base_dir)

        if rel_path == ".":
            continue

        package = package_prefix + "." + rel_path.replace(os.sep, ".")
        packages.append(package)

    return packages


setup(
    name=package_name,
    version="0.1.0",

    packages=find_eira_packages(),
    package_dir={package_name: "scripts"},

    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],

    install_requires=[
        "setuptools",
        "rclpy",
        "py_trees",
    ],

    zip_safe=True,

    maintainer="Paulus Windi Kurniawan CE 23",
    maintainer_email="pwindikurniawan@gmail.com",
    description="Intelligent Node",
    license="MIT",

    entry_points={
        "console_scripts": [
            "intelligent_node = eira_intelligent.intelligent_node:main",
        ],
    },
)
