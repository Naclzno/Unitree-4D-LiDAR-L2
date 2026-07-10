from setuptools import find_packages, setup


package_name = "um981_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["um981", "um981.*", "um981_ros", "um981_ros.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="UM981 User",
    maintainer_email="user@example.com",
    description="Python SDK and ROS2 node for the Unicore UM981 receiver.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "um981_echo = um981.cli_echo:main",
            "um981_node = um981_ros.node:main",
        ],
    },
)
