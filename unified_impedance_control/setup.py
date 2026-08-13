from glob import glob

from setuptools import setup


package_name = "unified_impedance_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lumos",
    maintainer_email="user@example.com",
    description="Y-button authority gate shared by EasyDP inference and Quest teleoperation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_authority_node = unified_impedance_control.control_authority_node:main",
            "authority_data_recorder_node = unified_impedance_control.authority_data_recorder_node:main",
        ],
    },
)
