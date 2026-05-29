from glob import glob
from setuptools import setup

package_name = "quest3_oculus_rviz"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="teleop",
    maintainer_email="user@example.com",
    description="RViz bridge for visualizing Meta Quest controller poses from oculus_reader.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "oculus_tf_node = quest3_oculus_rviz.oculus_tf_node:main",
            "franka_sim_ik_node = quest3_oculus_rviz.franka_sim_ik_node:main",
            "hand_teleop_sim_node = quest3_oculus_rviz.right_hand_teleop_sim_node:main",
            "right_hand_teleop_sim_node = quest3_oculus_rviz.right_hand_teleop_sim_node:main",
            "franky_cartesian_pose_node = quest3_oculus_rviz.franky_cartesian_pose_node:main",
            "teleop_realtime_plot_node = quest3_oculus_rviz.teleop_realtime_plot_node:main",
            "simple_quest_impedance_teleop_node = "
            "quest3_oculus_rviz.simple_quest_impedance_teleop_node:main",
        ],
    },
)
