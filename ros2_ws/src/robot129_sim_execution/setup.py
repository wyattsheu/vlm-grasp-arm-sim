from setuptools import find_packages, setup

package_name = "robot129_sim_execution"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/adapter.launch.py"]),
        ("share/" + package_name + "/config", ["config/adapter_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robot 129",
    maintainer_email="wyattsheu@example.com",
    description=(
        "Simulation-only FollowJointTrajectory adapter bridging MoveIt/MTC to the "
        "live Isaac Robot 129 JointTrajectory topics."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "adapter_node = robot129_sim_execution.adapter_node:main",
            "run_grasp_motion = robot129_sim_execution.run_grasp_motion:main",
        ],
    },
)
