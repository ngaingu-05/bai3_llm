from glob import glob
from setuptools import find_packages, setup


package_name = "ur3_llm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Nghia",
    maintainer_email="student@example.com",
    description="LLM task planning and validated UR3 robot skills",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "llm_planner = ur3_llm_control.llm_planner:main",
            "camera_detector = ur3_llm_control.camera_detector:main",
        ],
    },
)
