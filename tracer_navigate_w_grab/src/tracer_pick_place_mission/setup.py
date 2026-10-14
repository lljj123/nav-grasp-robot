from glob import glob
from setuptools import find_packages, setup


package_name = "tracer_pick_place_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TRACER Piper Developer",
    maintainer_email="maintainer@example.com",
    description="Two-cycle Nav2 pick/place mission and return to start for TRACER Piper.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_place_mission = tracer_pick_place_mission.mission_node:main",
            "mission_console = tracer_pick_place_mission.mission_console:main",
        ],
    },
)
