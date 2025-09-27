import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "tinkerbot_gazebo_sim"


def get_model_data_files() -> list[tuple[str, list]]:
    """Get a list of all targets & sources needed for building custom gazebo models"""
    data_files = []
    share_dir = os.path.join("share", PACKAGE_NAME)

    # Walk models directory for all files
    for root, _, files in os.walk("models"):
        if not files:
            continue

        # Map source files to target share directory
        sources = [os.path.join(root, file) for file in files]
        target = os.path.join(share_dir, root)
        data_files.append((target, sources))

    return data_files


setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME), glob("description/*")),
        (os.path.join("share", PACKAGE_NAME), glob("config/*")),
        (os.path.join("share", PACKAGE_NAME, "worlds"), glob("worlds/*")),
    ]
    + get_model_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="",
    maintainer_email="todo@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
