from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["robot_fingers", "robot_fingers.tasks"],
    package_dir={"": "python"}
)

setup(**d)
