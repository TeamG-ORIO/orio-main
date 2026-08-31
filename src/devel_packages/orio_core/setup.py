## catkin setup for the pure orio_core python package.
## Do not invoke directly — used by catkin_python_setup() in CMakeLists.txt.
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["orio_core"],
    package_dir={"": "."},
)

setup(**setup_args)
