# Copied from the catkin 0.7.29 documentation.
# http://docs.ros.org/en/kinetic/api/catkin/html/howto/format2/installing_python.html
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['labjack'],
    package_dir={'': 'src'}
)

setup(**setup_args)
