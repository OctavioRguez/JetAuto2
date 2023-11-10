#!/usr/bin/python3
from os.path import dirname, abspath, basename
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = [basename(dirname(abspath(__file__)))],
    package_dir = {'' : 'scripts'}
)

setup(**setup_args)