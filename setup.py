#!/usr/bin/env python3
# license removed for brevity
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['Khezu'],
    package_dir={'': 'src'}
)

setup(**d)
