#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['behavior_explore_and_search_victims_using_arm'],
    package_dir = {'': 'src'}
)

setup(**d)