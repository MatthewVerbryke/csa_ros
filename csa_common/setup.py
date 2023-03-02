#!/usr/bin/env python

"""
  http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  NOTE: Do not manually invoke this script, use catkin instead.
"""


from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


# Fetch values from 'package.xml'
setup_args = generate_distutils_setup(
    packages = ['csa_common'],
    package_dir = {'': 'src'},
)

setup(**setup_args)
