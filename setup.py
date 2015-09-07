#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['axis_camera'],
    package_dir={'': 'scripts'},
    install_requires=['rospy', 'urllib2'],
    )

setup(**setup_args)
