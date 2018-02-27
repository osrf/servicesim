#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rqt_servicesim_score'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy']
)

setup(**setup_args)
