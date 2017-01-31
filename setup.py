#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# TODO : mutate this into a catkin_pip package, with both pip and ros pkg

# TODO : property based testing to make sure we behave the same way as <insert reference ROS/python parser here> + optional fields feature

# TODO : pyros should rely on this package for serializing/deserializing to ROS messages.

# TODO : eventually extend to not rely on any ros package as dependency, only python pip packages.

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'pyros_schemas',
    ],
    package_dir={
        'pyros_schemas': 'pyros_schemas'
    },
)

setup(**setup_args)
