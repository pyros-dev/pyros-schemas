#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'pyros_schemas',
    ],
    package_dir={
        'pyros_schemas': 'src/pyros_schemas'
    },
)

setup(**setup_args)
