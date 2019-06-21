#!/usr/bin/env python

from disutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_disutils_setup(
        package=['motor_control']
        package_dir={'': 'src'}
)

setup(**setup_args)
