from __future__ import absolute_import
from __future__ import print_function

# try:
#     import std_msgs
#     import pyros_msgs
# except ImportError:
#     # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
#     import pyros_setup
#     # We rely on default configuration to point us ot the proper distro
#     pyros_setup.configurable_import().configure().activate()
#     import std_msgs
#     import pyros_msgs

from .ros import (
    with_explicitly_matched_type,
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString,
    RosTextString,
    RosNested,
    RosList,
    create
)

__all__ = [
    'with_explicitly_matched_type',

    'RosBool',
    'RosInt8', 'RosInt16', 'RosInt32', 'RosInt64',
    'RosUInt8', 'RosUInt16', 'RosUInt32', 'RosUInt64',
    'RosFloat32', 'RosFloat64',
    'RosString', 'RosTextString',
    'RosOpt',
    'RosNested',
    'RosList',
    'create'
]
