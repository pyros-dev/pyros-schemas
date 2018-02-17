from __future__ import absolute_import, print_function

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
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString,
    RosTextString,
    RosTime,
    RosDuration,
    RosNested,
    RosList,
    RosOpt,
    create,
    with_service_schemas,
)

__all__ = [
    'RosBool',
    'RosInt8', 'RosInt16', 'RosInt32', 'RosInt64',
    'RosUInt8', 'RosUInt16', 'RosUInt32', 'RosUInt64',
    'RosFloat32', 'RosFloat64',
    'RosString', 'RosTextString',
    'RosTimeVerbatim',
    'RosDurationVerbatim',
    'RosOpt',
    'RosNested',
    'RosList',
    'RosOpt',
    'create',
    'with_service_schemas'
]
