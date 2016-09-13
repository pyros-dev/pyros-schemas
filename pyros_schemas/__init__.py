from __future__ import absolute_import
from __future__ import print_function

try:
    import std_msgs
    import pyros_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us ot the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs
    import pyros_msgs

from .decorators import wraps_cls, with_explicitly_matched_type, with_explicitly_matched_optional_type
from .std_Empty import RosMsgEmpty
from .std_Bool import RosFieldBool, RosMsgBool
from .std_Int import (
    RosFieldInt8, RosFieldInt16, RosFieldInt32, RosFieldInt64, RosFieldUInt8, RosFieldUInt16, RosFieldUInt32, RosFieldUInt64,
    RosMsgInt8, RosMsgInt16, RosMsgInt32, RosMsgInt64, RosMsgUInt8, RosMsgUInt16, RosMsgUInt32, RosMsgUInt64,
)
from .std_Float import RosFieldFloat32, RosFieldFloat64, RosMsgFloat32, RosMsgFloat64
from .std_String import RosFieldString, RosMsgString
from .std_Time import RosFieldTime, RosMsgTime
from .std_Duration import RosFieldDuration, RosMsgDuration

from .std_Header import RosMsgHeader

from .opt_Empty import PyrosMsgOptEmpty
from .opt_Bool import PyrosMsgOptBool

from .http_statuscode import RosMsgHttpStatusCode
__all__ = [
    'with_explicitly_matched_type',
    'with_explicitly_matched_optional_type',
    'RosMsgEmpty',
    'RosFieldBool', 'RosMsgBool',
    'RosFieldInt8', 'RosFieldInt16', 'RosFieldInt32', 'RosFieldInt64', 'RosFieldUInt8', 'RosFieldUInt16', 'RosFieldUInt32', 'RosFieldUInt64',
    'RosMsgInt8', 'RosMsgInt16', 'RosMsgInt32', 'RosMsgInt64', 'RosMsgUInt8', 'RosMsgUInt16', 'RosMsgUInt32', 'RosMsgUInt64',
    'RosFieldFloat32', 'RosFieldFloat64', 'RosMsgFloat32', 'RosMsgFloat64',
    'RosFieldString', 'RosMsgString',
    'RosFieldTime', 'RosMsgTime',
    'RosFieldDuration', 'RosMsgDuration',
    'RosMsgHeader',

    'PyrosMsgOptEmpty',
    'PyrosMsgOptBool',

    'RosMsgHttpStatusCode',
]
