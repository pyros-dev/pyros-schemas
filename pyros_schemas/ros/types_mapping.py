from __future__ import absolute_import
from __future__ import print_function

import sys

# for py2 / py3 compatibility
import six
six_long = six.integer_types[-1]


from .basic_fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString,
    RosTime,
    RosDuration,
)

# from .time_fields import (
#     # RosTimeVerbatim,
#     RosDurationVerbatim,
# )

ros_msgtype_mapping = {
    'bool': RosBool,
    'int8': RosInt8, 'int16': RosInt16, 'int32': RosInt32, 'int64': RosInt64,
    'uint8': RosUInt8, 'uint16': RosUInt16, 'uint32': RosUInt32, 'uint64': RosUInt64,
    'float32': RosFloat32, 'float64': RosFloat64,
    # Note: both of these work for service response (check with ip callback)
    'string': RosString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
    #'string': RosTextString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
    'time': RosTime,
    #'time': RosTimeVerbatim,
    # TODO : returning datetime as ISO string ?
    'duration': RosDuration,
    #'duration': RosDurationVerbatim,
}


# This is for explicit matching types.

# Ref : http://wiki.ros.org/msg
# TODO : arrays ?
ros_pythontype_mapping = {
    'bool': bool,
    'int8': int, 'int16': int, 'int32': int, 'int64': six_long,
    'uint8': int, 'uint16': int, 'uint32': int, 'uint64': six_long,
    'float32': float, 'float64': float,
    'string': str,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
    #'string': RosTextString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
    # Time ???
}
