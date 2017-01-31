from __future__ import absolute_import
from __future__ import print_function

from .basic_fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString,
)

from .time_fields import (
    RosTime, RosTimeVerbatim,
    RosDuration, RosDurationVerbatim,
)

ros_msgtype_mapping = {
    'bool': RosBool,
    'int8': RosInt8, 'int16': RosInt16, 'int32': RosInt32, 'int64': RosInt64,
    'uint8': RosUInt8, 'uint16': RosUInt16, 'uint32': RosUInt32, 'uint64': RosUInt64,
    'float32': RosFloat32, 'float64': RosFloat64,
    # Note: both of these work for service response (check with ip callback)
    'string': RosString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
    #'string': RosTextString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
    # 'time': RosTime,
    'time': RosTimeVerbatim,
    # TODO : returning datetime as ISO string ?
    # 'duration': RosDuration,
    'duration': RosDurationVerbatim,
}