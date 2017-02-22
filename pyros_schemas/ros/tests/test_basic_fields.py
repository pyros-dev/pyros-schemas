from __future__ import absolute_import
from __future__ import print_function

import pytest

# for py2 / py3 compatibility
import six
six_long = six.integer_types[-1]

# since we do fancy stuff to check for types
from types import NoneType


try:
    import std_msgs
    import genpy
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs
    import genpy


# absolute import ros field types
from pyros_schemas.ros.basic_fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString, RosTextString,
)

from pyros_schemas.ros.time_fields import (
    RosTimeVerbatim, RosTime,
    RosDurationVerbatim, RosDuration,
)

from pyros_schemas.ros.types_mapping import (
    ros_msgtype_mapping,
    ros_pythontype_mapping
)

#
# Test functions, called via test generator
#
@pytest.mark.parametrize("msg, schema_field_type, in_rosfield_pytype, dictfield_pytype, out_rosfield_pytype", [
    # Bool
    (std_msgs.msg.Bool(data=True), RosBool, bool, bool, bool),
    (std_msgs.msg.Bool(data=False), RosBool, bool, bool, bool),
    (std_msgs.msg.Bool(data=None), RosBool, bool, bool, bool),  # ROS will set a default of False for that data
    (std_msgs.msg.Bool(), RosBool, bool, bool, bool),  # ROS will set a default of False for that data
    # Int8
    (std_msgs.msg.Int8(data=42), RosInt8, int, int, int),
    (std_msgs.msg.Int8(data=-23), RosInt8, int, int, int),
    (std_msgs.msg.Int8(data=0), RosInt8, int, int, int),
    (std_msgs.msg.Int8(data=None), RosInt8, int, int, int),  # ROS will set a default of 0 for that data
    (std_msgs.msg.Int8(), RosInt8, int, int, int),  # ROS will set a default of 0 for that data
    # Int16
    (std_msgs.msg.Int16(data=42), RosInt16, int, int, int),
    (std_msgs.msg.Int16(data=-23), RosInt16, int, int, int),
    (std_msgs.msg.Int16(data=0), RosInt16, int, int, int),
    (std_msgs.msg.Int16(data=None), RosInt16, int, int, int),  # ROS will set a default of 0 for that data
    (std_msgs.msg.Int16(), RosInt16, int, int, int),  # ROS will set a default of 0 for that data
    # Int32
    (std_msgs.msg.Int32(data=42), RosInt32, int, int, int),
    (std_msgs.msg.Int32(data=-23), RosInt32, int, int, int),
    (std_msgs.msg.Int32(data=0), RosInt32, int, int, int),
    (std_msgs.msg.Int32(data=None), RosInt32, int, int, int),  # ROS will set a default of 0 for that data
    (std_msgs.msg.Int32(), RosInt32, int, int, int),  # ROS will set a default of 0 for that data
    # Int64
    # Careful ROS doc says Int64 is long, but it is completely dynamic
    (std_msgs.msg.Int64(data=42), RosInt64, int, six_long, six_long),
    (std_msgs.msg.Int64(data=-23), RosInt64, int, six_long, six_long),
    (std_msgs.msg.Int64(data=42424242424242424242), RosInt64, six_long, six_long, six_long),
    (std_msgs.msg.Int64(data=-23232323232323232323), RosInt64, six_long, six_long, six_long),
    (std_msgs.msg.Int64(data=0), RosInt64, int, six_long, six_long),
    (std_msgs.msg.Int64(data=None), RosInt64, int, six_long, six_long),  # ROS will set a default of 0 for that data but it will come out as 0L
    (std_msgs.msg.Int64(), RosInt64, int, six_long, six_long),  # ROS will set a default of 0 for that data but it will come out as 0L
    # UInt8
    (std_msgs.msg.UInt8(data=42), RosUInt8, int, int, int),
    # Careful : negative integer are accepted by ROS in UInt fields
    (std_msgs.msg.UInt8(data=-23), RosUInt8, int, int, int),
    (std_msgs.msg.UInt8(data=0), RosUInt8, int, int, int),
    (std_msgs.msg.UInt8(data=None), RosUInt8, int, int, int),  # ROS will set a default of 0 for that data
    (std_msgs.msg.UInt8(), RosUInt8, int, int, int),  # ROS will set a default of 0 for that data
    # UInt16
    (std_msgs.msg.UInt16(data=42), RosUInt16, int, int, int),
    # Careful : negative integer are accepted by ROS in UInt fields
    (std_msgs.msg.UInt16(data=-23), RosUInt16, int, int, int),
    (std_msgs.msg.UInt16(data=0), RosUInt16,  int, int, int),
    (std_msgs.msg.UInt16(data=None), RosUInt16, int, int, int),  # ROS will set a default of 0 for that data
    (std_msgs.msg.UInt16(), RosUInt16, int, int, int),  # ROS will set a default of 0 for that data
    # UInt32
    (std_msgs.msg.UInt32(data=42), RosUInt32, int, int, int),
    # Careful : negative integer are accepted by ROS in UInt fields
    (std_msgs.msg.UInt32(data=-23), RosUInt32, int, int, int),
    (std_msgs.msg.UInt32(data=0), RosUInt32, int, int, int),
    (std_msgs.msg.UInt32(data=None), RosUInt32, int, int, int),  # ROS will set a default of 0 for that data
    (std_msgs.msg.UInt32(), RosUInt32, int, int, int),  # ROS will set a default of 0 for that data
    # UInt64
    # Careful ROS doc says Int64 is long, but it is completely dynamic
    (std_msgs.msg.UInt64(data=42), RosUInt64, int, six_long, six_long),
    (std_msgs.msg.UInt64(data=-23), RosUInt64, int, six_long, six_long),
    (std_msgs.msg.UInt64(data=42424242424242424242), RosUInt64, six_long, six_long, six_long),
    (std_msgs.msg.UInt64(data=-23232323232323232323), RosUInt64, six_long, six_long, six_long),
    (std_msgs.msg.UInt64(data=0), RosUInt64, int, six_long, six_long),
    (std_msgs.msg.UInt64(data=None), RosUInt64, int, six_long, six_long),  # ROS will set a default of 0 for that data but it will come out as 0L
    (std_msgs.msg.UInt64(), RosUInt64, int, six_long, six_long),  # ROS will set a default of 0 for that data but it will come out as 0L
    # Float32
    (std_msgs.msg.Float32(data=42.), RosFloat32, float, float, float),
    (std_msgs.msg.Float32(data=0.), RosFloat32, float, float, float),
    (std_msgs.msg.Float32(data=None), RosFloat32, float, float, float),  # ROS will set a default of 0.0 for that data
    (std_msgs.msg.Float32(), RosFloat32, float, float, float),  # ROS will set a default of 0.0 for that data
    # Float64
    (std_msgs.msg.Float64(data=42.), RosFloat64, float, float, float),
    (std_msgs.msg.Float64(data=0.), RosFloat64, float, float, float),
    (std_msgs.msg.Float64(data=None), RosFloat64, float, float, float),  # ROS will set a default of 0.0 for that data
    (std_msgs.msg.Float64(), RosFloat64, float, float, float),  # ROS will set a default of 0.0 for that data
    # String
    (std_msgs.msg.String(data='fortytwo'), RosString, str, str, str),
    (std_msgs.msg.String(data=''), RosString, str, str, str),
    # Ros string field accepts unicode as data without validation, but then something will break in ROS later on...
    (std_msgs.msg.String(data=u'fortytwo'), RosString, unicode, str, str),
    (std_msgs.msg.String(data=u''), RosString, unicode, str, str),
    (std_msgs.msg.String(data=None), RosString, str, str, str),  # ROS will set a default of '' for that data
    (std_msgs.msg.String(), RosString, str, str, str),  # ROS will set a default of '' for that data
    # TextString
    (std_msgs.msg.String(data='fortytwo'), RosTextString, str, unicode, str),
    (std_msgs.msg.String(data=''), RosTextString, str, unicode, str),
    # Ros string field accepts unicode as data without validation, but then something will break in ROS later on...
    (std_msgs.msg.String(data=u'fortytwo'), RosTextString, unicode, unicode, str),
    (std_msgs.msg.String(data=u''), RosTextString, unicode, unicode, str),
    (std_msgs.msg.String(data=None), RosTextString, str, unicode, str),  # ROS will set a default of '' for that data
    (std_msgs.msg.String(), RosTextString, str, unicode, str),  # ROS will set a default of '' for that data
    # TimeVerbatim
    (std_msgs.msg.Time(genpy.Time(secs=42, nsecs=31)), RosTimeVerbatim, genpy.Time, dict, genpy.Time),
    # Raises TypeError Reason: ROS checks for time values to be positive
    # (std_msgs.msg.Time(genpy.Time(secs=-23, nsecs=31)), RosTimeVerbatim, genpy.Time, dict, genpy.Time),
    (std_msgs.msg.Time(genpy.Time(secs=0, nsecs=0)), RosTimeVerbatim, genpy.Time, dict, genpy.Time),
    (std_msgs.msg.Time(genpy.Time()), RosTimeVerbatim, genpy.Time, dict, genpy.Time),
    # Time
    (std_msgs.msg.Time(genpy.Time(secs=42, nsecs=31)), RosTime, genpy.Time, float, genpy.Time),
    # Raises TypeError Reason: ROS checks for time values to be positive
    # (std_msgs.msg.Time(genpy.Time(secs=-23, nsecs=31)), RosTime, genpy.Time, float, genpy.Time),
    (std_msgs.msg.Time(genpy.Time(secs=0, nsecs=0)), RosTime, genpy.Time, float, genpy.Time),
    (std_msgs.msg.Time(genpy.Time()), RosTime, genpy.Time, float, genpy.Time),
    # DurationVErbatim
    (std_msgs.msg.Duration(genpy.Duration(secs=42, nsecs=31)), RosDurationVerbatim, genpy.Duration, dict, genpy.Duration),
    (std_msgs.msg.Duration(genpy.Duration(secs=-23, nsecs=31)), RosDurationVerbatim, genpy.Duration, dict, genpy.Duration),
    (std_msgs.msg.Duration(genpy.Duration(secs=0, nsecs=0)), RosDurationVerbatim, genpy.Duration, dict, genpy.Duration),
    (std_msgs.msg.Duration(genpy.Duration()), RosDurationVerbatim, genpy.Duration, dict, genpy.Duration),
    # Duration
    (std_msgs.msg.Duration(genpy.Duration(secs=42, nsecs=31)), RosDuration, genpy.Duration, float, genpy.Duration),
    (std_msgs.msg.Duration(genpy.Duration(secs=-23, nsecs=31)), RosDuration, genpy.Duration, float, genpy.Duration),
    (std_msgs.msg.Duration(genpy.Duration(secs=0, nsecs=0)), RosDuration, genpy.Duration, float, genpy.Duration),
    (std_msgs.msg.Duration(genpy.Duration()), RosDuration, genpy.Duration, float, genpy.Duration),

])
def test_fromros(msg, schema_field_type, in_rosfield_pytype, dictfield_pytype, out_rosfield_pytype):
    """
    Checking deserialization/serialization from a rosmsg
    :param msg: the message
    :param schema_field_type: the field type for that message 'data' field
    :param rosmsg_type: the actual rosmsg type
    :param rosmsgfield_pytype:
    :param dictfield_pytype:
    :param rosfield_pytype:
    :return:
    """

    # Schemas' Field constructor
    field = schema_field_type()

    assert hasattr(msg, 'data')
    # Making sure the data msg field is of the intended pytype
    # in case ROS messages do - or dont do - some conversions
    assert isinstance(msg.data, in_rosfield_pytype)
    deserialized = field.deserialize(msg.data)

    # check the serialized version is the type we expect
    assert isinstance(deserialized, dictfield_pytype)
    # check the deserialized value is the same as the value of that field in the original message
    # We need the type conversion to deal with deserialized object in different format than ros data (like string)
    # we also need to deal with slots in case we have complex objects (only one level supported)
    if dictfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type]:
        if in_rosfield_pytype == genpy.rostime.Time or in_rosfield_pytype == genpy.rostime.Duration:  # non verbatim basic fields
            assert deserialized == dictfield_pytype(msg.data.to_sec())
        else:
            assert deserialized == dictfield_pytype(msg.data)
    else:  # not a basic type for python (slots should be there though...)
        assert deserialized == dictfield_pytype([(s, getattr(msg.data, s)) for s in msg.data.__slots__])

    serialized = field.serialize(0, [deserialized])

    # Check the field value we obtain is the expected ros type and same value.
    assert isinstance(serialized, out_rosfield_pytype)
    assert serialized == msg.data


@pytest.mark.parametrize("pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype", [
    # Bool
    (True, RosBool, std_msgs.msg.Bool, bool, bool),
    (False, RosBool, std_msgs.msg.Bool, bool, bool),
    (bool(), RosBool, std_msgs.msg.Bool, bool, bool),  # will use default python value of dictfield_pytype()
    # Int8
    (42, RosInt8, std_msgs.msg.Int8, int, int),
    (-23, RosInt8, std_msgs.msg.Int8, int, int),
    (int(), RosInt8, std_msgs.msg.Int8, int, int),  # will use default python value of dictfield_pytype()
    # Int16
    (42, RosInt16, std_msgs.msg.Int16, int, int),
    (-23, RosInt16, std_msgs.msg.Int16, int, int),
    (int(), RosInt16, std_msgs.msg.Int16, int, int),  # will use default python value of dictfield_pytype()
    # Int32
    (42, RosInt32, std_msgs.msg.Int32, int, int),
    (-23, RosInt32, std_msgs.msg.Int32, int, int),
    (int(), RosInt32, std_msgs.msg.Int32, int, int),  # will use default python value of dictfield_pytype()
    # Int64
    # Careful ROS doc says Int64 is long, but it is completely dynamic
    (42, RosInt64, std_msgs.msg.Int64, six_long, six_long),
    (-23, RosInt64, std_msgs.msg.Int64, six_long, six_long),
    (42424242424242424242, RosInt64, std_msgs.msg.Int64, six_long, six_long),
    (-23232323232323232323, RosInt64, std_msgs.msg.Int64, six_long, six_long),
    (int(), RosInt64, std_msgs.msg.Int64, six_long, six_long),
    (six_long(), RosInt64, std_msgs.msg.Int64, six_long, six_long),  # will use default python value of dictfield_pytype()
    # UInt8
    (42, RosUInt8, std_msgs.msg.UInt8, int, int),
    # Careful : negative integer are accepted by ROS in UInt fields
    (-23, RosUInt8, std_msgs.msg.UInt8, int, int),
    (int(), RosUInt8, std_msgs.msg.UInt8, int, int),  # will use default python value of dictfield_pytype()
    # UInt16
    (42, RosUInt16, std_msgs.msg.UInt16, int, int),
    # Careful : negative integer are accepted by ROS in UInt fields
    (-23, RosUInt16, std_msgs.msg.UInt16, int, int),
    (int(), RosUInt16, std_msgs.msg.UInt16, int, int),  # will use default python value of dictfield_pytype()
    # UInt32
    (42, RosUInt32, std_msgs.msg.UInt32, int, int),
    # Careful : negative integer are accepted by ROS in UInt fields
    (-23, RosUInt32, std_msgs.msg.UInt32, int, int),
    (int(), RosUInt32, std_msgs.msg.UInt32, int, int),  # will use default python value of dictfield_pytype()
    # UInt64
    # Careful ROS doc says UInt64 is long, but it is completely dynamic
    (42, RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
    (-23, RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
    (42424242424242424242, RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
    (-23232323232323232323, RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
    (int(), RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
    (six_long(), RosUInt64, std_msgs.msg.UInt64, six_long, six_long),  # will use default python value of dictfield_pytype()
    # Float32
    (42., RosFloat32, std_msgs.msg.Float32, float, float),
    (float(), RosFloat32, std_msgs.msg.Float32, float, float),  # will use default python value of dictfield_pytype()
    # Float64
    (42., RosFloat64, std_msgs.msg.Float64, float, float),
    (float(), RosFloat64, std_msgs.msg.Float64, float, float),  # will use default python value of dictfield_pytype()
    # String
    ('fortytwo', RosString, std_msgs.msg.String, str, str),
    ('', RosString, std_msgs.msg.String, str, str),
    # Ros string field accepts unicode as data without validation, but then something will break in ROS later on...
    (u'fortytwo', RosString, std_msgs.msg.String, str, str),
    (u'', RosString, std_msgs.msg.String, str, str),
    # TextString
    ('fortytwo', RosTextString, std_msgs.msg.String, str, unicode),
    ('', RosTextString, std_msgs.msg.String, str, unicode),
    # Ros string field accepts unicode as data without validation, but then something will break in ROS later on...
    (u'fortytwo', RosTextString, std_msgs.msg.String, str, unicode),
    (u'', RosTextString, std_msgs.msg.String, str, unicode),
    # TimeVerbatim
    (dict(secs=42, nsecs=31), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, dict),
    pytest.mark.xfail((dict(secs=-23, nsecs=31), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, dict), strict=True, raises=TypeError, reason="ROS checks that times values are positive"),
    (dict(secs=0, nsecs=0), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, dict),
    (dict(), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, dict),
    # Time
    (42.00000031, RosTime, std_msgs.msg.Time, genpy.Time, float),
    pytest.mark.xfail((-23.00000031, RosTime, std_msgs.msg.Time, genpy.Time, float), strict=True, raises=TypeError, reason="ROS checks that times values are positive"),
    (0.0, RosTime, std_msgs.msg.Time, genpy.Time, float),
    # Duration Verbatim
    (dict(secs=42, nsecs=31), RosDurationVerbatim, std_msgs.msg.Duration, genpy.Duration, dict),
    (dict(secs=-23, nsecs=31), RosDurationVerbatim, std_msgs.msg.Duration, genpy.Duration, dict),
    (dict(secs=0, nsecs=0), RosDurationVerbatim, std_msgs.msg.Duration, genpy.Duration, dict),
    (dict(), RosDurationVerbatim, std_msgs.msg.Duration, genpy.Duration, dict),
    # Duration
    (42.00000031, RosDuration, std_msgs.msg.Duration, genpy.Duration, float),
    (-23.00000031, RosDuration, std_msgs.msg.Duration, genpy.Duration, float),
    (0.0, RosDuration, std_msgs.msg.Duration, genpy.Duration, float),

])
def test_frompy(pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype):

    # Schemas' Field constructor
    field = schema_field_type()

    serialized = field.serialize(0, [pyfield])

    # Check the serialized field is the type we expect.
    assert isinstance(serialized, rosfield_pytype)
    # check the serialized value is the same as the value of that field in the original message
    # We need the type conversion to deal with serialized object in different format than ros data (like string)
    # we also need to deal with slots in case we have complex objects (only one level supported)
    if rosfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type]:
        assert serialized == pyfield
    else:  # not a basic type for python
        if pyfield_pytype in [int, six_long, float]:  # non verbatim basic fields
            assert serialized == rosfield_pytype(secs=int(pyfield), nsecs=int(pyfield * 1e9 - int(pyfield) *1e9))
        else:  #dict format can be used though...
            assert serialized == rosfield_pytype(**pyfield)

    # Building the ros message in case it changes something...
    ros_msg = rosmsg_type(data=serialized)
    deserialized = field.deserialize(ros_msg.data)

    # Check the dict we obtain is the expected type and same value.
    assert isinstance(deserialized, pyfield_pytype)
    if pyfield_pytype not in [bool, int, six_long, float, six.binary_type, six.text_type]:
        # If we were missing some fields, we need to initialise to default ROS value to be able to compare
        for i, s in enumerate(ros_msg.data.__slots__):
            if s not in pyfield.keys():
                pyfield[s] = ros_pythontype_mapping[ros_msg.data._slot_types[i]]()

    assert deserialized == pyfield


# # Since the rospy message type member field is already a python int,
# # we do not need anything special here, we rely on marshmallow python type validation.
# # Yet we are specifying each on in case we want to extend it later...
#

# Just in case we run this directly
if __name__ == '__main__':
    pytest.main([
        'test_basic_fields.py::test_fromros',
        'test_basic_fields.py::test_frompy',
    ])
