from __future__ import absolute_import, division, print_function

import pytest

try:
    import std_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us ot the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs


# absolute import ros field types
from pyros_schemas.ros.basic_fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString, RosTextString,
)


#
# Test functions, called via test generator
#

def fromros(ros_msg, FieldType, RosMsgType, PyType, PyTypeRos, Expected_Exceptions=()):

    try:
        # verifying ros_msg type first (static typing - ROS style)
        assert isinstance(ros_msg, RosMsgType)

        field = FieldType()

        assert hasattr(ros_msg, 'data')
        assert isinstance(ros_msg.data, PyTypeRos)

        deserialized = field.deserialize(ros_msg.data)

        # check the serialized version is the type we expect
        assert isinstance(deserialized, PyType)
        # check the serialized value is the same as the value of that field in the original message
        # We need the type conversion to deal with serialized object in different format than ros data (like string)
        assert deserialized == PyType(ros_msg.data)
        serialized = field.serialize(0, [deserialized])

        # Check the field value we obtain is the same, both type and value.
        assert isinstance(serialized, type(ros_msg.data))
        assert serialized == ros_msg.data
    except Expected_Exceptions:
        pass
    except Exception:
        raise
    else:
        # If no exception, make sure we weren't expecting any
        assert len(Expected_Exceptions) == 0


def frompy(py_inst, FieldType, RosMsgType, PyType, PyTypeRos, Expected_Exceptions=()):

    try:
        # verifying parameter type can at least convert (useful for unicode - str correspondance)
        PyType(py_inst)  # just try dynamic typing - python style - to make sure it wont except

        field = FieldType()

        serialized = field.serialize(0, [py_inst])

        # Check the field value we obtain is the expected one and can be used to build a ROS message.
        assert isinstance(serialized, PyTypeRos)
        assert serialized == py_inst

        ros_msg = RosMsgType(data=serialized)
        assert isinstance(ros_msg.data, PyTypeRos)
        assert ros_msg.data == py_inst

        deserialized = field.deserialize(ros_msg.data)

        # check the serialized version is the type we expect
        assert isinstance(deserialized, PyType)
        # check the serialized value is the same as the original object
        assert deserialized == py_inst

    except Expected_Exceptions:
        pass
    except Exception:
        raise
    else:
        # If no exception, make sure we weren't expecting any
        assert len(Expected_Exceptions) == 0

#
# Testing each ros field type
#

def test_ros_field_bool():
    # Test all explicit values when possible, or pick a few meaningful ones
    yield fromros, std_msgs.msg.Bool(data=True), RosBool, std_msgs.msg.Bool, bool, bool
    yield fromros, std_msgs.msg.Bool(data=False), RosBool, std_msgs.msg.Bool, bool, bool
    # also test None and default value
    yield fromros, std_msgs.msg.Bool(data=None), RosBool, std_msgs.msg.Bool, bool, bool
    yield fromros, std_msgs.msg.Bool(), RosBool, std_msgs.msg.Bool, bool, bool

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, True, RosBool, std_msgs.msg.Bool, bool, bool
    yield frompy, False, RosBool, std_msgs.msg.Bool, bool, bool
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosBool, std_msgs.msg.Bool, bool, bool
    yield frompy, bool(), RosBool, std_msgs.msg.Bool, bool, bool


def test_ros_field_int8():
    yield fromros, std_msgs.msg.Int8(data=42), RosInt8, std_msgs.msg.Int8, int, int
    # also test None and default value
    yield fromros, std_msgs.msg.Int8(data=None), RosInt8, std_msgs.msg.Int8, int, int
    yield fromros, std_msgs.msg.Int8(), RosInt8, std_msgs.msg.Int8, int, int

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42, RosInt8, std_msgs.msg.Int8, int, int
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosInt8, std_msgs.msg.Int8, int, int
    yield frompy, int(), RosInt8, std_msgs.msg.Int8, int, int


def test_ros_field_int16():
    yield fromros, std_msgs.msg.Int16(data=42), RosInt16, std_msgs.msg.Int16, int, int
    # also test None and default value
    yield fromros, std_msgs.msg.Int16(data=None), RosInt16, std_msgs.msg.Int16, int, int
    yield fromros, std_msgs.msg.Int16(), RosInt16, std_msgs.msg.Int16, int, int

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42, RosInt16, std_msgs.msg.Int16, int, int
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosInt16, std_msgs.msg.Int16, int, int
    yield frompy, int(), RosInt16, std_msgs.msg.Int16, int, int


def test_ros_field_int32():
    yield fromros, std_msgs.msg.Int32(data=42), RosInt32, std_msgs.msg.Int32, int, int
    # also test None and default value
    yield fromros, std_msgs.msg.Int32(data=None), RosInt32, std_msgs.msg.Int32, int, int
    yield fromros, std_msgs.msg.Int32(), RosInt32, std_msgs.msg.Int32, int, int

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42, RosInt32, std_msgs.msg.Int32, int, int
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosInt32, std_msgs.msg.Int32, int, int
    yield frompy, int(), RosInt32, std_msgs.msg.Int32, int, int


def test_ros_field_int64():
    yield fromros, std_msgs.msg.Int64(data=42), RosInt64, std_msgs.msg.Int64, int, int
    # also test None and default value
    yield fromros, std_msgs.msg.Int64(data=None), RosInt64, std_msgs.msg.Int64, int, int
    yield fromros, std_msgs.msg.Int64(), RosInt64, std_msgs.msg.Int64, int, int

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42, RosInt64, std_msgs.msg.Int64, int, int
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosInt64, std_msgs.msg.Int64, int, int
    yield frompy, int(), RosInt64, std_msgs.msg.Int64, int, int


def test_ros_field_uint8():
    yield fromros, std_msgs.msg.UInt8(data=42), RosUInt8, std_msgs.msg.UInt8, int, int
    # also test None and default value
    yield fromros, std_msgs.msg.UInt8(data=None), RosUInt8, std_msgs.msg.UInt8, int, int
    yield fromros, std_msgs.msg.UInt8(), RosUInt8, std_msgs.msg.UInt8, int, int

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42, RosUInt8, std_msgs.msg.UInt8, int, int
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosUInt8, std_msgs.msg.UInt8, int, int
    yield frompy, int(), RosUInt8, std_msgs.msg.UInt8, int, int


def test_ros_field_uint16():
    yield fromros, std_msgs.msg.UInt16(data=42), RosUInt16, std_msgs.msg.UInt16, int, int
    # also test None and default value
    yield fromros, std_msgs.msg.UInt16(data=None), RosUInt16, std_msgs.msg.UInt16, int, int
    yield fromros, std_msgs.msg.UInt16(), RosUInt16, std_msgs.msg.UInt16, int, int

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42, RosUInt16, std_msgs.msg.UInt16, int, int
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosInt16, std_msgs.msg.UInt16, int, int
    yield frompy, int(), RosUInt16, std_msgs.msg.UInt16, int, int


def test_ros_field_uint32():
    yield fromros, std_msgs.msg.UInt32(data=42), RosUInt32, std_msgs.msg.UInt32, int, int
    # also test None and default value
    yield fromros, std_msgs.msg.UInt32(data=None), RosUInt32, std_msgs.msg.UInt32, int, int
    yield fromros, std_msgs.msg.UInt32(), RosUInt32, std_msgs.msg.UInt32, int, int

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42, RosUInt32, std_msgs.msg.UInt32, int, int
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosInt32, std_msgs.msg.UInt32, int, int
    yield frompy, int(), RosUInt32, std_msgs.msg.UInt32, int, int


def test_ros_field_uint64():
    yield fromros, std_msgs.msg.UInt64(data=42), RosUInt64, std_msgs.msg.UInt64, int, int
    # also test None and default value
    yield fromros, std_msgs.msg.UInt64(data=None), RosUInt64, std_msgs.msg.UInt64, int, int
    yield fromros, std_msgs.msg.UInt64(), RosUInt64, std_msgs.msg.UInt64, int, int

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42, RosUInt64, std_msgs.msg.UInt64, int, int
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosUInt64, std_msgs.msg.UInt64, int, int
    yield frompy, int(), RosUInt64, std_msgs.msg.UInt64, int, int


def test_ros_field_float32():
    yield fromros, std_msgs.msg.Float32(data=42.), RosFloat32, std_msgs.msg.Float32, float, float
    # also test None and default value
    yield fromros, std_msgs.msg.Float32(data=None), RosFloat32, std_msgs.msg.Float32, float, float
    yield fromros, std_msgs.msg.Float32(), RosFloat32, std_msgs.msg.Float32, float, float

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42., RosFloat32, std_msgs.msg.Float32, float, float
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosFloat32, std_msgs.msg.Float32, float, float
    yield frompy, float(), RosFloat32, std_msgs.msg.Float32, float, float


def test_ros_field_float64():
    yield fromros, std_msgs.msg.Float64(data=42.), RosFloat64, std_msgs.msg.Float64, float, float
    # also test None and default value
    yield fromros, std_msgs.msg.Float64(data=None), RosFloat64, std_msgs.msg.Float64, float, float
    yield fromros, std_msgs.msg.Float64(), RosFloat64, std_msgs.msg.Float64, float, float

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 42., RosFloat64, std_msgs.msg.Float64, float, float
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosFloat64, std_msgs.msg.Float64, float, float
    yield frompy, float(), RosFloat64, std_msgs.msg.Float64, float, float


def test_ros_field_string():
    yield fromros, std_msgs.msg.String(data='fortytwo'), RosString, std_msgs.msg.String, str, str
    # this should except since Ros string field accepts unicode as data without validation,
    # but then something will break later on...
    yield fromros, std_msgs.msg.String(data=u'fortytwo'), RosString, std_msgs.msg.String, str, str, (AssertionError,)
    # also test None and default value
    yield fromros, std_msgs.msg.String(data=None), RosString, std_msgs.msg.String, str, str
    yield fromros, std_msgs.msg.String(), RosString, std_msgs.msg.String, str, str

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 'fortytwo', RosString, std_msgs.msg.String, str, str
    # Schema will encode unicode string.
    yield frompy, u'fortytwo', RosString, std_msgs.msg.String, str, str
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosString, std_msgs.msg.String, str, str
    yield frompy, str(), RosString, std_msgs.msg.String, str, str
    # CAREFUL this breaks on python 3
    yield frompy, unicode(), RosString, std_msgs.msg.String, str, str


def test_ros_field_textstring():
    yield fromros, std_msgs.msg.String(data='fortytwo'), RosTextString, std_msgs.msg.String, unicode, str
    # this should except since Ros string field accepts unicode as data without validation,
    # but then something will break later on...
    yield fromros, std_msgs.msg.String(data=u'fortytwo'), RosTextString, std_msgs.msg.String, unicode, str, (AssertionError,)
    # also test None and default value
    yield fromros, std_msgs.msg.String(data=None), RosTextString, std_msgs.msg.String, unicode, str
    yield fromros, std_msgs.msg.String(), RosTextString, std_msgs.msg.String, unicode, str

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 'fortytwo', RosTextString, std_msgs.msg.String, unicode, str
    yield frompy, u'fortytwo', RosTextString, std_msgs.msg.String, unicode, str
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosString, std_msgs.msg.String, unicode, str
    yield frompy, str(), RosTextString, std_msgs.msg.String, unicode, str
    yield frompy, unicode(), RosTextString, std_msgs.msg.String, unicode, str

# # Since the rospy message type member field is already a python int,
# # we do not need anything special here, we rely on marshmallow python type validation.
# # Yet we are specifying each on in case we want to extend it later...
#

# Just in case we run this directly
if __name__ == '__main__':
    pytest.main(['-s', '-x', __file__])
