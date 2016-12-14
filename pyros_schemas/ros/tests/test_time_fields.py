from __future__ import absolute_import
from __future__ import print_function

import functools
import nose

try:
    import std_msgs
    import genpy
    import rospy
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us ot the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs
    import genpy
    import rospy


# absolute import ros field types
from pyros_schemas.ros.time_fields import (
    RosTimeVerbatim,
    RosTime,
    RosDurationVerbatim,
    RosDuration
)


#
# Test functions, called via test generator
#

@nose.tools.nottest
def fromrostime(ros_msg, FieldType, RosMsgType, PyType, PyTypeRos, Expected_Exceptions=()):

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
        # TODO : make this generic (checking slots), or have different functions to check type conversion for each type...
        # assert deserialized == PyType(secs=ros_msg.data.secs, nsecs=ros_msg.data.nsecs) # DOES NOT APPLY IF NOT VERBATIM
        serialized = field.serialize(0, [deserialized])

        # Check the field value we obtain is the same, both type and value.
        # NOPE DONT APPLY HERE assert isinstance(serialized, type(ros_msg.data))  # CAREFUL here with a hierachy of type (like pyros.Time() and genpy.Time())
        assert serialized == ros_msg.data
    except Expected_Exceptions:
        pass
    except Exception:
        raise
    else:
        # If no exception, make sure we weren't expecting any
        assert len(Expected_Exceptions) == 0


@nose.tools.nottest
def frompytime(py_inst, FieldType, RosMsgType, PyType, PyTypeRos, Expected_Exceptions=()):

    try:
        # verifying parameter type can at least convert (useful for unicode - str correspondance)
        # TODO : make this generic (checking slots), or have different functions to check type conversion for each type...
        # PyType(secs=py_inst.secs, nsecs=py_inst.nsecs)  # just try dynamic typing - python style - to make sure it wont except

        # verifying ros_msg type first (static typing - ROS style)
        assert isinstance(py_inst, PyType)  # TODO : not always true (see string...)

        field = FieldType()

        serialized = field.serialize(0, [py_inst])

        # Check the field value we obtain is the expected one and can be used to build a ROS message.
        assert isinstance(serialized, PyTypeRos)
        # assert serialized == py_inst  DOES NOT APPLY FOR NESTED SCHEMA

        ros_msg = RosMsgType(data=serialized)
        assert isinstance(ros_msg.data, PyTypeRos)
        # assert ros_msg.data == py_inst  DOES NOT APPLY FOR NESTED SCHEMA

        deserialized = field.deserialize(ros_msg.data)

        # check the serialized version is the type we expect
        assert isinstance(deserialized, PyType)
        # check the serialized value is the same as the original object
        # assert deserialized == py_inst  # DOES NOT APPLY FOR DEFAULT VALUES

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

def test_ros_field_time_verbatim():
    # Test all explicit values when possible, or pick a few meaningful ones
    yield fromrostime, std_msgs.msg.Time(rospy.Time(secs=42, nsecs=31)), RosTimeVerbatim, std_msgs.msg.Time, dict, rospy.Time
    # also test default value
    yield fromrostime, std_msgs.msg.Time(rospy.Time()), RosTimeVerbatim, std_msgs.msg.Time, dict, rospy.Time  # default

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompytime, dict(secs=42, nsecs=31), RosTimeVerbatim, std_msgs.msg.Time, dict, rospy.Time
    # also test (None ?) and default value
    yield frompytime, dict(), RosTimeVerbatim, std_msgs.msg.Time, dict, rospy.Time


def test_ros_field_time():
    # Test all explicit values when possible, or pick a few meaningful ones
    yield fromrostime, std_msgs.msg.Time(rospy.Time(secs=42, nsecs=31)), RosTime, std_msgs.msg.Time, float, rospy.Time
    # also test default value
    yield fromrostime, std_msgs.msg.Time(rospy.Time()), RosTime, std_msgs.msg.Time, float, rospy.Time

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompytime, 42.00000031, RosTime, std_msgs.msg.Time, float, rospy.Time


def test_ros_field_duration_verbatim():
    # Test all explicit values when possible, or pick a few meaningful ones
    yield fromrostime, std_msgs.msg.Duration(rospy.Duration(secs=42, nsecs=31)), RosDurationVerbatim, std_msgs.msg.Duration, dict, rospy.Duration
    # also test default value
    yield fromrostime, std_msgs.msg.Duration(rospy.Duration()), RosDurationVerbatim, std_msgs.msg.Duration, dict, rospy.Duration

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompytime, dict(secs=42, nsecs=31), RosDurationVerbatim, std_msgs.msg.Duration, dict, rospy.Duration
    # also test (None ?) and default value
    yield frompytime, dict(), RosDurationVerbatim, std_msgs.msg.Duration, dict, rospy.Duration


def test_ros_field_duration():
    # Test all explicit values when possible, or pick a few meaningful ones
    yield fromrostime, std_msgs.msg.Duration(rospy.Duration(secs=42, nsecs=31)), RosDuration, std_msgs.msg.Duration, float, rospy.Duration
    # also test default value
    yield fromrostime, std_msgs.msg.Duration(rospy.Duration()), RosDuration, std_msgs.msg.Duration, float, rospy.Duration

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompytime, 42.00000031, RosDuration, std_msgs.msg.Duration, float, rospy.Duration

# # Since the rospy message type member field is already a python int,
# # we do not need anything special here, we rely on marshmallow python type validation.
# # Yet we are specifying each on in case we want to extend it later...
#

# Just in case we run this directly
if __name__ == '__main__':
    nose.runmodule(__name__)
