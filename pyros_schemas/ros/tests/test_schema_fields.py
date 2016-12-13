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
from pyros_schemas.ros.schema_fields import (
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

        serialized = field.serialize('data', ros_msg)

        # check the serialized version is the type we expect
        assert isinstance(serialized, PyType)
        # check the serialized value is the same as the value of that field in the original message
        # We need the type conversion to deal with serialized object in different format than ros data (like string)
        # TODO : make this generic (checking slots), or have different functions to check type conversion for each type...
        assert serialized == PyType(secs=ros_msg.data.secs, nsecs=ros_msg.data.nsecs)
        deserialized = field.deserialize(serialized)

        # Check the field value we obtain is the same, both type and value.
        assert isinstance(deserialized, type(ros_msg.data))  # CAREFUL here with a hierachy of type (like pyros.Time() and genpy.Time())
        assert deserialized == ros_msg.data
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
        PyType(secs=py_inst.secs, nsecs=py_inst.nsecs)  # just try dynamic typing - python style - to make sure it wont except

        field = FieldType()

        deserialized = field.deserialize(py_inst)

        # Check the field value we obtain is the expected one and can be used to build a ROS message.
        assert isinstance(deserialized, PyTypeRos)
        assert deserialized == py_inst

        ros_msg = RosMsgType(data=deserialized)
        assert isinstance(ros_msg.data, PyTypeRos)
        assert ros_msg.data == py_inst

        serialized = field.serialize('data', ros_msg)

        # check the serialized version is the type we expect
        assert isinstance(serialized, PyType)
        # check the serialized value is the same as the original object
        assert serialized == py_inst

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
    yield fromrostime, std_msgs.msg.Time(genpy.Time(secs=42, nsecs=31)), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, genpy.Time  # default type, same as default constructor for msg
    yield fromrostime, std_msgs.msg.Time(rospy.Time(secs=42, nsecs=31)), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, rospy.Time  # subtype, likely to be used as well
    # also test default value
    yield fromrostime, std_msgs.msg.Time(), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, genpy.Time

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompytime, genpy.Time(secs=42, nsecs=31), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, genpy.Time  # default type, same as default constructor for msg
    yield frompytime, rospy.Time(secs=42, nsecs=31), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, rospy.Time  # subtype, likely to be used as well
    # also test (None ?) and default value
    yield frompytime, genpy.Time(), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, genpy.Time
    yield frompytime, rospy.Time(), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, rospy.Time



# # Since the rospy message type member field is already a python int,
# # we do not need anything special here, we rely on marshmallow python type validation.
# # Yet we are specifying each on in case we want to extend it later...
#

# Just in case we run this directly
if __name__ == '__main__':
    nose.runmodule(__name__)
