from __future__ import absolute_import
from __future__ import print_function

import functools
import nose
import pytest
import marshmallow.utils

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


# generating all and accessing the required message class.
from pyros_schemas.ros.tests import msg_generate

try:
    test_gen_msgs, gen_test_srvs = msg_generate.generate_test_msgs()
except Exception as e:
    pytest.raises(e)


import hypothesis
import hypothesis.strategies


import pyros_msgs.opt_as_array
# patching (need to know the field name)
pyros_msgs.opt_as_array.duck_punch(test_gen_msgs.test_opt_bool_as_array, ['data'])


# absolute import ros field types
from pyros_schemas.ros.basic_fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString, RosTextString,
)
from pyros_schemas.ros.optional_fields import (
    RosOptAsList,
    RosOptAsNested,  # TODO : optional message as Nested
)


#
# Test functions, called via test generator
#

@nose.tools.nottest
def fromrosopt(ros_msg, FieldType, RosMsgType, PyType, PyTypeRos, Expected_Exceptions=()):

    try:
        # verifying ros_msg type first (static typing - ROS style)
        assert isinstance(ros_msg, RosMsgType)

        field = FieldType()

        assert hasattr(ros_msg, 'data')
        assert isinstance(ros_msg.data, PyTypeRos)

        deserialized = field.deserialize(ros_msg.data)

        # check the serialized version is the type we expect (or a missing optional field)
        assert isinstance(deserialized, PyType) or deserialized == marshmallow.utils.missing
        if deserialized != marshmallow.utils.missing:
            # check the serialized value is the same as the value of that field in the original message
            # We need the type conversion to deal with serialized object in different format than ros data (like string)
            assert deserialized == ros_msg.data[0]  # This is another (type dependent) way to deserialize
            serialized = field.serialize(0, [deserialized])

            # Check the field value we obtain is the same, both type and value.
            assert isinstance(serialized, type(ros_msg.data))  # This is another (type dependent) way to serialize
            assert serialized == ros_msg.data

    except Expected_Exceptions:
        pass
    except Exception:
        raise
    else:
        # If no exception, make sure we weren't expecting any
        assert len(Expected_Exceptions) == 0


@nose.tools.nottest
def frompyopt(py_inst, FieldType, RosMsgType, PyType, PyTypeRos, Expected_Exceptions=()):

    try:
        # verifying parameter type can at least convert (useful for unicode - str correspondance)
        # TODO : make this generic (checking slots), or have different functions to check type conversion for each type...
        # PyType(secs=py_inst.secs, nsecs=py_inst.nsecs)  # just try dynamic typing - python style - to make sure it wont except

        # verifying ros_msg type first (static typing - ROS style)
        # assert isinstance(py_inst, PyType)  # TODO : not always true (see string...)

        field = FieldType()

        serialized = field.serialize(0, [py_inst])

        # Check the field value we obtain is the expected one and can be used to build a ROS message.
        assert isinstance(serialized, PyTypeRos)
        if serialized != []:
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


def test_ros_field_opt_bool():
    # Test all explicit values when possible, or pick a few meaningful ones
    yield fromrosopt, test_gen_msgs.test_opt_bool_as_array(data=True), lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    yield fromrosopt, test_gen_msgs.test_opt_bool_as_array(data=False), lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    yield fromrosopt, test_gen_msgs.test_opt_bool_as_array(data=[True]), lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    yield fromrosopt, test_gen_msgs.test_opt_bool_as_array(data=[False]), lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    # also test [], None and default value
    yield fromrosopt, test_gen_msgs.test_opt_bool_as_array(data=[]), lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    yield fromrosopt, test_gen_msgs.test_opt_bool_as_array(data=None), lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    yield fromrosopt, test_gen_msgs.test_opt_bool_as_array(), lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompyopt, True, lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    yield frompyopt, False, lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    # also test [], None and default value
    yield frompyopt, [], lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    yield frompyopt, None, lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list
    # careful : bool() defaults to False
    yield frompyopt, bool(), lambda: RosOptAsList(RosBool()), test_gen_msgs.test_opt_bool_as_array, bool, list


# TODO : all of the field types...

# # Since the rospy message type member field is already a python int,
# # we do not need anything special here, we rely on marshmallow python type validation.
# # Yet we are specifying each on in case we want to extend it later...
#

# Just in case we run this directly
if __name__ == '__main__':
    nose.runmodule(__name__)
