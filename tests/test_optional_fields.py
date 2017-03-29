from __future__ import absolute_import, division, print_function, unicode_literals

import pytest

try:
    import std_msgs.msg as std_msgs
    import genpy
    import pyros_msgs.msg
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs.msg as std_msgs
    import genpy
    import pyros_msgs.msg

import six
import marshmallow
import hypothesis
import hypothesis.strategies as st

# absolute import ros field types
from pyros_schemas.ros import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString, RosTextString,
    RosList,
    RosTime,
    RosDuration,
)

# from pyros_schemas.ros.time_fields import (
#     # RosTime,
#     RosDuration,
# )

from pyros_schemas.ros.types_mapping import (
    ros_msgtype_mapping,
    ros_pythontype_mapping
)

from . import six_long, maybe_list, proper_basic_msg_strategy_selector, proper_basic_data_strategy_selector


# TODO : make that generic to be able to test any message type...
# Note : we do not want to test implicit python type conversion here (thats the job of pyros_msgs typechecker)
#: (schema_field_type, rosfield_pytype, dictfield_pytype)
std_msgs_types_data_schemas_rosopttype_pytype = {
    'pyros_msgs/test_opt_bool_as_array': (lambda: RosOptAsList(RosBool()), bool, bool),
    'pyros_msgs/test_opt_int8_as_array': (lambda: RosOptAsList(RosInt8()), int, int),
    'pyros_msgs/test_opt_int16_as_array': (lambda: RosOptAsList(RosInt16()), int, int),
    'pyros_msgs/test_opt_int32_as_array': (lambda: RosOptAsList(RosInt32()), int, int),
    'pyros_msgs/test_opt_int64_as_array': (lambda: RosOptAsList(RosInt64()), six_long, six_long),
    'pyros_msgs/test_opt_uint8_as_array': (lambda: RosOptAsList(RosUInt8()), int, int),
    'pyros_msgs/test_opt_uint16_as_array': (lambda: RosOptAsList(RosUInt16()), int, int),
    'pyros_msgs/test_opt_uint32_as_array': (lambda: RosOptAsList(RosUInt32()), int, int),
    'pyros_msgs/test_opt_uint64_as_array': (lambda: RosOptAsList(RosUInt64()), six_long, six_long),
    'pyros_msgs/test_opt_float32_as_array': (lambda: RosOptAsList(RosFloat32()), float, float),
    'pyros_msgs/test_opt_float64_as_array': (lambda: RosOptAsList(RosFloat64()), float, float),
    'pyros_msgs/test_opt_string_as_array': [(lambda: RosOptAsList(RosString()), six.binary_type, six.binary_type)],  #, (RosTextString, six.binary_type, six.text_type)],
    'pyros_msgs/test_opt_time_as_array': [(lambda: RosOptAsList(RosTime()), genpy.Time, six_long)],
    'pyros_msgs/test_opt_duration_as_array': [(lambda: RosOptAsList(RosDuration()), genpy.Duration, six_long)],
}


# We need a composite strategy to link slot type and slot value
@st.composite
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1)
def msg_rostype_and_value(draw, msgs_type_strat_tuples):
    msg_type_strat = draw(st.sampled_from(msgs_type_strat_tuples))
    msg_value = draw(msg_type_strat[1])
    return msg_type_strat[0], msg_value


@hypothesis.given(msg_rostype_and_value(proper_basic_msg_strategy_selector(
    'pyros_msgs/test_opt_bool_as_array',
    'pyros_msgs/test_opt_int8_as_array',
    'pyros_msgs/test_opt_int16_as_array',
    'pyros_msgs/test_opt_int32_as_array',
    'pyros_msgs/test_opt_int64_as_array',
    'pyros_msgs/test_opt_uint8_as_array',
    'pyros_msgs/test_opt_uint16_as_array',
    'pyros_msgs/test_opt_uint32_as_array',
    'pyros_msgs/test_opt_uint64_as_array',
    'pyros_msgs/test_opt_float32_as_array',
    'pyros_msgs/test_opt_float64_as_array',
    'pyros_msgs/test_opt_string_as_array',
    'pyros_msgs/test_opt_time_as_array',
    'pyros_msgs/test_opt_duration_as_array',
    #TODO : more of that...
)))
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose)
def test_field_deserialize_serialize_from_ros_inverse(msg_rostype_and_value):
    msg_type = msg_rostype_and_value[0]
    msg_value = msg_rostype_and_value[1]

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(std_msgs_types_data_schemas_rostype_pytype.get(msg_type)):
        schema_field_type, rosfield_pytype, dictfield_pytype = possible_interpretation

        # Schemas' Field constructor
        field = schema_field_type()

        assert hasattr(msg_value, 'data')
        deserialized = field.deserialize(msg_value.data)

        # check the serialized version is the type we expect
        assert isinstance(deserialized, dictfield_pytype)
        # check the deserialized value is the same as the value of that field in the original message
        # We need the type conversion to deal with deserialized object in different format than ros data (like string)
        # we also need to deal with slots in case we have complex objects (only one level supported)
        if dictfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type, list]:
            if rosfield_pytype == genpy.rostime.Time or rosfield_pytype == genpy.rostime.Duration:  # non verbatim basic fields
                # TODO : find a way to get rid of this special case...
                assert deserialized == dictfield_pytype(msg_value.data.to_sec())
            elif rosfield_pytype == list:  # TODO : improve this check
                # TODO : find a way to get rid of this special case...
                for idx, elem in enumerate(msg_value.data):
                    assert deserialized[idx] == elem
            else:
                assert deserialized == dictfield_pytype(msg_value.data)
        else:  # not a basic type for python (slots should be there though...)
            assert deserialized == dictfield_pytype([(s, getattr(msg_value.data, s)) for s in msg_value.data.__slots__])

        serialized = field.serialize(0, [deserialized])

        # Check the field value we obtain is the expected ros type and same value.
        assert isinstance(serialized, rosfield_pytype)
        assert serialized == msg_value.data


@hypothesis.given(msg_rostype_and_value(proper_basic_data_strategy_selector(
    'pyros_msgs/test_opt_bool_as_array',
    'pyros_msgs/test_opt_int8_as_array',
    'pyros_msgs/test_opt_int16_as_array',
    'pyros_msgs/test_opt_int32_as_array',
    'pyros_msgs/test_opt_int64_as_array',
    'pyros_msgs/test_opt_uint8_as_array',
    'pyros_msgs/test_opt_uint16_as_array',
    'pyros_msgs/test_opt_uint32_as_array',
    'pyros_msgs/test_opt_uint64_as_array',
    'pyros_msgs/test_opt_float32_as_array',
    'pyros_msgs/test_opt_float64_as_array',
    'pyros_msgs/test_opt_string_as_array',
    'pyros_msgs/test_opt_time_as_array',
    'pyros_msgs/test_opt_duration_as_array',
    #TODO : more of that...
)))
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose)
def test_field_serialize_deserialize_from_py_inverse(msg_rostype_and_value):
    # TODO : makeit clearer that we get different data here, even if we still use msg_rostype_and_value
    # Same values as for ros message test
    msg_type = msg_rostype_and_value[0]
    pyfield = msg_rostype_and_value[1]

    # get actual type from type string
    rosmsg_type = genpy.message.get_message_class(msg_type)

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(std_msgs_types_data_schemas_rostype_pytype.get(msg_type)):
        schema_field_type, rosfield_pytype, pyfield_pytype = possible_interpretation

        # test_frompy(pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype):

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
            if rosfield_pytype == genpy.Time or rosfield_pytype == genpy.Duration:
                # these are deserialized (deterministically) as basic types (long nsecs)
                # working around genpy.rostime abismal performance
                pyfield_s = pyfield // 1000000000
                pyfield_ns = pyfield - pyfield_s * 1000000000
                assert serialized == rosfield_pytype(secs=pyfield_s, nsecs=pyfield_ns)
            elif pyfield_pytype == list:
                for idx, elem in enumerate(pyfield):
                    assert serialized[idx] == elem
            else:  # dict format can be used for nested types though...
                assert serialized == rosfield_pytype(**pyfield)

        # Building the ros message in case it changes something...
        ros_msg = rosmsg_type(data=serialized)
        deserialized = field.deserialize(ros_msg.data)

        # Check the dict we obtain is the expected type and same value.
        assert isinstance(deserialized, pyfield_pytype)
        if pyfield_pytype not in [bool, int, six_long, float, six.binary_type, six.text_type, list]:
            # If we were missing some fields, we need to initialise to default ROS value to be able to compare
            for i, s in enumerate(ros_msg.data.__slots__):
                if s not in pyfield.keys():
                    pyfield[s] = ros_pythontype_mapping[ros_msg.data._slot_types[i]]()

        assert deserialized == pyfield

# Just in case we run this directly
if __name__ == '__main__':
    pytest.main([
        '-s',
        'test_basic_fields.py::test_field_deserialize_serialize_from_ros_inverse',
        'test_basic_fields.py::test_field_serialize_deserialize_from_py_inverse',
    ])





import pytest

# for py2 / py3 compatibility
import six
six_long = six.integer_types[-1]


try:
    import std_msgs
    import genpy
    import rospy
    import pyros_msgs.opt_as_array  # This will duck punch the standard message type initialization code.
    from pyros_msgs.msg import test_opt_bool_as_array  # a message type just for testing
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us ot the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs
    import genpy
    import rospy
    import pyros_msgs.opt_as_array  # This will duck punch the standard message type initialization code.
    from pyros_msgs.msg import test_opt_bool_as_array  # a message type just for testing
    from pyros_msgs.msg import test_opt_float32_as_array  # a message type just for testing
    # TODO : all of them

# patching
pyros_msgs.opt_as_array.duck_punch(test_opt_bool_as_array, ['data'])


import marshmallow.utils

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

from pyros_schemas.ros.types_mapping import (
    ros_msgtype_mapping,
    ros_pythontype_mapping
)


#
# Test functions, called via test generator
#

@pytest.mark.parametrize("msg, schema_field_type, in_rosfield_pytype, dictfield_pytype, out_rosfield_pytype", [
    # Bool
    # basic exmple : (std_msgs.msg.Bool(data=True), RosBool, bool, bool, bool),
    (test_opt_bool_as_array(data=[True]), lambda: RosOptAsList(RosBool()), list, bool, list),
    (test_opt_bool_as_array(data=[False]), lambda: RosOptAsList(RosBool()), list, bool, list),
    # also test [], None and default value
    (test_opt_bool_as_array(data=[]), lambda: RosOptAsList(RosBool()), list, bool, list),
    # Raises AttributeError Reason: pyros_msgs checks for data value to have proper type
    #pytest.mark.xfail(strict=True, raises=AttributeError, reason="None is not accepted as value for data")((test_opt_bool_as_array(data=None), lambda: RosOptAsList(RosBool()), list, bool, list)),
    (test_opt_bool_as_array(), lambda: RosOptAsList(RosBool()), list, bool, list),
])
def test_fromrosopt(msg, schema_field_type, in_rosfield_pytype, dictfield_pytype, out_rosfield_pytype):
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

    # check the deserialized version is the type we expect (or a missing optional field)
    assert isinstance(deserialized, dictfield_pytype) or deserialized == marshmallow.utils.missing
    if deserialized != marshmallow.utils.missing:
        # check the deserialized value is the same as the value of that field in the original message
        # We need the type conversion to deal with deserialized object in different format than ros data (like string)
        # we also need to deal with slots in case we have complex objects (only one level supported)
        if dictfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type]:
            if in_rosfield_pytype == genpy.rostime.Time or in_rosfield_pytype == genpy.rostime.Duration:  # non verbatim basic fields
                assert deserialized == dictfield_pytype(msg.data.to_sec())
            elif dictfield_pytype == list:  # TODO : improve this check
                # TODO : find a way to get rid of this special case...
                for idx, elem in enumerate(msg.data):
                    assert deserialized[idx] == elem
            else:
                assert deserialized == dictfield_pytype(msg.data[0])
        else:  # not a basic type for python (slots should be there though...)
            assert deserialized == dictfield_pytype([(s, getattr(msg.data, s)) for s in msg.data.__slots__])

        serialized = field.serialize(0, [deserialized])

        # Check the field value we obtain is the expected ros type and same value.
        assert isinstance(serialized, out_rosfield_pytype)
        assert serialized == msg.data


@pytest.mark.parametrize("pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype", [
    # Bool
    # basic exmple (True, RosBool, std_msgs.msg.Bool, bool, bool),
    (True, lambda: RosOptAsList(RosBool()), list, bool, list),
    (False, lambda: RosOptAsList(RosBool()), list, bool, list),
    # also test [], None and default value
    ([], lambda: RosOptAsList(RosBool()), list, bool, list),
    (None, lambda: RosOptAsList(RosBool()), list, bool, list),
    # careful : bool() defaults to False
    (bool(), lambda: RosOptAsList(RosBool()), list, bool, list),
])
def test_frompy(pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype):


