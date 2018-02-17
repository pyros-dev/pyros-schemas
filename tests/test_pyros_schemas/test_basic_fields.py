from __future__ import absolute_import, division, print_function, unicode_literals

import pytest

import std_msgs.msg as std_msgs
import genpy

import six
import marshmallow
import hypothesis
import hypothesis.strategies as st


from . import six_long, maybe_list

from .strategies.python import field_strat_ok
from .strategies.ros import std_msgs_types_strat_ok, rostype_from_rostypestring, fieldtypestring_from_rostypestring

# Some tests are still failing on python3, related to strings. see https://github.com/ros/genpy/pull/85 and https://github.com/ros/genpy/pull/90 for related discussion
# also https://discourse.ros.org/t/python3-and-strings/2392


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

# This is here as it is specific to this test of ROS <-> python conversion

# TODO : make that generic to be able to test any message type...
# Note : we do not want to test implicit python type conversion here (thats the job of pyros_msgs typechecker)
#: (schema_field_type, rosfield_pytype, dictfield_pytype)
field_types_data_schemas_rostype_pytype = {
    'bool': (RosBool, bool, bool),
    'int8': (RosInt8, int, int),
    'int16': (RosInt16, int, int),
    'int32': (RosInt32, int, int),
    'int64': (RosInt64, six_long, six_long),
    'uint8': (RosUInt8, int, int),
    'uint16': (RosUInt16, int, int),
    'uint32': (RosUInt32, int, int),
    'uint64': (RosUInt64, six_long, six_long),
    'float32': (RosFloat32, float, float),
    'float64': (RosFloat64, float, float),
    # From here multiple interpretation are possible
    # careful with py2 and py3 differences here...
    'string': [(RosString, six.binary_type, six.binary_type)],  #, (RosTextString, six.binary_type, six.text_type),
    'time': [(RosTime, genpy.Time, six_long)],
    'duration': [(RosDuration, genpy.Duration, six_long)],
}

def schema_from_fieldtypestring(fieldtypestring):
    return field_types_data_schemas_rostype_pytype.get(fieldtypestring)[0]

def rosfieldtype_from_fieldtypestring(fieldtypestring):
    return field_types_data_schemas_rostype_pytype.get(fieldtypestring)[1]

def pyfieldtype_from_fieldtypestring(fieldtypestring):
    return field_types_data_schemas_rostype_pytype.get(fieldtypestring)[2]


# Note : the ROS messages used here are just needed for testing field serialization
# TODO : check only field serialization without using messages... => HOW ?


#
# # We need a composite strategy to link slot type and slot value
# @st.composite
# @hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1)
# def msg_rostype_and_value(draw, msgs_type_strat_tuples):
#     msg_type_strat = draw(st.sampled_from(msgs_type_strat_tuples))
#     msg_value = draw(msg_type_strat[1])
#     return msg_type_strat[0], msg_value
#
#
# # We need a composite strategy to link slot type and slot value
# @st.composite
# @hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1)
# def fieldtype_and_value(draw, field_type_strat_tuples):
#     fieldtype_strat = draw(st.sampled_from(field_type_strat_tuples))
#     msg_value = draw(fieldtype_strat[1])
#     return fieldtype_strat[0], msg_value



def field_deserialize_serialize_from_ros_inverse(msg_typestring, msg_value):

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(field_types_data_schemas_rostype_pytype.get(fieldtypestring_from_rostypestring(msg_typestring))):
        schema_field_type, rosfield_pytype, dictfield_pytype = possible_interpretation

        # Schemas' Field constructor
        field = schema_field_type()

        assert hasattr(msg_value, 'data')
        deserialized = field.deserialize(msg_value.data)

        # check the serialized version is the type we expect
        assert isinstance(deserialized, dictfield_pytype)
        serialized = field.serialize(0, [deserialized])

        # Check the field value we obtain is the expected ros type and same value.
        assert isinstance(serialized, rosfield_pytype)
        assert serialized == msg_value.data

# Note : aggregating these tests leads to slow data generation

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Bool'))
def test_Bool_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Bool', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int8'))
def test_Int8_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Int8', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int16'))
def test_Int16_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Int16', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int32'))
def test_Int32_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Int32', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int64'))
def test_Int64_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Int64', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt8'))
def test_UInt8_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/UInt8', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt16'))
def test_UInt16_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/UInt16', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt32'))
def test_UInt32_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/UInt32', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt64'))
def test_UInt64_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/UInt64', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Float32'))
def test_Float32_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Float32', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Float64'))
def test_Float64_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Float64', msg_value)



@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/String'))
def test_String_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/String', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Time'))
def test_Time_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Time', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Duration'))
def test_Duration_field_deserialize_serialize_from_ros_inverse(msg_value):
    return field_deserialize_serialize_from_ros_inverse('std_msgs/Duration', msg_value)

#TODO : more of that...




def field_deserialize_from_ros_to_type(msg_type, msg_value):

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(field_types_data_schemas_rostype_pytype.get(fieldtypestring_from_rostypestring(msg_type))):
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
            if rosfield_pytype == genpy.Time or rosfield_pytype == genpy.Duration:  # non verbatim basic fields
                # TODO : find a way to get rid of this special case...
                assert deserialized == dictfield_pytype(msg_value.data.to_nsec())
            elif rosfield_pytype == list:  # TODO : improve this check
                # TODO : find a way to get rid of this special case...
                for idx, elem in enumerate(msg_value.data):
                    assert deserialized[idx] == elem
            else:
                assert deserialized == dictfield_pytype(msg_value.data)
        else:  # not a basic type for python (slots should be there though...)
            assert deserialized == dictfield_pytype([(s, getattr(msg_value.data, s)) for s in msg_value.data.__slots__])


# Note : aggregating these tests leads to slow data generation

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Bool'))
def test_Bool_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Bool', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int8'))
def test_Int8_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Int8', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int16'))
def test_Int16_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Int16', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int32'))
def test_Int32_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Int32', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int64'))
def test_Int64_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Int64', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt8'))
def test_UInt8_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/UInt8', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt16'))
def test_UInt16_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/UInt16', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt32'))
def test_UInt32_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/UInt32', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt64'))
def test_UInt64_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/UInt64', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Float32'))
def test_Float32_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Float32', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Float64'))
def test_Float64_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Float64', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/String'))
def test_String_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/String', msg_value)


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Time'))
def test_Time_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Time', msg_value)

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Duration'))
def test_Duration_field_deserialize_from_ros_to_type(msg_value):
    return field_deserialize_from_ros_to_type('std_msgs/Duration', msg_value)



def field_serialize_deserialize_from_py_inverse(field_type, pyfield):

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(field_types_data_schemas_rostype_pytype.get(field_type)):
        schema_field_type, rosfield_pytype, pyfield_pytype = possible_interpretation

        # Schemas' Field constructor
        field = schema_field_type()

        serialized = field.serialize(0, [pyfield])

        # Check the serialized field is the type we expect.
        assert isinstance(serialized, rosfield_pytype)

        deserialized = field.deserialize(serialized)

        # Check the dict we obtain is the expected type and same value.
        assert isinstance(deserialized, pyfield_pytype)

        assert deserialized == pyfield


@hypothesis.given(field_strat_ok.get('bool'))
def test_bool_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('bool', pyfield)


@hypothesis.given(field_strat_ok.get('int8'))
def test_int8_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('int8', pyfield)

@hypothesis.given(field_strat_ok.get('int16'))
def test_int16_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('int16', pyfield)

@hypothesis.given(field_strat_ok.get('int32'))
def test_int32_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('int32', pyfield)

@hypothesis.given(field_strat_ok.get('int64'))
def test_int64_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('int64', pyfield)


@hypothesis.given(field_strat_ok.get('uint8'))
def test_uint8_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('uint8', pyfield)

@hypothesis.given(field_strat_ok.get('uint16'))
def test_uint16_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('uint16', pyfield)

@hypothesis.given(field_strat_ok.get('uint32'))
def test_uint32_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('uint32', pyfield)

@hypothesis.given(field_strat_ok.get('uint64'))
def test_uint64_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('uint64', pyfield)


@hypothesis.given(field_strat_ok.get('float32'))
def test_float32_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('float32', pyfield)

@hypothesis.given(field_strat_ok.get('float64'))
def test_float64_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('float64', pyfield)


@hypothesis.given(field_strat_ok.get('string'))
def test_string_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('string', pyfield)


@hypothesis.given(field_strat_ok.get('time'))
def test_time_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('time', pyfield)


@hypothesis.given(field_strat_ok.get('duration'))
def test_duration_field_serialize_deserialize_from_py_inverse(pyfield):
    return field_serialize_deserialize_from_py_inverse('duration', pyfield)


#
# @hypothesis.given(fieldtype_and_value(proper_basic_data_strategy_selector(
#     'bool',
#     'int8',
#     'int16',
#     'int32',
#     'int64',
#     'uint8',
#     'uint16',
#     'uint32',
#     'uint64',
#     'float32',
#     'float64',
#     'string',
#     'time',
#     'duration',
#     #TODO : more of that...
# )))
# def test_Bool_field_serialize_deserialize_from_py_inverse(msg_rostype_and_value):
#     # TODO : make it clearer that we get different data here, even if we still use msg_rostype_and_value
#     # Same values as for ros message test
#     msg_type = msg_rostype_and_value[0]
#     pyfield = msg_rostype_and_value[1]
#
#     # testing all possible schemas for data field
#     for possible_interpretation in maybe_list(field_types_data_schemas_rostype_pytype.get(msg_type)):
#         schema_field_type, rosfield_pytype, pyfield_pytype = possible_interpretation
#
#         # Schemas' Field constructor
#         field = schema_field_type()
#
#         serialized = field.serialize(0, [pyfield])
#
#         # Check the serialized field is the type we expect.
#         assert isinstance(serialized, rosfield_pytype)
#
#         deserialized = field.deserialize(serialized)
#
#         # Check the dict we obtain is the expected type and same value.
#         assert isinstance(deserialized, pyfield_pytype)
#
#         assert deserialized == pyfield







def field_serialize_from_py_to_type(msg_type, pyfield):

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(field_types_data_schemas_rostype_pytype.get(msg_type)):
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



@hypothesis.given(field_strat_ok.get('bool'))
def test_bool_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('bool', pyfield)


@hypothesis.given(field_strat_ok.get('int8'))
def test_int8_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('int8', pyfield)

@hypothesis.given(field_strat_ok.get('int16'))
def test_int16_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('int16', pyfield)

@hypothesis.given(field_strat_ok.get('int32'))
def test_int32_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('int32', pyfield)

@hypothesis.given(field_strat_ok.get('int64'))
def test_int64_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('int64', pyfield)


@hypothesis.given(field_strat_ok.get('uint8'))
def test_uint8_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('uint8', pyfield)

@hypothesis.given(field_strat_ok.get('uint16'))
def test_uint16_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('uint16', pyfield)

@hypothesis.given(field_strat_ok.get('uint32'))
def test_Bool_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('uint32', pyfield)

@hypothesis.given(field_strat_ok.get('uint64'))
def test_uint64_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('uint64', pyfield)


@hypothesis.given(field_strat_ok.get('float32'))
def test_float32_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('float32', pyfield)

@hypothesis.given(field_strat_ok.get('float64'))
def test_float64_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('float64', pyfield)


@hypothesis.given(field_strat_ok.get('string'))
def test_string_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('string', pyfield)


@hypothesis.given(field_strat_ok.get('time'))
def test_time_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('time', pyfield)


@hypothesis.given(field_strat_ok.get('duration'))
def test_duration_field_serialize_from_py_to_type(pyfield):
    return field_serialize_from_py_to_type('duration', pyfield)

#
#
# @hypothesis.given(fieldtype_and_value(proper_basic_data_strategy_selector(
#     'bool',
#     'int8',
#     'int16',
#     'int32',
#     'int64',
#     'uint8',
#     'uint16',
#     'uint32',
#     'uint64',
#     'float32',
#     'float64',
#     'string',
#     'time',
#     'duration',
#     #TODO : more of that...
# )))
# def test_field_serialize_from_py_to_type(msg_rostype_and_value):
#     # TODO : makeit clearer that we get different data here, even if we still use msg_rostype_and_value
#     # Same values as for ros message test
#     msg_type = msg_rostype_and_value[0]
#     pyfield = msg_rostype_and_value[1]
#
#     # testing all possible schemas for data field
#     for possible_interpretation in maybe_list(field_types_data_schemas_rostype_pytype.get(msg_type)):
#         schema_field_type, rosfield_pytype, pyfield_pytype = possible_interpretation
#
#         # test_frompy(pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype):
#
#         # Schemas' Field constructor
#         field = schema_field_type()
#
#         serialized = field.serialize(0, [pyfield])
#
#         # Check the serialized field is the type we expect.
#         assert isinstance(serialized, rosfield_pytype)
#         # check the serialized value is the same as the value of that field in the original message
#         # We need the type conversion to deal with serialized object in different format than ros data (like string)
#         # we also need to deal with slots in case we have complex objects (only one level supported)
#         if rosfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type]:
#             assert serialized == pyfield
#         else:  # not a basic type for python
#             if rosfield_pytype == genpy.Time or rosfield_pytype == genpy.Duration:
#                 # these are deserialized (deterministically) as basic types (long nsecs)
#                 # working around genpy.rostime abismal performance
#                 pyfield_s = pyfield // 1000000000
#                 pyfield_ns = pyfield - pyfield_s * 1000000000
#                 assert serialized == rosfield_pytype(secs=pyfield_s, nsecs=pyfield_ns)
#             elif pyfield_pytype == list:
#                 for idx, elem in enumerate(pyfield):
#                     assert serialized[idx] == elem
#             else:  # dict format can be used for nested types though...
#                 assert serialized == rosfield_pytype(**pyfield)


# Just in case we run this directly
if __name__ == '__main__':
    pytest.main([
        '-s', __file__,
    ])
