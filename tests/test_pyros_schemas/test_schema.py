from __future__ import absolute_import, print_function

import pytest

from pyros_schemas.ros.schemagic import create

from . import six_long

import hypothesis
import hypothesis.strategies as st


import std_msgs.msg as std_msgs
import genpy

from . import msg as pyros_schemas_test_msgs

from .strategies.ros import std_msgs_types_strat_ok, std_msgs_dicts_strat_ok

# Some tests are still failing on python3, related to strings. see https://github.com/ros/genpy/pull/85 and https://github.com/ros/genpy/pull/90 for related discussion
# also https://discourse.ros.org/t/python3-and-strings/2392


std_msgs_types = {
    'std_msgs/Bool': std_msgs.Bool,
    'std_msgs/Int8': std_msgs.Int8,
    'std_msgs/Int16': std_msgs.Int16,
    'std_msgs/Int32': std_msgs.Int32,
    'std_msgs/Int64': std_msgs.Int64,
    'std_msgs/UInt8': std_msgs.UInt8,
    'std_msgs/UInt16': std_msgs.UInt16,
    'std_msgs/UInt32': std_msgs.UInt32,
    'std_msgs/UInt64': std_msgs.UInt64,
    'std_msgs/Float32': std_msgs.Float32,
    'std_msgs/Float64': std_msgs.Float64,
    'std_msgs/String': std_msgs.String,
    'std_msgs/Time': std_msgs.Time,
    'std_msgs/Duration': std_msgs.Duration
}
pyros_schemas_opttypes = {
    'pyros_schemas/test_opt_bool_as_array': pyros_schemas_test_msgs.test_opt_bool_as_array,
    'pyros_schemas/test_opt_int8_as_array': pyros_schemas_test_msgs.test_opt_int8_as_array,
    'pyros_schemas/test_opt_int16_as_array': pyros_schemas_test_msgs.test_opt_int16_as_array,
    'pyros_schemas/test_opt_int32_as_array': pyros_schemas_test_msgs.test_opt_int32_as_array,
    'pyros_schemas/test_opt_int64_as_array': pyros_schemas_test_msgs.test_opt_int64_as_array,
    'pyros_schemas/test_opt_uint8_as_array': pyros_schemas_test_msgs.test_opt_uint8_as_array,
    'pyros_schemas/test_opt_uint16_as_array': pyros_schemas_test_msgs.test_opt_uint16_as_array,
    'pyros_schemas/test_opt_uint32_as_array': pyros_schemas_test_msgs.test_opt_uint32_as_array,
    'pyros_schemas/test_opt_uint64_as_array': pyros_schemas_test_msgs.test_opt_uint64_as_array,
    'pyros_schemas/test_opt_float32_as_array': pyros_schemas_test_msgs.test_opt_float32_as_array,
    'pyros_schemas/test_opt_float64_as_array': pyros_schemas_test_msgs.test_opt_float64_as_array,
    'pyros_schemas/test_opt_string_as_array': pyros_schemas_test_msgs.test_opt_string_as_array,
    'pyros_schemas/test_opt_time_as_array': pyros_schemas_test_msgs.test_opt_time_as_array,
    'pyros_schemas/test_opt_duration_as_array': pyros_schemas_test_msgs.test_opt_duration_as_array,
}

# simple way to define mapping between ros types and deserialized dictionary for testing
def std_msgs_dicts_from_rostype_map(msg_type, rostype_value):
    if msg_type in (
        'std_msgs/Bool',
        'std_msgs/Int8', 'std_msgs/Int16', 'std_msgs/Int32', 'std_msgs/Int64',
        'std_msgs/UInt8', 'std_msgs/UInt16', 'std_msgs/UInt32', 'std_msgs/UInt64',
    ):
        return {'data': rostype_value.data}
    elif msg_type in (
        'std_msgs/Float32', 'std_msgs/Float64',
    ):
        return {'data': rostype_value.data}
    elif msg_type in (
        'std_msgs/String',
    ):
        # no need to decode/encode here but be careful about non-printable control characters...
        # Ref : http://www.madore.org/~david/computers/unicode/#faq_ascii
        return {'data': rostype_value.data}
    elif msg_type in (
        'std_msgs/Time', 'std_msgs/Duration',
    ):
        return {'data': rostype_value.data.to_nsec()}


def pyros_schemas_dicts_from_rostype_map(msg_type, rostype_value):
    if msg_type in (
            'pyros_schemas/test_opt_bool_as_array',
            'pyros_schemas/test_opt_int8_as_array', 'pyros_schemas/test_opt_int16_as_array',
            'pyros_schemas/test_opt_int32_as_array', 'pyros_schemas/test_opt_int64_as_array',
            'pyros_schemas/test_opt_uint8_as_array', 'pyros_schemas/test_opt_uint16_as_array',
            'pyros_schemas/test_opt_uint32_as_array', 'pyros_schemas/test_opt_uint64_as_array',
    ):
        return {'data': rostype_value.data}
    elif msg_type in (
            'pyros_schemas/test_opt_float32_as_array', 'pyros_schemas/test_opt_float64_as_array',
    ):
        return {'data': rostype_value.data}
    elif msg_type in (
            'pyros_schemas/test_opt_string_as_array',
    ):
        # no need to decode/encode here but be careful about non-printable control characters...
        # Ref : http://www.madore.org/~david/computers/unicode/#faq_ascii
        return {'data': rostype_value.data}
    elif msg_type in (
            'pyros_schemas/test_opt_time_as_array', 'pyros_schemas/test_opt_duration_as_array'
    ):
        return {'data': rostype_value.data.to_nsec()}


# simple way to define mapping between dictionary and serialized rostype for testing
def std_msgs_rostypes_from_dict_map(msg_type, dict_value):
    if msg_type in (
        'std_msgs/Bool',
        'std_msgs/Int8', 'std_msgs/Int16', 'std_msgs/Int32', 'std_msgs/Int64',
        'std_msgs/UInt8', 'std_msgs/UInt16', 'std_msgs/UInt32', 'std_msgs/UInt64',
    ):
        rostype = std_msgs_types.get(msg_type)
        return rostype(data=dict_value.get('data'))
    elif msg_type in (
        'std_msgs/Float32', 'std_msgs/Float64',
    ):
        rostype = std_msgs_types.get(msg_type)
        return rostype(data=dict_value.get('data'))
    elif msg_type in (
        'std_msgs/String',
    ):
        rostype = std_msgs_types.get(msg_type)
        return rostype(data=dict_value.get('data'))  # careful about non-printable control characters
    elif msg_type in (
        'std_msgs/Time',
    ):
        rostype = std_msgs_types.get(msg_type)
        return rostype(data=genpy.Time(nsecs=dict_value.get('data')))
    elif msg_type in (
        'std_msgs/Duration',
    ):
        rostype = std_msgs_types.get(msg_type)
        return rostype(data=genpy.Duration(nsecs=dict_value.get('data')))


def pyros_schemas_rostypes_from_dict_map(msg_type, dict_value):
    if msg_type in (
        'pyros_schemas/test_opt_bool_as_array',
        'pyros_schemas/test_opt_int8_as_array', 'pyros_schemas/test_opt_int16_as_array', 'pyros_schemas/test_opt_int32_as_array', 'pyros_schemas/test_opt_int64_as_array',
        'pyros_schemas/test_opt_uint8_as_array', 'pyros_schemas/test_opt_uint16_as_array', 'pyros_schemas/test_opt_uint32_as_array', 'pyros_schemas/test_opt_uint64_as_array',
    ):
        rostype = pyros_schemas_opttypes.get(msg_type)
        return rostype(data=dict_value.get('data'))
    elif msg_type in (
        'pyros_schemas/test_opt_float32_as_array', 'pyros_schemas/test_opt_float64_as_array',
    ):
        rostype = pyros_schemas_opttypes.get(msg_type)
        return rostype(data=dict_value.get('data'))
    elif msg_type in (
        'pyros_schemas/test_opt_string_as_array',
    ):
        rostype = pyros_schemas_opttypes.get(msg_type)
        return rostype(data=dict_value.get('data'))  # careful about non-printable control characters
    elif msg_type in (
        'pyros_schemas/test_opt_time_as_array',

    ):
        rostype = pyros_schemas_opttypes.get(msg_type)
        return rostype(data=genpy.Time(nsecs=dict_value.get('data')))
    elif msg_type in (
        'pyros_schemas/test_opt_duration_as_array',
    ):
        rostype = pyros_schemas_opttypes.get(msg_type)
        return rostype(data=genpy.Duration(nsecs=dict_value.get('data')))


# We need a composite strategy to link msg type and dict structure
@st.composite
def msg_rostype_and_dict(draw, msgs_type_strat_tuples):
    msg_type_strat = draw(st.sampled_from(msgs_type_strat_tuples))
    msg_value = draw(msg_type_strat[1])
    msg_dict = std_msgs_dicts_from_rostype_map(msg_type_strat[0], msg_value)
    return msg_type_strat[0], msg_value, msg_dict


def schema_load_dump_fromros_inverse(msg_rostype, ros_msg, py_inst_expected):
    # msg_rostype is just for info/debug purposes

    schema = create(type(ros_msg))

    marshalled, errors = schema.load(ros_msg)
    assert not errors and marshalled == py_inst_expected

    value, errors = schema.dump(marshalled)
    assert not errors and type(value) == type(ros_msg) and value == ros_msg


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Bool'))
def test_bool_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Bool', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Bool', msg_value))


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int8'))
def test_int8_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Int8', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Int8', msg_value))

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int16'))
def test_int16_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Int16', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Int16', msg_value))

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int32'))
def test_int32_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Int32', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Int32', msg_value))

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int64'))
def test_int64_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Int64', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Int64', msg_value))


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt8'))
def test_uint8_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/UInt8', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/UInt8', msg_value))

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt16'))
def test_uint16_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/UInt16', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/UInt16', msg_value))

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Int32'))
def test_uint32_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Int32', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/UInt32', msg_value))

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/UInt64'))
def test_uint64_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/UInt64', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/UInt64', msg_value))


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Float32'))
def test_float32_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Float32', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Float32', msg_value))

@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Float64'))
def test_float64_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Float64', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Float64', msg_value))


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/String'))
def test_string_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/String', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/String', msg_value))


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Time'))
def test_time_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Time', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Time', msg_value))


@hypothesis.given(std_msgs_types_strat_ok.get('std_msgs/Duration'))
def test_duration_schema_load_dump_fromros_inverse(msg_value):
    return schema_load_dump_fromros_inverse('std_msgs/Duration', msg_value, std_msgs_dicts_from_rostype_map('std_msgs/Duration', msg_value))


def schema_dump_load_frompy_inverse(msg_rostype, py_inst, ros_msg_expected):
    # msg_rostype is just for info/debug purposes

    schema = create(type(ros_msg_expected))

    unmarshalled, errors = schema.dump(py_inst)
    assert not errors and type(unmarshalled) == type(ros_msg_expected) and unmarshalled == ros_msg_expected

    obj, errors = schema.load(unmarshalled)
    assert not errors and type(obj) == type(py_inst) and obj == py_inst


@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Bool'))
def test_bool_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Bool', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Bool', msg_value))


@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Int8'))
def test_int8_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Int8', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Int8', msg_value))

@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Int16'))
def test_int16_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Int16', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Int16', msg_value))

@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Int32'))
def test_int32_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Int32', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Int32', msg_value))

@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Int64'))
def test_int64_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Int64', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Int64', msg_value))


@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/UInt8'))
def test_uint8_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/UInt8', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/UInt8', msg_value))

@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/UInt16'))
def test_uint16_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/UInt16', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/UInt16', msg_value))

@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Int32'))
def test_uint32_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Int32', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/UInt32', msg_value))

@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/UInt64'))
def test_uint64_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/UInt64', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/UInt64', msg_value))


@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Float32'))
def test_float32_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Float32', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Float32', msg_value))

@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Float64'))
def test_float64_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Float64', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Float64', msg_value))


@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/String'))
def test_string_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/String', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/String', msg_value))


@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Time'))
def test_time_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Time', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Time', msg_value))


@hypothesis.given(std_msgs_dicts_strat_ok.get('std_msgs/Duration'))
def test_duration_schema_dump_load_frompy_inverse(msg_value):
    return schema_dump_load_frompy_inverse('std_msgs/Duration', msg_value, std_msgs_rostypes_from_dict_map('std_msgs/Duration', msg_value))



# TODO :
#     # MultiArrayDimension
#     (std_msgs.msg.MultiArrayDimension(label=, size=, stride=), RosBool, bool, bool, bool),


# Just in case we run this directly
if __name__ == '__main__':
    pytest.main([
        '-s',
        'test_schema.py::test_schema_load_dump_fromros_inverse',
        'test_schema.py::test_schema_dump_load_frompy_inverse',
    ])
