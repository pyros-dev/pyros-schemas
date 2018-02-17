from __future__ import absolute_import, division, print_function, unicode_literals

import pytest

import std_msgs.msg as std_msgs
import genpy

import sys
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

from pyros_schemas.ros.optional_fields import (
    RosOptAsList,
)


# Some tests are still failing on python3, related to strings. see https://github.com/ros/genpy/pull/85 and https://github.com/ros/genpy/pull/90 for related discussion
# also https://discourse.ros.org/t/python3-and-strings/2392

from . import six_long, maybe_list

from . import msg as pyros_schemas_test_msgs

from .strategies.ros import pyros_schemas_opttypes_strat_ok

# TODO : make that generic to be able to test any message type...
# Note : we do not want to test implicit python type conversion here (thats the job of pyros_msgs typechecker)
#: (schema_field_type, rosfield_pytype, dictfield_pytype)
pyros_schemas_opttypes_data_schemas_rosopttype_pytype = {
    'optbool': (lambda: RosOptAsList(RosBool()), bool, bool),
    'optint8': (lambda: RosOptAsList(RosInt8()), int, int),
    'optint16': (lambda: RosOptAsList(RosInt16()), int, int),
    'optint32': (lambda: RosOptAsList(RosInt32()), int, int),
    'optint64': (lambda: RosOptAsList(RosInt64()), six_long, six_long),
    'optuint8': (lambda: RosOptAsList(RosUInt8()), int, int),
    'optuint16': (lambda: RosOptAsList(RosUInt16()), int, int),
    'optuint32': (lambda: RosOptAsList(RosUInt32()), int, int),
    'optuint64': (lambda: RosOptAsList(RosUInt64()), six_long, six_long),
    'optfloat32': (lambda: RosOptAsList(RosFloat32()), float, float),
    'optfloat64': (lambda: RosOptAsList(RosFloat64()), float, float),
#    'optstring': [(lambda: RosOptAsList(RosString()), six.binary_type, six.binary_type)], # , (RosTextString, six.binary_type, six.text_type)],
    'optstring': [(lambda: RosOptAsList(RosTextString()), six.binary_type, six.text_type)],  # Note the ambiguity of str for py2/py3

    'opttime': [(lambda: RosOptAsList(RosTime()), genpy.Time, six_long)],
    'optduration': [(lambda: RosOptAsList(RosDuration()), genpy.Duration, six_long)],
}


# if sys.version_info < (3,0):
#     pyros_schemas_opttypes_data_schemas_rosopttype_pytype['optstring']= [(lambda: RosOptAsList(RosTextString()), six.binary_type, six.binary_type)]


pyros_schemas_opttypes_data_ros_field_types = {
    'pyros_schemas/test_opt_bool_as_array': (pyros_schemas_test_msgs.test_opt_bool_as_array, 'optbool'),
    'pyros_schemas/test_opt_int8_as_array': (pyros_schemas_test_msgs.test_opt_int8_as_array, 'optint8'),
    'pyros_schemas/test_opt_int16_as_array': (pyros_schemas_test_msgs.test_opt_int16_as_array, 'optint16'),
    'pyros_schemas/test_opt_int32_as_array': (pyros_schemas_test_msgs.test_opt_int32_as_array, 'optint32'),
    'pyros_schemas/test_opt_int64_as_array': (pyros_schemas_test_msgs.test_opt_int64_as_array, 'optint64'),
    'pyros_schemas/test_opt_uint8_as_array': (pyros_schemas_test_msgs.test_opt_uint8_as_array, 'optuint8'),
    'pyros_schemas/test_opt_uint16_as_array': (pyros_schemas_test_msgs.test_opt_uint16_as_array, 'optuint16'),
    'pyros_schemas/test_opt_uint32_as_array': (pyros_schemas_test_msgs.test_opt_uint32_as_array, 'optuint32'),
    'pyros_schemas/test_opt_uint64_as_array': (pyros_schemas_test_msgs.test_opt_uint64_as_array, 'optuint64'),
    'pyros_schemas/test_opt_float32_as_array': (pyros_schemas_test_msgs.test_opt_float32_as_array, 'optfloat32'),
    'pyros_schemas/test_opt_float64_as_array': (pyros_schemas_test_msgs.test_opt_float64_as_array, 'optfloat64'),
    'pyros_schemas/test_opt_string_as_array': (pyros_schemas_test_msgs.test_opt_string_as_array, 'optstring'),
    'pyros_schemas/test_opt_time_as_array': (pyros_schemas_test_msgs.test_opt_time_as_array, 'opttime'),
    'pyros_schemas/test_opt_duration_as_array': (pyros_schemas_test_msgs.test_opt_duration_as_array, 'optduration'),
}


def rostype_from_rostypestring(rostypestring):
    return pyros_schemas_opttypes_data_ros_field_types.get(rostypestring)[0]


def fieldtypestring_from_rostypestring(rostypestring):
    return pyros_schemas_opttypes_data_ros_field_types.get(rostypestring)[1]


# We need a composite strategy to link slot type and slot value
@st.composite
@hypothesis.settings(timeout=1)
def msg_rostype_and_value(draw, msgs_type_strat_tuples):
    msg_type_strat = draw(st.sampled_from(msgs_type_strat_tuples))
    # print(msg_type_strat[1])  # just in case, to help debugging strategies
    msg_value = draw(msg_type_strat[1])
    return msg_type_strat[0], msg_value


def optfield_deserialize_serialize_from_ros_inverse(msg_type, msg_value):

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(pyros_schemas_opttypes_data_schemas_rosopttype_pytype.get(fieldtypestring_from_rostypestring(msg_type))):
        schema_field_type, rosfield_pytype, dictfield_pytype = possible_interpretation

        # Schemas' Field constructor
        field = schema_field_type()

        assert hasattr(msg_value, 'data')
        deserialized = field.deserialize(msg_value.data)

        # check the deserialized version is the type we expect (or a missing optional field)
        assert isinstance(deserialized, dictfield_pytype) or deserialized == marshmallow.utils.missing
        if deserialized != marshmallow.utils.missing:  # no point to do further testing on missing field

            serialized = field.serialize(0, [deserialized])

            # Check the field value we obtain is the expected ros type and same value.
            assert isinstance(serialized[0], rosfield_pytype)
            assert serialized == msg_value.data


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_bool_as_array'))
def test_bool_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_bool_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_int8_as_array'))
def test_int8_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_int8_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_int16_as_array'))
def test_int16_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_int16_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_int32_as_array'))
def test_int32_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_int32_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_int64_as_array'))
def test_int64_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_int64_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_uint8_as_array'))
def test_uint8_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_uint8_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_uint16_as_array'))
def test_uint16_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_uint16_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_uint32_as_array'))
def test_uint32_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_uint32_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_uint64_as_array'))
def test_uint64_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_uint64_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_float32_as_array'))
def test_float32_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_float32_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_float64_as_array'))
def test_float64_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_float64_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_string_as_array'))
def test_string_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_string_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_time_as_array'))
def test_time_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_time_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_duration_as_array'))
def test_duration_optfield_deserialize_serialize_from_ros_inverse(msg_value):
    return optfield_deserialize_serialize_from_ros_inverse('pyros_schemas/test_opt_duration_as_array', msg_value)


def optfield_deserialize_from_ros_to_type_in_list(msg_type, msg_value):

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(pyros_schemas_opttypes_data_schemas_rosopttype_pytype.get(fieldtypestring_from_rostypestring(msg_type))):
        schema_field_type, rosfield_pytype, dictfield_pytype = possible_interpretation

        # Schemas' Field constructor
        field = schema_field_type()

        assert hasattr(msg_value, 'data')
        # Making sure the data msg field is of the intended pytype
        # in case ROS messages do - or dont do - some conversions
        # TODO investigate : string breaking here (str/unicode)
        #assert len(msg_value.data) == 0 or isinstance(msg_value.data[0], rosfield_pytype)
        deserialized = field.deserialize(msg_value.data)

        # check the deserialized version is the type we expect (or a missing optional field)
        assert deserialized == marshmallow.utils.missing or isinstance(deserialized, dictfield_pytype)
        if deserialized != marshmallow.utils.missing:
            # check the deserialized value is the same as the value of that field in the original message
            # We need the type conversion to deal with deserialized object in different format than ros data (like string)
            # we also need to deal with slots in case we have complex objects (only one level supported)
            if dictfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type, list]:
                if rosfield_pytype == genpy.Time or rosfield_pytype == genpy.Duration:  # non verbatim basic fields
                    # TODO : find a way to get rid of this special case...
                    assert deserialized == dictfield_pytype(msg_value.data[0].to_nsec())
                elif rosfield_pytype == list:  # TODO : improve this check
                    # TODO : find a way to get rid of this special case...
                    for idx, elem in enumerate(msg_value.data):
                        assert deserialized[idx] == elem
                else:
                    assert deserialized == dictfield_pytype(msg_value.data[0])
            else:  # not a basic type for python (slots should be there though...)
                assert deserialized == dictfield_pytype(
                    [(s, getattr(msg_value.data, s)) for s in msg_value.data.__slots__])


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_bool_as_array'))
def test_bool_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_bool_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_int8_as_array'))
def test_int8_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_int8_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_int16_as_array'))
def test_int16_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_int16_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_int32_as_array'))
def test_int32_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_int32_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_int64_as_array'))
def test_int64_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_int64_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_uint8_as_array'))
def test_uint8_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_uint8_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_uint16_as_array'))
def test_uint16_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_uint16_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_uint32_as_array'))
def test_uint32_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_uint32_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_uint64_as_array'))
def test_uint64_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_uint64_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_float32_as_array'))
def test_float32_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_float32_as_array', msg_value)

@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_float64_as_array'))
def test_float64_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_float64_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_string_as_array'))
def test_string_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_string_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_time_as_array'))
def test_time_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_time_as_array', msg_value)


@hypothesis.given(pyros_schemas_opttypes_strat_ok.get('pyros_schemas/test_opt_duration_as_array'))
def test_duration_optfield_deserialize_from_ros_to_type_in_list(msg_value):
    return optfield_deserialize_from_ros_to_type_in_list('pyros_schemas/test_opt_duration_as_array', msg_value)


def optfield_serialize_deserialize_from_py_inverse(msg_type, pyfield):
    # get actual type from type string
    # rosmsg_type = rostype_from_rostypestring(msg_type)

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(pyros_schemas_opttypes_data_schemas_rosopttype_pytype.get(msg_type)):
        schema_field_type, rosfield_pytype, pyfield_pytype = possible_interpretation

        # test_frompy(pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype):

        # Schemas' Field constructor
        field = schema_field_type()

        serialized = field.serialize(0, [pyfield])

        # Building the ros message in case it changes something...
        # ros_msg = rosmsg_type(data=serialized)
        # deserialized = field.deserialize(ros_msg.data)
        deserialized = field.deserialize(serialized)

        if deserialized != marshmallow.utils.missing:

            # Check the dict we obtain is the expected type and same value.
            assert isinstance(deserialized, pyfield_pytype)
            # if pyfield_pytype not in [bool, int, six_long, float, six.binary_type, six.text_type, list]:
            #     # If we were missing some fields, we need to initialise to default ROS value to be able to compare
            #     for i, s in enumerate(ros_msg.data.__slots__):
            #         if s not in pyfield.keys():
            #             pyfield[s] = ros_pythontype_mapping[ros_msg.data._slot_types[i]]()

            assert deserialized == pyfield


from .strategies.python import optfield_strat_ok


@hypothesis.given(optfield_strat_ok.get('optbool'))
def test_bool_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optbool', msg_value)


@hypothesis.given(optfield_strat_ok.get('optint8'))
def test_int8_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optint8', msg_value)

@hypothesis.given(optfield_strat_ok.get('optint16'))
def test_int16_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optint16', msg_value)

@hypothesis.given(optfield_strat_ok.get('optint32'))
def test_int32_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optint32', msg_value)

@hypothesis.given(optfield_strat_ok.get('optint64'))
def test_int64_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optint64', msg_value)


@hypothesis.given(optfield_strat_ok.get('optuint8'))
def test_uint8_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optuint8', msg_value)

@hypothesis.given(optfield_strat_ok.get('optuint16'))
def test_uint16_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optuint16', msg_value)

@hypothesis.given(optfield_strat_ok.get('optuint32'))
def test_uint32_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optuint32', msg_value)

@hypothesis.given(optfield_strat_ok.get('optuint64'))
def test_uint64_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optuint64', msg_value)


@hypothesis.given(optfield_strat_ok.get('optfloat32'))
def test_float32_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optfloat32', msg_value)

@hypothesis.given(optfield_strat_ok.get('optfloat64'))
def test_float64_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optfloat64', msg_value)


@hypothesis.given(optfield_strat_ok.get('optstring'))
def test_string_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optstring', msg_value)


@hypothesis.given(optfield_strat_ok.get('opttime'))
def test_time_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('opttime', msg_value)


@hypothesis.given(optfield_strat_ok.get('optduration'))
def test_duration_optfield_serialize_deserialize_from_py_inverse(msg_value):
    return optfield_serialize_deserialize_from_py_inverse('optduration', msg_value)


def optfield_serialize_from_py_to_listtype(msg_type, pyfield):

    # get actual type from type string
    # rosmsg_type = genpy.message.get_message_class(msg_type)

    # testing all possible schemas for data field
    for possible_interpretation in maybe_list(pyros_schemas_opttypes_data_schemas_rosopttype_pytype.get(msg_type)):
        schema_field_type, rosfield_pytype, pyfield_pytype = possible_interpretation

        # Schemas' Field constructor
        field = schema_field_type()

        serialized = field.serialize(0, [pyfield])

        # Check the serialized field is the type we expect.
        assert len(serialized) == 0 or isinstance(serialized[0], rosfield_pytype)
        if len(serialized) > 0:
            # check the serialized value is the same as the value of that field in the original message
            # We need the type conversion to deal with serialized object in different format than ros data (like string)
            # we also need to deal with slots in case we have complex objects (only one level supported)
            if rosfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type]:
                assert serialized[0] == pyfield
            else:  # not a basic type for python
                if rosfield_pytype == genpy.Time or rosfield_pytype == genpy.Duration:
                    # these are deserialized (deterministically) as basic types (long nsecs)
                    # working around genpy.rostime abismal performance
                    pyfield_s = pyfield // 1000000000
                    pyfield_ns = pyfield - pyfield_s * 1000000000
                    assert serialized[0] == rosfield_pytype(secs=pyfield_s, nsecs=pyfield_ns)
                elif pyfield_pytype == list:
                    for idx, elem in enumerate(pyfield):
                        assert serialized[idx][0] == elem
                else:  # dict format can be used for nested types though...
                    assert serialized[0] == rosfield_pytype(**pyfield)


@hypothesis.given(optfield_strat_ok.get('optbool'))
def test_bool_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optbool', msg_value)


@hypothesis.given(optfield_strat_ok.get('optint8'))
def test_int8_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optint8', msg_value)

@hypothesis.given(optfield_strat_ok.get('optint16'))
def test_int16_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optint16', msg_value)

@hypothesis.given(optfield_strat_ok.get('optint32'))
def test_int32_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optint32', msg_value)

@hypothesis.given(optfield_strat_ok.get('optint64'))
def test_int64_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optint64', msg_value)


@hypothesis.given(optfield_strat_ok.get('optuint8'))
def test_uint8_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optuint8', msg_value)

@hypothesis.given(optfield_strat_ok.get('optuint16'))
def test_uint16_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optuint16', msg_value)

@hypothesis.given(optfield_strat_ok.get('optuint32'))
def test_uint32_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optuint32', msg_value)

@hypothesis.given(optfield_strat_ok.get('optuint64'))
def test_uint64_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optuint64', msg_value)


@hypothesis.given(optfield_strat_ok.get('optfloat32'))
def test_float32_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optfloat32', msg_value)

@hypothesis.given(optfield_strat_ok.get('optfloat64'))
def test_float64_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optfloat64', msg_value)


@hypothesis.given(optfield_strat_ok.get('optstring'))
def test_string_ooptfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optstring', msg_value)


@hypothesis.given(optfield_strat_ok.get('opttime'))
def test_time_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('opttime', msg_value)


@hypothesis.given(optfield_strat_ok.get('optduration'))
def test_duration_optfield_serialize_from_py_to_listtype(msg_value):
    return optfield_serialize_from_py_to_listtype('optduration', msg_value)


# Just in case we run this directly
if __name__ == '__main__':
    pytest.main([
        '-s', __file__
    ])

