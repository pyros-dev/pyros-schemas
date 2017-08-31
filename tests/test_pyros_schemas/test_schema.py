from __future__ import absolute_import, print_function

import pytest

from pyros_schemas.ros.schemagic import create

from . import (
    six_long,
    proper_basic_dict_strategy_selector,
    proper_basic_msg_strategy_selector,
    std_msgs_rostypes_from_dict_map,
    std_msgs_dicts_from_rostype_map,
)

import hypothesis
import hypothesis.strategies as st


# We need a composite strategy to link msg type and dict structure
@st.composite
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose)
def msg_rostype_and_dict(draw, msgs_type_strat_tuples):
    msg_type_strat = draw(st.sampled_from(msgs_type_strat_tuples))
    msg_value = draw(msg_type_strat[1])
    msg_dict = std_msgs_dicts_from_rostype_map(msg_type_strat[0], msg_value)
    return msg_type_strat[0], msg_value, msg_dict


@hypothesis.given(msg_rostype_and_dict(proper_basic_msg_strategy_selector(
    'std_msgs/Bool',
    'std_msgs/Int8',
    'std_msgs/Int16',
    'std_msgs/Int32',
    'std_msgs/Int64',
    'std_msgs/UInt8',
    'std_msgs/UInt16',
    'std_msgs/UInt32',
    'std_msgs/UInt64',
    'std_msgs/Float32',
    'std_msgs/Float64',
    'std_msgs/String',
    'std_msgs/Time',
    'std_msgs/Duration',
    #TODO : more of that...
)))
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose)
def test_schema_load_dump_fromros_inverse(msg_rostype_value_and_dict):
    msg_rostype = msg_rostype_value_and_dict[0]  # just for info/debug purposes
    ros_msg = msg_rostype_value_and_dict[1]
    py_inst_expected = msg_rostype_value_and_dict[2]

    schema = create(type(ros_msg))

    marshalled, errors = schema.load(ros_msg)
    assert not errors and marshalled == py_inst_expected

    value, errors = schema.dump(marshalled)
    assert not errors and type(value) == type(ros_msg) and value == ros_msg



# We need a composite strategy to link msg type and dict structure
@st.composite
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose)
def msg_dict_and_rostype(draw, msgs_dict_strat_tuples):
    msg_dict_strat = draw(st.sampled_from(msgs_dict_strat_tuples))
    msg_dict = draw(msg_dict_strat[1])
    msg_value = std_msgs_rostypes_from_dict_map(msg_dict_strat[0], msg_dict)
    return msg_dict_strat[0], msg_dict, msg_value


@hypothesis.given(msg_dict_and_rostype(proper_basic_dict_strategy_selector(
    'std_msgs/Bool',
    'std_msgs/Int8',
    'std_msgs/Int16',
    'std_msgs/Int32',
    'std_msgs/Int64',
    'std_msgs/UInt8',
    'std_msgs/UInt16',
    'std_msgs/UInt32',
    'std_msgs/UInt64',
    'std_msgs/Float32',
    'std_msgs/Float64',
    'std_msgs/String',
    'std_msgs/Time',
    'std_msgs/Duration',
    #TODO : more of that...
)))
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose)
def test_schema_dump_load_frompy_inverse(msg_rostype_dict_and_value):
    msg_rostype = msg_rostype_dict_and_value[0]  # just for info/debug purposes
    py_inst = msg_rostype_dict_and_value[1]
    ros_msg_expected = msg_rostype_dict_and_value[2]

    schema = create(type(ros_msg_expected))

    unmarshalled, errors = schema.dump(py_inst)
    assert not errors and type(unmarshalled) == type(ros_msg_expected) and unmarshalled == ros_msg_expected

    obj, errors = schema.load(unmarshalled)
    assert not errors and type(obj) == type(py_inst) and obj == py_inst


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
