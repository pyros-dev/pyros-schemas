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

from . import six_long, maybe_list, proper_basic_strategy_selector, proper_basic_data_strategy_selector


# TODO : make that generic to be able to test any message type...
# Note : we do not want to test implicit python type conversion here (thats the job of pyros_msgs typechecker)
#: (schema_field_type, rosfield_pytype, dictfield_pytype)
std_msgs_types_data_schemas_rostype_pytype = {
    'std_msgs/Bool': (RosBool, bool, bool),
    'std_msgs/Int8': (RosInt8, int, int),
    'std_msgs/Int16': (RosInt16, int, int),
    'std_msgs/Int32': (RosInt32, int, int),
    'std_msgs/Int64': (RosInt64, six_long, six_long),
    'std_msgs/UInt8': (RosUInt8, int, int),
    'std_msgs/UInt16': (RosUInt16, int, int),
    'std_msgs/UInt32': (RosUInt32, int, int),
    'std_msgs/UInt64': (RosUInt64, six_long, six_long),
    'std_msgs/Float32': (RosFloat32, float, float),
    'std_msgs/Float64': (RosFloat64, float, float),
    'std_msgs/String': [(RosString, six.binary_type, six.binary_type)],  #, (RosTextString, six.binary_type, six.text_type)],
    'std_msgs/Time': [(RosTime, genpy.Time, six_long)],
    'std_msgs/Duration': [(RosDuration, genpy.Duration, six_long)],
}


# We need a composite strategy to link slot type and slot value
@st.composite
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1, suppress_health_check=[hypothesis.HealthCheck.too_slow])
def msg_rostype_and_value(draw, msgs_type_strat_tuples):
    msg_type_strat = draw(st.sampled_from(msgs_type_strat_tuples))
    msg_value = draw(msg_type_strat[1])
    return msg_type_strat[0], msg_value


@hypothesis.given(msg_rostype_and_value(proper_basic_strategy_selector(
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
    # 'std_msgs/Time',
    # 'std_msgs/Duration',
    #TODO : more of that...
)))
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1, suppress_health_check=[hypothesis.HealthCheck.too_slow])
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
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1, suppress_health_check=[hypothesis.HealthCheck.too_slow])
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


# @pytest.mark.parametrize("pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype", [
#     # Bool
#     (True, RosBool, std_msgs.msg.Bool, bool, bool),
#     (False, RosBool, std_msgs.msg.Bool, bool, bool),
#     (bool(), RosBool, std_msgs.msg.Bool, bool, bool),  # will use default python value of dictfield_pytype()
#     # Int8
#     (42, RosInt8, std_msgs.msg.Int8, int, int),
#     (-23, RosInt8, std_msgs.msg.Int8, int, int),
#     (int(), RosInt8, std_msgs.msg.Int8, int, int),  # will use default python value of dictfield_pytype()
#     # Int16
#     (42, RosInt16, std_msgs.msg.Int16, int, int),
#     (-23, RosInt16, std_msgs.msg.Int16, int, int),
#     (int(), RosInt16, std_msgs.msg.Int16, int, int),  # will use default python value of dictfield_pytype()
#     # Int32
#     (42, RosInt32, std_msgs.msg.Int32, int, int),
#     (-23, RosInt32, std_msgs.msg.Int32, int, int),
#     (int(), RosInt32, std_msgs.msg.Int32, int, int),  # will use default python value of dictfield_pytype()
#     # Int64
#     # Careful ROS doc says Int64 is long, but it is completely dynamic
#     (42, RosInt64, std_msgs.msg.Int64, six_long, six_long),
#     (-23, RosInt64, std_msgs.msg.Int64, six_long, six_long),
#     (42424242424242424242, RosInt64, std_msgs.msg.Int64, six_long, six_long),
#     (-23232323232323232323, RosInt64, std_msgs.msg.Int64, six_long, six_long),
#     (int(), RosInt64, std_msgs.msg.Int64, six_long, six_long),
#     (six_long(), RosInt64, std_msgs.msg.Int64, six_long, six_long),  # will use default python value of dictfield_pytype()
#     # UInt8
#     (42, RosUInt8, std_msgs.msg.UInt8, int, int),
#     # Careful : negative integer are accepted by ROS in UInt fields
#     (-23, RosUInt8, std_msgs.msg.UInt8, int, int),
#     (int(), RosUInt8, std_msgs.msg.UInt8, int, int),  # will use default python value of dictfield_pytype()
#     # UInt16
#     (42, RosUInt16, std_msgs.msg.UInt16, int, int),
#     # Careful : negative integer are accepted by ROS in UInt fields
#     (-23, RosUInt16, std_msgs.msg.UInt16, int, int),
#     (int(), RosUInt16, std_msgs.msg.UInt16, int, int),  # will use default python value of dictfield_pytype()
#     # UInt32
#     (42, RosUInt32, std_msgs.msg.UInt32, int, int),
#     # Careful : negative integer are accepted by ROS in UInt fields
#     (-23, RosUInt32, std_msgs.msg.UInt32, int, int),
#     (int(), RosUInt32, std_msgs.msg.UInt32, int, int),  # will use default python value of dictfield_pytype()
#     # UInt64
#     # Careful ROS doc says UInt64 is long, but it is completely dynamic
#     (42, RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
#     (-23, RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
#     (42424242424242424242, RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
#     (-23232323232323232323, RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
#     (int(), RosUInt64, std_msgs.msg.UInt64, six_long, six_long),
#     (six_long(), RosUInt64, std_msgs.msg.UInt64, six_long, six_long),  # will use default python value of dictfield_pytype()
#     # Float32
#     (42., RosFloat32, std_msgs.msg.Float32, float, float),
#     (float(), RosFloat32, std_msgs.msg.Float32, float, float),  # will use default python value of dictfield_pytype()
#     # Float64
#     (42., RosFloat64, std_msgs.msg.Float64, float, float),
#     (float(), RosFloat64, std_msgs.msg.Float64, float, float),  # will use default python value of dictfield_pytype()
#     # String
#     ('fortytwo', RosString, std_msgs.msg.String, str, str),
#     ('', RosString, std_msgs.msg.String, str, str),
#     # Ros string field accepts unicode as data without validation, but then something will break in ROS later on...
#     (u'fortytwo', RosString, std_msgs.msg.String, str, str),
#     (u'', RosString, std_msgs.msg.String, str, str),
#     # TextString
#     ('fortytwo', RosTextString, std_msgs.msg.String, str, unicode),
#     ('', RosTextString, std_msgs.msg.String, str, unicode),
#     # Ros string field accepts unicode as data without validation, but then something will break in ROS later on...
#     (u'fortytwo', RosTextString, std_msgs.msg.String, str, unicode),
#     (u'', RosTextString, std_msgs.msg.String, str, unicode),
#     # TimeVerbatim
#     (dict(secs=42, nsecs=31), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, dict),
#     pytest.mark.xfail(strict=True, raises=TypeError, reason="ROS checks that times values are positive")((dict(secs=-23, nsecs=31), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, dict)),
#     (dict(secs=0, nsecs=0), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, dict),
#     (dict(), RosTimeVerbatim, std_msgs.msg.Time, genpy.Time, dict),
#     # Time
#     (42.00000031, RosTime, std_msgs.msg.Time, genpy.Time, float),
#     pytest.mark.xfail(strict=True, raises=TypeError, reason="ROS checks that times values are positive")((-23.00000031, RosTime, std_msgs.msg.Time, genpy.Time, float)),
#     (0.0, RosTime, std_msgs.msg.Time, genpy.Time, float),
#     # Duration Verbatim
#     (dict(secs=42, nsecs=31), RosDurationVerbatim, std_msgs.msg.Duration, genpy.Duration, dict),
#     (dict(secs=-23, nsecs=31), RosDurationVerbatim, std_msgs.msg.Duration, genpy.Duration, dict),
#     (dict(secs=0, nsecs=0), RosDurationVerbatim, std_msgs.msg.Duration, genpy.Duration, dict),
#     (dict(), RosDurationVerbatim, std_msgs.msg.Duration, genpy.Duration, dict),
#     # Duration
#     (42.00000031, RosDuration, std_msgs.msg.Duration, genpy.Duration, float),
#     (-23.00000031, RosDuration, std_msgs.msg.Duration, genpy.Duration, float),
#     (0.0, RosDuration, std_msgs.msg.Duration, genpy.Duration, float),
#
#     # To test arrays with simple messages (none in std_msgs)
#     # not we do not play with optional patching here, we are just treating that message
#     # as a message containing a simple array field.
#     ([True, False], lambda: RosList(RosBool()), pyros_msgs.msg.test_opt_bool_as_array, list, list),
#     ([False], lambda: RosList(RosBool()), pyros_msgs.msg.test_opt_bool_as_array, list, list),
#     ([], lambda: RosList(RosBool()), pyros_msgs.msg.test_opt_bool_as_array, list, list),
#     pytest.mark.xfail(strict=True, raises=marshmallow.ValidationError, reason="None is not accepted as value inside a list field")(([None], lambda: RosList(RosBool()), pyros_msgs.msg.test_opt_bool_as_array, list, list)),
#
# ])
# def test_frompy(pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype):
#
#     # Schemas' Field constructor
#     field = schema_field_type()
#
#     serialized = field.serialize(0, [pyfield])
#
#     # Check the serialized field is the type we expect.
#     assert isinstance(serialized, rosfield_pytype)
#     # check the serialized value is the same as the value of that field in the original message
#     # We need the type conversion to deal with serialized object in different format than ros data (like string)
#     # we also need to deal with slots in case we have complex objects (only one level supported)
#     if rosfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type]:
#         assert serialized == pyfield
#     else:  # not a basic type for python
#         if pyfield_pytype in [int, six_long, float]:  # non verbatim basic fields
#             assert serialized == rosfield_pytype(secs=int(pyfield), nsecs=int(pyfield * 1e9 - int(pyfield) *1e9))
#         elif pyfield_pytype == list:
#             for idx, elem in enumerate(pyfield):
#                 assert serialized[idx] == elem
#         else:  #dict format can be used though...
#             assert serialized == rosfield_pytype(**pyfield)
#
#     # Building the ros message in case it changes something...
#     ros_msg = rosmsg_type(data=serialized)
#     deserialized = field.deserialize(ros_msg.data)
#
#     # Check the dict we obtain is the expected type and same value.
#     assert isinstance(deserialized, pyfield_pytype)
#     if pyfield_pytype not in [bool, int, six_long, float, six.binary_type, six.text_type, list]:
#         # If we were missing some fields, we need to initialise to default ROS value to be able to compare
#         for i, s in enumerate(ros_msg.data.__slots__):
#             if s not in pyfield.keys():
#                 pyfield[s] = ros_pythontype_mapping[ros_msg.data._slot_types[i]]()
#
#     assert deserialized == pyfield


# Just in case we run this directly
if __name__ == '__main__':
    pytest.main([
        'test_basic_fields.py::test_fromros',
        'test_basic_fields.py::test_frompy',
    ])
