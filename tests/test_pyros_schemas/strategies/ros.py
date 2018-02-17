from __future__ import absolute_import, print_function


import six
six_long = six.integer_types[-1]

import hypothesis
import hypothesis.strategies as st


import std_msgs.msg as std_msgs
import genpy

std_msgs_types_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    # Where there is no ambiguity, we can reuse std_msgs_dict_field_strat_ok strategies
    'std_msgs/Bool': st.builds(std_msgs.Bool, data=st.booleans()),
    'std_msgs/Int8': st.builds(std_msgs.Int8, data=st.integers(min_value=-128, max_value=127)), # CAREFUL in python booleans are integers
    'std_msgs/Int16': st.builds(std_msgs.Int16, data=st.integers(min_value=-32768, max_value=32767)),
    'std_msgs/Int32': st.builds(std_msgs.Int32, data=st.integers(min_value=-2147483648, max_value=2147483647)),
    'std_msgs/Int64': st.builds(std_msgs.Int64, data=st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807))),
    'std_msgs/UInt8': st.builds(std_msgs.UInt8, data=st.integers(min_value=0, max_value=255)),
    'std_msgs/UInt16': st.builds(std_msgs.UInt16, data=st.integers(min_value=0, max_value=65535)),
    'std_msgs/UInt32': st.builds(std_msgs.UInt32, data=st.integers(min_value=0, max_value=4294967295)),
    'std_msgs/UInt64': st.builds(std_msgs.UInt64, data=st.integers(min_value=0, max_value=six_long(18446744073709551615))),
    'std_msgs/Float32': st.builds(std_msgs.Float32, data=st.floats(min_value=-3.4028235e+38, max_value=3.4028235e+38)),
    'std_msgs/Float64': st.builds(std_msgs.Float64, data=st.floats(min_value=-1.7976931348623157e+308, max_value=1.7976931348623157e+308, )),
    'std_msgs/String': st.builds(std_msgs.String, data=st.text(alphabet=st.characters(max_codepoint=127))),
    'std_msgs/Time': st.builds(std_msgs.Time, data=st.one_of(
        # different ways to build a genpy.time (check genpy code)
        st.builds(genpy.Time, secs=st.integers(min_value=0, max_value=4294967295 -3), nsecs=st.integers(min_value=0, max_value=4294967295)),
        #st.builds(genpy.Time, nsecs=st.integers(min_value=six_long(0), max_value=six_long(9223372036854775807))),  # too slow for now (waiting on genpy patch)
        st.builds(genpy.Time, secs=st.floats(min_value=0, max_value=4294967295 -3, allow_infinity=False, allow_nan=False)),
    )),
    'std_msgs/Duration': st.builds(std_msgs.Duration, data=st.one_of(
        # different ways to build a genpy.duration (check genpy code)
        st.builds(genpy.Duration, secs=st.integers(min_value=-2147483648+1, max_value=2147483647-1), nsecs=st.integers(min_value=-2147483648, max_value=2147483647)),
        #st.builds(genpy.Duration, nsecs=st.integers(min_value=six_long(0), max_value=six_long(9223372036854775807))),  # to slow for now (waiting on genpy patch)
        st.builds(genpy.Duration, secs=st.floats(min_value=-2147483648+1, max_value=2147483647-1, allow_infinity=False, allow_nan=False)),
    )),
    # TODO : add more. we should test all.
}

std_msgs_dicts_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    # Where there is no ambiguity, we can reuse std_msgs_dict_field_strat_ok strategies
    'std_msgs/Bool': st.builds(dict, data=st.booleans()),
    'std_msgs/Int8': st.builds(dict, data=st.integers(min_value=-128, max_value=127)), # CAREFUL in python booleans are integers
    'std_msgs/Int16': st.builds(dict, data=st.integers(min_value=-32768, max_value=32767)),
    'std_msgs/Int32': st.builds(dict, data=st.integers(min_value=-2147483648, max_value=2147483647)),
    'std_msgs/Int64': st.builds(dict, data=st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807))),
    'std_msgs/UInt8': st.builds(dict, data=st.integers(min_value=0, max_value=255)),
    'std_msgs/UInt16': st.builds(dict, data=st.integers(min_value=0, max_value=65535)),
    'std_msgs/UInt32': st.builds(dict, data=st.integers(min_value=0, max_value=4294967295)),
    'std_msgs/UInt64': st.builds(dict, data=st.integers(min_value=0, max_value=six_long(18446744073709551615))),
    'std_msgs/Float32': st.builds(dict, data=st.floats(min_value=-3.4028235e+38, max_value=3.4028235e+38)),
    'std_msgs/Float64': st.builds(dict, data=st.floats(min_value=-1.7976931348623157e+308, max_value=1.7976931348623157e+308, )),
    'std_msgs/String': st.builds(dict, data=st.text(alphabet=st.characters(max_codepoint=127))),
    # 'std_msgs/Time': st.builds(dict, data=st.integers(min_value=six_long(0), max_value=six_long(18446744073709551615))),  # too long for now (waiting genpy patch)
    'std_msgs/Time': st.builds(dict, data=st.integers(min_value=six_long(0), max_value=4294967295)),  # TODO : extend this
    # 'std_msgs/Duration': st.builds(dict, data=st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807))),  # too long for now ( waiting genpy patch)
    'std_msgs/Duration': st.builds(dict, data=st.integers(min_value=-2147483648, max_value=2147483647)),  # TODO : extend this
    # TODO : add more. we should test all.
}

msg_types_data_ros_field_types = {
    # ( actual type, data field type string )
    'std_msgs/Bool': (std_msgs.Bool, 'bool'),
    'std_msgs/Int8': (std_msgs.Int8, 'int8'),
    'std_msgs/Int16': (std_msgs.Int16, 'int16'),
    'std_msgs/Int32': (std_msgs.Int32, 'int32'),
    'std_msgs/Int64': (std_msgs.Int64, 'int64'),
    'std_msgs/UInt8': (std_msgs.UInt8, 'uint8'),
    'std_msgs/UInt16': (std_msgs.UInt16, 'uint16'),
    'std_msgs/UInt32': (std_msgs.UInt32, 'uint32'),
    'std_msgs/UInt64': (std_msgs.UInt64, 'uint64'),
    'std_msgs/Float32': (std_msgs.Float32, 'float32'),
    'std_msgs/Float64': (std_msgs.Float64, 'float64'),
    'std_msgs/String': (std_msgs.String, 'string'),
    'std_msgs/Time': (std_msgs.Time, 'time'),
    'std_msgs/Duration': (std_msgs.Duration, 'duration'),
}


def rostype_from_rostypestring(rostypestring):
    return msg_types_data_ros_field_types.get(rostypestring)[0]

def fieldtypestring_from_rostypestring(rostypestring):
    return msg_types_data_ros_field_types.get(rostypestring)[1]




# Strategies for Optional types added by this package to ROS


from .. import msg as pyros_schemas_test_msgs

pyros_schemas_opttypes_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    # Where there is no ambiguity, we can reuse optfield_strat_ok strategies
    'pyros_schemas/test_opt_bool_as_array': st.builds(pyros_schemas_test_msgs.test_opt_bool_as_array, data=st.one_of(st.none(), st.booleans())),
    'pyros_schemas/test_opt_int8_as_array': st.builds(pyros_schemas_test_msgs.test_opt_int8_as_array, data=st.one_of(st.none(), st.integers(min_value=-128, max_value=127))),
    'pyros_schemas/test_opt_int16_as_array': st.builds(pyros_schemas_test_msgs.test_opt_int16_as_array, data=st.one_of(st.none(), st.integers(min_value=-32768, max_value=32767))),
    'pyros_schemas/test_opt_int32_as_array': st.builds(pyros_schemas_test_msgs.test_opt_int32_as_array, data=st.one_of(st.none(), st.integers(min_value=-2147483648, max_value=2147483647))),
    'pyros_schemas/test_opt_int64_as_array': st.builds(pyros_schemas_test_msgs.test_opt_int64_as_array, data=st.one_of(st.none(), st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807)))),
    'pyros_schemas/test_opt_uint8_as_array': st.builds(pyros_schemas_test_msgs.test_opt_uint8_as_array, data=st.one_of(st.none(), st.integers(min_value=0, max_value=255))),
    'pyros_schemas/test_opt_uint16_as_array': st.builds(pyros_schemas_test_msgs.test_opt_uint16_as_array, data=st.one_of(st.none(), st.integers(min_value=0, max_value=65535))),
    'pyros_schemas/test_opt_uint32_as_array': st.builds(pyros_schemas_test_msgs.test_opt_uint32_as_array, data=st.one_of(st.none(), st.integers(min_value=0, max_value=4294967295))),
    'pyros_schemas/test_opt_uint64_as_array': st.builds(pyros_schemas_test_msgs.test_opt_uint64_as_array, data=st.one_of(st.none(), st.integers(min_value=0, max_value=six_long(18446744073709551615)))),
    'pyros_schemas/test_opt_float32_as_array': st.builds(pyros_schemas_test_msgs.test_opt_float32_as_array, data=st.one_of(st.none(), st.floats(min_value=-3.4028235e+38, max_value=3.4028235e+38))),
    'pyros_schemas/test_opt_float64_as_array': st.builds(pyros_schemas_test_msgs.test_opt_float64_as_array, data=st.one_of(st.none(), st.floats(min_value=-1.7976931348623157e+308, max_value=1.7976931348623157e+308, ))),
    'pyros_schemas/test_opt_string_as_array': st.builds(pyros_schemas_test_msgs.test_opt_string_as_array, data=st.one_of(st.none(), st.text(alphabet=st.characters(max_codepoint=127)))),
    'pyros_schemas/test_opt_time_as_array': st.builds(pyros_schemas_test_msgs.test_opt_time_as_array, data=st.one_of(
        # different ways to build a genpy.time (check genpy code)
        st.builds(genpy.Time, secs=st.integers(min_value=0, max_value=4294967295 -3), nsecs=st.integers(min_value=0, max_value=4294967295)),
        #st.builds(genpy.Time, nsecs=st.integers(min_value=six_long(0), max_value=six_long(9223372036854775807))),  # too slow for now (waiting on genpy patch)
        st.builds(genpy.Time, secs=st.floats(min_value=0, max_value=4294967295 -3, allow_infinity=False, allow_nan=False)),  # TODO : extend this
    )),
    'pyros_schemas/test_opt_duration_as_array': st.builds(pyros_schemas_test_msgs.test_opt_duration_as_array, data=st.one_of(
        # different ways to build a genpy.duration (check genpy code)
        st.builds(genpy.Duration, secs=st.integers(min_value=-2147483648+1, max_value=2147483647-1), nsecs=st.integers(min_value=-2147483648, max_value=2147483647)),
        #st.builds(genpy.Duration, nsecs=st.integers(min_value=six_long(0), max_value=six_long(9223372036854775807))),  # to slow for now (waiting on genpy patch)
        st.builds(genpy.Duration, secs=st.floats(min_value=-2147483648+1, max_value=2147483647-1, allow_infinity=False, allow_nan=False)),  # TODO : extend this
    )),
    # TODO : add more. we should test all.
}


pyros_schemas_dicts_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    # Where there is no ambiguity, we can reuse std_msgs_dict_field_strat_ok strategies
    'pyros_schemas/test_opt_bool_as_array': st.builds(dict, data=st.one_of(st.none(), st.booleans())),
    'pyros_schemas/test_opt_int8_as_array': st.builds(dict, data=st.one_of(st.none(), st.integers(min_value=-128, max_value=127))),
    'pyros_schemas/test_opt_int16_as_array': st.builds(dict, data=st.one_of(st.none(), st.integers(min_value=-32768, max_value=32767))),
    'pyros_schemas/test_opt_int32_as_array': st.builds(dict, data=st.one_of(st.none(), st.integers(min_value=-2147483648, max_value=2147483647))),
    'pyros_schemas/test_opt_int64_as_array': st.builds(dict, data=st.one_of(st.none(), st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807)))),
    'pyros_schemas/test_opt_uint8_as_array': st.builds(dict, data=st.one_of(st.none(), st.integers(min_value=0, max_value=255))),
    'pyros_schemas/test_opt_uint16_as_array': st.builds(dict, data=st.one_of(st.none(), st.integers(min_value=0, max_value=65535))),
    'pyros_schemas/test_opt_uint32_as_array': st.builds(dict, data=st.one_of(st.none(), st.integers(min_value=0, max_value=4294967295))),
    'pyros_schemas/test_opt_uint64_as_array': st.builds(dict, data=st.one_of(st.none(), st.integers(min_value=0, max_value=six_long(18446744073709551615)))),
    'pyros_schemas/test_opt_float32_as_array': st.builds(dict, data=st.one_of(st.none(), st.floats(min_value=-3.4028235e+38, max_value=3.4028235e+38))),
    'pyros_schemas/test_opt_float64_as_array': st.builds(dict, data=st.one_of(st.none(), st.floats(min_value=-1.7976931348623157e+308, max_value=1.7976931348623157e+308, ))),
    'pyros_schemas/test_opt_string_as_array': st.builds(dict, data=st.one_of(st.none(), st.text(alphabet=st.characters(max_codepoint=127)))),
    # 'pyros_schemas/test_opt_time_as_array': st.builds(dict, data=st.integers(min_value=six_long(0), max_value=six_long(18446744073709551615))),  # too long for now (waiting genpy patch)
    'pyros_schemas/test_opt_time_as_array': st.builds(dict, data=st.integers(min_value=six_long(0), max_value=4294967295)),  # TODO : extend this
    # 'pyros_schemas/test_opt_duration_as_array': st.builds(dict, data=st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807))),  # too long for now ( waiting genpy patch)
    'pyros_schemas/test_opt_duration_as_array': st.builds(dict, data=st.integers(min_value=-2147483648 +1, max_value=2147483647)),  # TODO : extend this
    # TODO : add more. we should test all.
}


# TODO : add selfcheck / doctests
