from __future__ import absolute_import, division, print_function, unicode_literals

try:
    import std_msgs.msg as std_msgs
    import genpy
    import pyros_msgs.opt_as_array
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs.msg as std_msgs
    import genpy
    import pyros_msgs.opt_as_array

import hypothesis
import hypothesis.strategies as st

import six
six_long = six.integer_types[-1]


def maybe_list(l):
    """Return list of one element if ``l`` is a scalar."""
    return l if l is None or isinstance(l, list) else [l]


# For now We use a set of basic messages for testing
std_msgs_field_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    'std_msgs/Bool': st.booleans(),
    'std_msgs/Int8': st.integers(min_value=-128, max_value=127),  # in python booleans are integers
    'std_msgs/Int16': st.integers(min_value=-32768, max_value=32767),
    'std_msgs/Int32': st.integers(min_value=-2147483648, max_value=2147483647),
    'std_msgs/Int64': st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807)),
    'std_msgs/UInt8': st.integers(min_value=0, max_value=255),
    'std_msgs/UInt16': st.integers(min_value=0, max_value=65535),
    'std_msgs/UInt32': st.integers(min_value=0, max_value=4294967295),
    'std_msgs/UInt64': st.integers(min_value=0, max_value=six_long(18446744073709551615)),
    'std_msgs/Float32': st.floats(min_value=-3.4028235e+38, max_value=3.4028235e+38),
    'std_msgs/Float64': st.floats(min_value=-1.7976931348623157e+308, max_value=1.7976931348623157e+308, ),
    #'std_msgs/String': st.one_of(st.binary(), st.text(alphabet=st.characters(max_codepoint=127))),
    #'std_msgs/String': st.binary(),  # this makes hypothesis crash on reporting (0x80 not valid in starting position : cannot be decoded with utf8)
    'std_msgs/String': st.text(alphabet=st.characters(max_codepoint=127)),
    'std_msgs/Time':
        # only one way to build a python data for a time message
        st.integers(min_value=six_long(0), max_value=six_long(18446744073709551615)),
    'std_msgs/Duration':
        # only one way to build a python data for a duration message
        st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807)),
    # TODO : add more. we should test all.
}

# For now We use a set of basic messages for testing
pyros_msgs_optfield_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    'pyros_msgs/test_opt_bool_as_array': st.one_of(st.none(), st.booleans()),
    'pyros_msgs/test_opt_int8_as_array': st.one_of(st.none(), st.integers(min_value=-128, max_value=127)),  # in python booleans are integers
    'pyros_msgs/test_opt_int16_as_array': st.one_of(st.none(), st.integers(min_value=-32768, max_value=32767)),
    'pyros_msgs/test_opt_int32_as_array': st.one_of(st.none(), st.integers(min_value=-2147483648, max_value=2147483647)),
    'pyros_msgs/test_opt_int64_as_array': st.one_of(st.none(), st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807))),
    'pyros_msgs/test_opt_uint8_as_array': st.one_of(st.none(), st.integers(min_value=0, max_value=255)),
    'pyros_msgs/test_opt_uint16_as_array': st.one_of(st.none(), st.integers(min_value=0, max_value=65535)),
    'pyros_msgs/test_opt_uint32_as_array': st.one_of(st.none(), st.integers(min_value=0, max_value=4294967295)),
    'pyros_msgs/test_opt_uint64_as_array': st.one_of(st.none(), st.integers(min_value=0, max_value=six_long(18446744073709551615))),
    'pyros_msgs/test_opt_float32_as_array': st.one_of(st.none(), st.floats(min_value=-3.4028235e+38, max_value=3.4028235e+38)),
    'pyros_msgs/test_opt_float64_as_array': st.one_of(st.none(), st.floats(min_value=-1.7976931348623157e+308, max_value=1.7976931348623157e+308, )),
    #'pyros_msgs/test_opt_string_as_array': st.one_of(st.none(), st.binary(), st.text(alphabet=st.characters(max_codepoint=127))),
    #'pyros_msgs/test_opt_string_as_array': st.one_of(st.none(), st.binary()),  # this makes hypothesis crash on reporting (0x80 not valid in starting position : cannot be decoded with utf8)
    'pyros_msgs/test_opt_string_as_array': st.one_of(st.none(), st.text(alphabet=st.characters(max_codepoint=127))),
    'pyros_msgs/test_opt_time_as_array':
        # only one way to build a python data for a time message
        st.one_of(st.none(), st.integers(min_value=six_long(0), max_value=six_long(4294967295999999999))),  # maximum time expressible in python with ROS serialization
    'pyros_msgs/test_opt_duration_as_array':
        # only one way to build a python data for a duration message
        st.one_of(st.none(), st.integers(min_value=-six_long(2147483648999999999), max_value=six_long(2147483647999999999))),  # maximum duration expressible in python with ROS serialization
    # TODO : add more. we should test all.
}

std_msgs_types_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    # Where there is no ambiguity, we can reuse std_msgs_dict_field_strat_ok strategies
    'std_msgs/Bool': st.builds(std_msgs.Bool, data=std_msgs_field_strat_ok.get('std_msgs/Bool')),
    'std_msgs/Int8': st.builds(std_msgs.Int8, data=std_msgs_field_strat_ok.get('std_msgs/Int8')),
    'std_msgs/Int16': st.builds(std_msgs.Int16, data=std_msgs_field_strat_ok.get('std_msgs/Int16')),
    'std_msgs/Int32': st.builds(std_msgs.Int32, data=std_msgs_field_strat_ok.get('std_msgs/Int32')),
    'std_msgs/Int64': st.builds(std_msgs.Int64, data=std_msgs_field_strat_ok.get('std_msgs/Int64')),
    'std_msgs/UInt8': st.builds(std_msgs.UInt8, data=std_msgs_field_strat_ok.get('std_msgs/UInt8')),
    'std_msgs/UInt16': st.builds(std_msgs.UInt16, data=std_msgs_field_strat_ok.get('std_msgs/UInt16')),
    'std_msgs/UInt32': st.builds(std_msgs.UInt32, data=std_msgs_field_strat_ok.get('std_msgs/UInt32')),
    'std_msgs/UInt64': st.builds(std_msgs.UInt64, data=std_msgs_field_strat_ok.get('std_msgs/UInt64')),
    'std_msgs/Float32': st.builds(std_msgs.Float32, data=std_msgs_field_strat_ok.get('std_msgs/Float32')),
    'std_msgs/Float64': st.builds(std_msgs.Float64, data=std_msgs_field_strat_ok.get('std_msgs/Float64')),
    'std_msgs/String': st.builds(std_msgs.String, data=std_msgs_field_strat_ok.get('std_msgs/String')),
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


pyros_msgs_opttypes_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    # Where there is no ambiguity, we can reuse std_msgs_dict_field_strat_ok strategies
    'pyros_msgs/test_opt_bool_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_bool_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_bool_as_array')),
    'pyros_msgs/test_opt_int8_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_int8_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_int8_as_array')),
    'pyros_msgs/test_opt_int16_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_int16_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_int16_as_array')),
    'pyros_msgs/test_opt_int32_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_int32_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_int32_as_array')),
    'pyros_msgs/test_opt_int64_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_int64_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_int64_as_array')),
    'pyros_msgs/test_opt_uint8_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_uint8_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_uint8_as_array')),
    'pyros_msgs/test_opt_uint16_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_uint16_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_uint16_as_array')),
    'pyros_msgs/test_opt_uint32_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_uint32_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_uint32_as_array')),
    'pyros_msgs/test_opt_uint64_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_uint64_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_uint64_as_array')),
    'pyros_msgs/test_opt_float32_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_float32_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_float32_as_array')),
    'pyros_msgs/test_opt_float64_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_float64_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_float64_as_array')),
    'pyros_msgs/test_opt_string_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_string_as_array, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_string_as_array')),
    'pyros_msgs/test_opt_time_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_time_as_array, data=st.one_of(
        # different ways to build a genpy.time (check genpy code)
        st.builds(genpy.Time, secs=st.integers(min_value=0, max_value=4294967295 -3), nsecs=st.integers(min_value=0, max_value=4294967295)),
        #st.builds(genpy.Time, nsecs=st.integers(min_value=six_long(0), max_value=six_long(9223372036854775807))),  # too slow for now (waiting on genpy patch)
        st.builds(genpy.Time, secs=st.floats(min_value=0, max_value=4294967295 -3, allow_infinity=False, allow_nan=False)),  # TODO : extend this
    )),
    'pyros_msgs/test_opt_duration_as_array': st.builds(pyros_msgs.opt_as_array.test_opt_duration_as_array, data=st.one_of(
        # different ways to build a genpy.duration (check genpy code)
        st.builds(genpy.Duration, secs=st.integers(min_value=-2147483648+1, max_value=2147483647-1), nsecs=st.integers(min_value=-2147483648, max_value=2147483647)),
        #st.builds(genpy.Duration, nsecs=st.integers(min_value=six_long(0), max_value=six_long(9223372036854775807))),  # to slow for now (waiting on genpy patch)
        st.builds(genpy.Duration, secs=st.floats(min_value=-2147483648+1, max_value=2147483647-1, allow_infinity=False, allow_nan=False)),  # TODO : extend this
    )),
    # TODO : add more. we should test all.
}

std_msgs_dicts_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    # Where there is no ambiguity, we can reuse std_msgs_dict_field_strat_ok strategies
    'std_msgs/Bool': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/Bool')),
    'std_msgs/Int8': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/Int8')),
    'std_msgs/Int16': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/Int16')),
    'std_msgs/Int32': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/Int32')),
    'std_msgs/Int64': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/Int64')),
    'std_msgs/UInt8': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/UInt8')),
    'std_msgs/UInt16': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/UInt16')),
    'std_msgs/UInt32': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/UInt32')),
    'std_msgs/UInt64': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/UInt64')),
    'std_msgs/Float32': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/Float32')),
    'std_msgs/Float64': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/Float64')),
    'std_msgs/String': st.builds(dict, data=std_msgs_field_strat_ok.get('std_msgs/String')),
    # 'std_msgs/Time': st.builds(dict, data=st.integers(min_value=six_long(0), max_value=six_long(18446744073709551615))),  # too long for now (waiting genpy patch)
    'std_msgs/Time': st.builds(dict, data=st.integers(min_value=six_long(0), max_value=4294967295)),  # TODO : extend this
    # 'std_msgs/Duration': st.builds(dict, data=st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807))),  # too long for now ( waiting genpy patch)
    'std_msgs/Duration': st.builds(dict, data=st.integers(min_value=-2147483648, max_value=2147483647)),  # TODO : extend this
    # TODO : add more. we should test all.
}


pyros_msgs_optdicts_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    # Where there is no ambiguity, we can reuse std_msgs_dict_field_strat_ok strategies
    'pyros_msgs/test_opt_bool_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_bool_as_array')),
    'pyros_msgs/test_opt_int8_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_int8_as_array')),
    'pyros_msgs/test_opt_int16_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_int16_as_array')),
    'pyros_msgs/test_opt_int32_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_int32_as_array')),
    'pyros_msgs/test_opt_int64_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_int64_as_array')),
    'pyros_msgs/test_opt_uint8_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_uint8_as_array')),
    'pyros_msgs/test_opt_uint16_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_uint16_as_array')),
    'pyros_msgs/test_opt_uint32_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_uint32_as_array')),
    'pyros_msgs/test_opt_uint64_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_uint64_as_array')),
    'pyros_msgs/test_opt_float32_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_float32_as_array')),
    'pyros_msgs/test_opt_float64_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_float64_as_array')),
    'pyros_msgs/test_opt_string_as_array': st.builds(dict, data=pyros_msgs_optfield_strat_ok.get('pyros_msgs/test_opt_string_as_array')),
    # 'pyros_msgs/test_opt_time_as_array': st.builds(dict, data=st.integers(min_value=six_long(0), max_value=six_long(18446744073709551615))),  # too long for now (waiting genpy patch)
    'pyros_msgs/test_opt_time_as_array': st.builds(dict, data=st.integers(min_value=six_long(0), max_value=4294967295)),  # TODO : extend this
    # 'pyros_msgs/test_opt_duration_as_array': st.builds(dict, data=st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807))),  # too long for now ( waiting genpy patch)
    'pyros_msgs/test_opt_duration_as_array': st.builds(dict, data=st.integers(min_value=-2147483648 +1, max_value=2147483647)),  # TODO : extend this
    # TODO : add more. we should test all.
}


def proper_basic_msg_strategy_selector(*msg_types):
    """Accept a (list of) rostype and return it with the matching strategy for ros message"""
    # TODO : break on error (type not in map)
    # we use a list comprehension here to avoid creating a generator (tuple comprehension)
    return tuple([(msg_type, std_msgs_types_strat_ok.get(msg_type)) for msg_type in msg_types])


def proper_basic_dict_strategy_selector(*msg_types):
    """Accept a (list of) rostype and return it with the matching strategy for dict"""
    # TODO : break on error (type not in map)
    # we use a list comprehension here to avoid creating a generator (tuple comprehension)
    return tuple([(msg_type, std_msgs_dicts_strat_ok.get(msg_type)) for msg_type in msg_types])


def proper_basic_data_strategy_selector(*msg_types):
    """Accept a (list of) rostype and return it with the matching strategy for data"""
    # TODO : break on error (type not in map)
    # we use a list comprehension here to avoid creating a generator (tuple comprehension)
    return tuple([(msg_type, std_msgs_field_strat_ok.get(msg_type)) for msg_type in msg_types])


def proper_basic_optmsg_strategy_selector(*msg_types):
    """Accept a (list of) rostype and return it with the matching strategy for ros message"""
    # TODO : break on error (type not in map)
    # we use a list comprehension here to avoid creating a generator (tuple comprehension)
    return tuple([(msg_type, pyros_msgs_opttypes_strat_ok.get(msg_type)) for msg_type in msg_types])


def proper_basic_optdict_strategy_selector(*msg_types):
    """Accept a (list of) rostype and return it with the matching strategy for dict"""
    # TODO : break on error (type not in map)
    # we use a list comprehension here to avoid creating a generator (tuple comprehension)
    return tuple([(msg_type, pyros_msgs_optdicts_strat_ok.get(msg_type)) for msg_type in msg_types])


def proper_basic_optdata_strategy_selector(*msg_types):
    """Accept a (list of) rostype and return it with the matching strategy for data"""
    # TODO : break on error (type not in map)
    # we use a list comprehension here to avoid creating a generator (tuple comprehension)
    return tuple([(msg_type, pyros_msgs_optfield_strat_ok.get(msg_type)) for msg_type in msg_types])


# simple way to define mapping between ros types and deserialized dictionary for testing
# since we are using mostly same message structure for std_msgs and pyros_msgs, we can combine those
def std_msgs_dicts_from_rostype_map(msg_type, rostype_value):
    if msg_type in (
        'std_msgs/Bool',
        'std_msgs/Int8', 'std_msgs/Int16', 'std_msgs/Int32', 'std_msgs/Int64',
        'std_msgs/UInt8', 'std_msgs/UInt16', 'std_msgs/UInt32', 'std_msgs/UInt64',

        'pyros_msgs/test_opt_bool_as_array',
        'pyros_msgs/test_opt_int8_as_array', 'pyros_msgs/test_opt_int16_as_array', 'pyros_msgs/test_opt_int32_as_array', 'pyros_msgs/test_opt_int64_as_array',
        'pyros_msgs/test_opt_uint8_as_array', 'pyros_msgs/test_opt_uint16_as_array', 'pyros_msgs/test_opt_uint32_as_array', 'pyros_msgs/test_opt_uint64_as_array',
    ):
        return {'data': rostype_value.data}
    elif msg_type in (
        'std_msgs/Float32', 'std_msgs/Float64',

        'pyros_msgs/test_opt_float32_as_array', 'pyros_msgs/test_opt_float64_as_array',
    ):
        return {'data': rostype_value.data}
    elif msg_type in (
        'std_msgs/String',

        'pyros_msgs/test_opt_string_as_array',
    ):
        # no need to decode/encode here but be careful about non-printable control characters...
        # Ref : http://www.madore.org/~david/computers/unicode/#faq_ascii
        return {'data': rostype_value.data}
    elif msg_type in (
        'std_msgs/Time', 'std_msgs/Duration',

        'pyros_msgs/test_opt_time_as_array', 'pyros_msgs/test_opt_duration_as_array'
    ):
        return {'data': rostype_value.data.to_nsec()}


# simple way to define mapping between dictionary and serialized rostype for testing
# since we are using mostly same message structure for std_msgs and pyros_msgs, we can combine those
def std_msgs_rostypes_from_dict_map(msg_type, dict_value):
    if msg_type in (
        'std_msgs/Bool',
        'std_msgs/Int8', 'std_msgs/Int16', 'std_msgs/Int32', 'std_msgs/Int64',
        'std_msgs/UInt8', 'std_msgs/UInt16', 'std_msgs/UInt32', 'std_msgs/UInt64',

        'pyros_msgs/test_opt_bool_as_array',
        'pyros_msgs/test_opt_int8_as_array', 'pyros_msgs/test_opt_int16_as_array', 'pyros_msgs/test_opt_int32_as_array', 'pyros_msgs/test_opt_int64_as_array',
        'pyros_msgs/test_opt_uint8_as_array', 'pyros_msgs/test_opt_uint16_as_array', 'pyros_msgs/test_opt_uint32_as_array', 'pyros_msgs/test_opt_uint64_as_array',
    ):
        rostype = genpy.message.get_message_class(msg_type)
        return rostype(data=dict_value.get('data'))
    elif msg_type in (
        'std_msgs/Float32', 'std_msgs/Float64',

        'pyros_msgs/test_opt_float32_as_array', 'pyros_msgs/test_opt_float64_as_array',
    ):
        rostype = genpy.message.get_message_class(msg_type)
        return rostype(data=dict_value.get('data'))
    elif msg_type in (
        'std_msgs/String',

        'pyros_msgs/test_opt_string_as_array',
    ):
        rostype = genpy.message.get_message_class(msg_type)
        return rostype(data=dict_value.get('data'))  # careful about non-printable control characters
    elif msg_type in (
        'std_msgs/Time',

        'pyros_msgs/test_opt_time_as_array',

    ):
        rostype = genpy.message.get_message_class(msg_type)
        return rostype(data=genpy.Time(nsecs=dict_value.get('data')))
    elif msg_type in (
        'std_msgs/Duration',

        'pyros_msgs/test_opt_duration_as_array',
    ):
        rostype = genpy.message.get_message_class(msg_type)
        return rostype(data=genpy.Duration(nsecs=dict_value.get('data')))

