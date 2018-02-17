from __future__ import absolute_import, print_function


import six
six_long = six.integer_types[-1]

import hypothesis
import hypothesis.strategies as st

import genpy



# For now We use a set of basic messages for testing
field_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    'bool': st.booleans(),
    'int8': st.integers(min_value=-128, max_value=127),  # in python booleans are integers
    'int16': st.integers(min_value=-32768, max_value=32767),
    'int32': st.integers(min_value=-2147483648, max_value=2147483647),
    'int64': st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807)),
    'uint8': st.integers(min_value=0, max_value=255),
    'uint16': st.integers(min_value=0, max_value=65535),
    'uint32': st.integers(min_value=0, max_value=4294967295),
    'uint64': st.integers(min_value=0, max_value=six_long(18446744073709551615)),
    'float32': st.floats(min_value=-3.4028235e+38, max_value=3.4028235e+38),
    'float64': st.floats(min_value=-1.7976931348623157e+308, max_value=1.7976931348623157e+308, ),
    #'std_msgs/String': st.one_of(st.binary(), st.text(alphabet=st.characters(max_codepoint=127))),
    #'std_msgs/String': st.binary(),  # this makes hypothesis crash on reporting (0x80 not valid in starting position : cannot be decoded with utf8)
    'string': st.text(alphabet=st.characters(max_codepoint=127)),
    'time':
        # only one way to build a python data for a time message
        st.integers(min_value=six_long(0), max_value=six_long(18446744073709551615)),
    'duration':
        # only one way to build a python data for a duration message
        st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807)),
    # TODO : add more. we should test all.
}



def proper_basic_data_strategy_selector(*field_types):
    """Accept a (list of) rostype and return it with the matching strategy for data"""
    # TODO : break on error (type not in map)
    # we use a list comprehension here to avoid creating a generator (tuple comprehension)
    return tuple([(field_type, field_strat_ok.get(field_type)) for field_type in field_types])


# We need a composite strategy to link slot type and slot value
@st.composite
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1)
def msg_rostype_and_value(draw, msgs_type_strat_tuples):
    msg_type_strat = draw(st.sampled_from(msgs_type_strat_tuples))
    msg_value = draw(msg_type_strat[1])
    return msg_type_strat[0], msg_value




# For now We use a set of basic messages for testing
optfield_strat_ok = {
    # in python, booleans are integer type, but we dont want to test that here.
    'optbool': st.one_of(st.none(), st.booleans()),
    'optint8': st.one_of(st.none(), st.integers(min_value=-128, max_value=127)),  # in python booleans are integers
    'optint16': st.one_of(st.none(), st.integers(min_value=-32768, max_value=32767)),
    'optint32': st.one_of(st.none(), st.integers(min_value=-2147483648, max_value=2147483647)),
    'optint64': st.one_of(st.none(), st.integers(min_value=-six_long(9223372036854775808), max_value=six_long(9223372036854775807))),
    'optuint8': st.one_of(st.none(), st.integers(min_value=0, max_value=255)),
    'optuint16': st.one_of(st.none(), st.integers(min_value=0, max_value=65535)),
    'optuint32': st.one_of(st.none(), st.integers(min_value=0, max_value=4294967295)),
    'optuint64': st.one_of(st.none(), st.integers(min_value=0, max_value=six_long(18446744073709551615))),
    'optfloat32': st.one_of(st.none(), st.floats(min_value=-3.4028235e+38, max_value=3.4028235e+38)),
    'optfloat64': st.one_of(st.none(), st.floats(min_value=-1.7976931348623157e+308, max_value=1.7976931348623157e+308, )),
    #'pyros_schemas/test_opt_string_as_array': st.one_of(st.none(), st.binary(), st.text(alphabet=st.characters(max_codepoint=127))),
    #'pyros_schemas/test_opt_string_as_array': st.one_of(st.none(), st.binary()),  # this makes hypothesis crash on reporting (0x80 not valid in starting position : cannot be decoded with utf8)
    'optstring': st.one_of(st.none(), st.text(alphabet=st.characters(max_codepoint=127))),
    'opttime':
        # only one way to build a python data for a time message
        st.one_of(st.none(), st.integers(min_value=six_long(0), max_value=six_long(4294967295999999999))),  # maximum time expressible in python with ROS serialization
    'optduration':
        # only one way to build a python data for a duration message
        st.one_of(st.none(), st.integers(min_value=-six_long(2147483648999999999), max_value=six_long(2147483647999999999))),  # maximum duration expressible in python with ROS serialization
    # TODO : add more. we should test all.
}


# TODO : add selfcheck / doctests