import hypothesis
import hypothesis.strategies as st
import six

"""
This is the gist of our hypothesis tests for pyros_schemas
"""


class MyClass(object):
    def __init__(self, data):
        self.data = data

    def __repr__(self):
        return "MyClass {0}".format(self.data)


class MyNestedClass(object):
    def __init__(self, nested):
        self.nested = nested

    def __repr__(self):
        return "MyNestedClass {0}".format(self.nested)


type_strats = {
    'bool': st.builds(MyClass, data=st.booleans()),
    'int': st.builds(MyClass, data=st.integers(min_value=-128, max_value=127)),
    'float': st.builds(MyClass, data=st.floats()),
    'nested': st.builds(MyClass, data=st.builds(MyNestedClass, nested=st.one_of(
        st.booleans(),
        st.integers(),
        st.floats(),
    ))),
}


def strategy_selector(*msg_types):
    """Accept a (list of) type strings and return it with the matching strategy"""
    # TODO : break on error (type not in map)
    # we use a list comprehension here to avoid creating a generator (tuple comprehension)
    return tuple([(msg_type, type_strats.get(msg_type)) for msg_type in msg_types])


@st.composite
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1)
def type_and_value(draw, msgs_type_strat_tuples):
    msg_type_strat = draw(st.sampled_from(msgs_type_strat_tuples))
    msg_value = draw(msg_type_strat[1])
    return msg_type_strat[0], msg_value


@hypothesis.given(type_and_value(strategy_selector(
    'bool', 'int', 'float', 'nested'
)))
@hypothesis.settings(verbosity=hypothesis.Verbosity.verbose, timeout=1)
def test_proper_type(msg_rostype_and_value):
    mtype = msg_rostype_and_value[0]
    mvalue = msg_rostype_and_value[1]

    assert isinstance(mvalue, MyClass)
    if mtype == 'bool':
        assert isinstance(mvalue.data, bool)
    elif mtype == 'int':
        assert isinstance(mvalue.data, six.integer_types)
    elif mtype == 'float':
        assert isinstance(mvalue.data, float)
    elif mtype == 'nested':
        assert isinstance(mvalue.data, MyNestedClass)
