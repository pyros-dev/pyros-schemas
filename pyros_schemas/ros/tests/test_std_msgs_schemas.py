from __future__ import absolute_import
from __future__ import print_function

try:
    import std_msgs.msg as std_msgs
#    import pyros_msgs.msg as pyros_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us ot the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs.msg as std_msgs
#    import pyros_msgs.msg as pyros_msgs


import nose
import marshmallow

# absolute import ros field types
from pyros_schemas.ros import (
    RosBool,
    RosInt8,
    RosString,
    RosOpt,
)

#absolute import ros std_msgs schemas
from pyros_schemas.ros import (
    RosMsgBool,
    RosMsgInt8,
    RosMsgString,
)

from pyros_schemas.ros import with_explicitly_matched_type


@nose.tools.nottest
def gen_rosmsg_test(schemaType, ros_msg, py_inst_expected):
    schema = schemaType()

    marshalled, errors = schema.dump(ros_msg)
    assert not errors and marshalled == py_inst_expected

    value, errors = schema.load(marshalled)
    assert not errors and type(value) == type(ros_msg) and value == ros_msg


@nose.tools.nottest
def gen_pymsg_test(schemaType, ros_msg_expected, py_inst):

    schema = schemaType()

    unmarshalled, errors = schema.load(py_inst)
    assert not errors and type(unmarshalled) == type(ros_msg_expected) and unmarshalled == ros_msg_expected

    obj, errors = schema.dump(unmarshalled)
    assert not errors and type(obj) == type(py_inst) and obj == py_inst


def test_msgbool_ros():
    yield gen_rosmsg_test, RosMsgBool, std_msgs.Bool(data=True), {'data': True}
    yield gen_rosmsg_test, RosMsgBool, std_msgs.Bool(data=False), {'data': False}


def test_msgbool_py():
    yield gen_pymsg_test, RosMsgBool, std_msgs.Bool(data=True), {'data': True}
    yield gen_pymsg_test, RosMsgBool, std_msgs.Bool(data=False), {'data': False}


def test_msgint8_ros():
    yield gen_rosmsg_test, RosMsgInt8, std_msgs.Int8(data=42), {'data': 42}


def test_msgint8_py():
    yield gen_pymsg_test, RosMsgInt8, std_msgs.Int8(data=42), {'data': 42}

# TODO : test other ints


def test_msgstring_ros():
    yield gen_rosmsg_test, RosMsgString, std_msgs.String(data='fortytwo'), {'data': u'fortytwo'}


def test_msgstring_py():
    yield gen_pymsg_test, RosMsgString, std_msgs.String(data='fortytwo'), {'data': u'fortytwo'}


#
# Also testing similar schemas but with optional fields to validate optional field implementation
#

@with_explicitly_matched_type(pyros_msgs.opt_bool)
class RosMsgOptBool(marshmallow.Schema):
    """
    Building a optional bool message for testing
    """
    data = RosOpt(RosBool())


def test_msgoptbool_ros():
    yield gen_rosmsg_test, RosMsgOptBool, pyros_msgs.opt_bool(data=[True]), {'data': True}
    yield gen_rosmsg_test, RosMsgOptBool, pyros_msgs.opt_bool(data=[False]), {'data': False}
    yield gen_rosmsg_test, RosMsgOptBool, pyros_msgs.opt_bool(data=[]), {}
    yield gen_rosmsg_test, RosMsgOptBool, pyros_msgs.opt_bool(), {}


def test_msgoptbool_py():
    yield gen_pymsg_test, RosMsgOptBool, pyros_msgs.opt_bool(data=[True]), {'data': True}
    yield gen_pymsg_test, RosMsgOptBool, pyros_msgs.opt_bool(data=[False]), {'data': False}
    yield gen_pymsg_test, RosMsgOptBool, pyros_msgs.opt_bool(data=[]), {}
    yield gen_pymsg_test, RosMsgOptBool, pyros_msgs.opt_bool(), {}


@with_explicitly_matched_type(pyros_msgs.opt_int8)
class RosMsgOptInt8(marshmallow.Schema):
    """
    Building a optional int8 message for testing
    """
    data = RosOpt(RosInt8())


def test_msgoptint8_ros():
    yield gen_rosmsg_test, RosMsgOptInt8, pyros_msgs.opt_int8(data=[42]), {'data': 42}
    yield gen_rosmsg_test, RosMsgOptInt8, pyros_msgs.opt_int8(data=[]), {}
    yield gen_rosmsg_test, RosMsgOptInt8, pyros_msgs.opt_int8(), {}


def test_msgoptint8_py():
    yield gen_pymsg_test, RosMsgOptInt8, pyros_msgs.opt_int8(data=[42]), {'data': 42}
    yield gen_pymsg_test, RosMsgOptInt8, pyros_msgs.opt_int8(data=[]), {}
    yield gen_pymsg_test, RosMsgOptInt8, pyros_msgs.opt_int8(), {}

# TODO : test other ints


@with_explicitly_matched_type(pyros_msgs.opt_string)
class RosMsgOptString(marshmallow.Schema):
    """
    Building a optional int8 message for testing
    """
    data = RosOpt(RosString())


def test_msgstring_ros():
    yield gen_rosmsg_test, RosMsgOptString, pyros_msgs.opt_string(data=['fortytwo']), {'data': u'fortytwo'}
    yield gen_rosmsg_test, RosMsgOptString, pyros_msgs.opt_string(data=['']), {'data': ''}
    yield gen_rosmsg_test, RosMsgOptString, pyros_msgs.opt_string(data=[]), {}
    yield gen_rosmsg_test, RosMsgOptString, pyros_msgs.opt_string(), {}


def test_msgstring_py():
    yield gen_pymsg_test, RosMsgOptString, pyros_msgs.opt_string(data=['fortytwo']), {'data': u'fortytwo'}
    yield gen_pymsg_test, RosMsgOptString, pyros_msgs.opt_string(data=['']), {'data': ''}
    yield gen_pymsg_test, RosMsgOptString, pyros_msgs.opt_string(data=[]), {}
    yield gen_pymsg_test, RosMsgOptString, pyros_msgs.opt_string(), {}

# Just in case we run this directly
if __name__ == '__main__':
    import nose
    nose.runmodule(__name__)
