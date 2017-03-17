from __future__ import absolute_import
from __future__ import print_function

try:
    import rospy
    import std_msgs.msg as std_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us ot the proper distro
    pyros_setup.configurable_import().configure().activate()
    import rospy
    import std_msgs.msg as std_msgs

import nose

from pyros_schemas.ros.schemagic import create

# TODO Property based testing
# import hypothesis


@nose.tools.nottest
def gen_rosmsg_test(schema, ros_msg, py_inst_expected):

    marshalled, errors = schema.load(ros_msg)
    assert not errors and marshalled == py_inst_expected

    value, errors = schema.dump(marshalled)
    assert not errors and type(value) == type(ros_msg) and value == ros_msg

@nose.tools.nottest
def gen_pymsg_test(schema, ros_msg_expected, py_inst):

    unmarshalled, errors = schema.dump(py_inst)
    assert not errors and type(unmarshalled) == type(ros_msg_expected) and unmarshalled == ros_msg_expected

    obj, errors = schema.load(unmarshalled)
    assert not errors and type(obj) == type(py_inst) and obj == py_inst


# TODO :
#     # MultiArrayDimension
#     (std_msgs.msg.MultiArrayDimension(label=, size=, stride=), RosBool, bool, bool, bool),

def test_msgbool_ros():
    yield gen_rosmsg_test, create(std_msgs.Bool), std_msgs.Bool(data=True), {'data': True}
    yield gen_rosmsg_test, create(std_msgs.Bool), std_msgs.Bool(data=False), {'data': False}

def test_msgbool_py():
    yield gen_pymsg_test, create(std_msgs.Bool), std_msgs.Bool(data=True), {'data': True}
    yield gen_pymsg_test, create(std_msgs.Bool), std_msgs.Bool(data=False), {'data': False}

def test_msgint8_ros():
    yield gen_rosmsg_test, create(std_msgs.Int8), std_msgs.Int8(data=42), {'data': 42}

def test_msgint8_py():
    yield gen_pymsg_test, create(std_msgs.Int8), std_msgs.Int8(data=42), {'data': 42}

# TODO : test other ints


def test_msgstring_ros():
    yield gen_rosmsg_test, create(std_msgs.String), std_msgs.String(data='fortytwo'), {'data': u'fortytwo'}

def test_msgstring_py():
    yield gen_pymsg_test, create(std_msgs.String), std_msgs.String(data='fortytwo'), {'data': u'fortytwo'}


def test_msgtime_ros():
    yield gen_rosmsg_test, create(std_msgs.Time), std_msgs.Time(rospy.Time(secs=42, nsecs=31)), {'data': six_long(42000000031)}

def test_msgtime_py():
    yield gen_pymsg_test, create(std_msgs.Time), std_msgs.Time(rospy.Time(secs=42, nsecs=31)), {'data': six_long(42000000031)}


def test_msgduration_ros():
    yield gen_rosmsg_test, create(std_msgs.Duration), std_msgs.Duration(rospy.Duration(secs=42, nsecs=31)), {'data': six_long(42000000031)}

def test_msgduration_py():
    yield gen_pymsg_test, create(std_msgs.Duration), std_msgs.Duration(rospy.Duration(secs=42, nsecs=31)), {'data': six_long(42000000031)}

# Just in case we run this directly
if __name__ == '__main__':
    nose.runmodule(__name__)
