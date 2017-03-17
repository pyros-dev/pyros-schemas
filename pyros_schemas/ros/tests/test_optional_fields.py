from __future__ import absolute_import
from __future__ import print_function

import pytest

# for py2 / py3 compatibility
import six
six_long = six.integer_types[-1]


try:
    import std_msgs
    import genpy
    import rospy
    import pyros_msgs.opt_as_array  # This will duck punch the standard message type initialization code.
    from pyros_msgs.msg import test_opt_bool_as_array  # a message type just for testing
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us ot the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs
    import genpy
    import rospy
    import pyros_msgs.opt_as_array  # This will duck punch the standard message type initialization code.
    from pyros_msgs.msg import test_opt_bool_as_array  # a message type just for testing
    from pyros_msgs.msg import test_opt_float32_as_array  # a message type just for testing
    # TODO : all of them

# patching
pyros_msgs.opt_as_array.duck_punch(test_opt_bool_as_array, ['data'])


import marshmallow.utils

# absolute import ros field types
from pyros_schemas.ros.basic_fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString, RosTextString,
)
from pyros_schemas.ros.optional_fields import (
    RosOptAsList,
    RosOptAsNested,  # TODO : optional message as Nested
)

from pyros_schemas.ros.types_mapping import (
    ros_msgtype_mapping,
    ros_pythontype_mapping
)


#
# Test functions, called via test generator
#

@pytest.mark.parametrize("msg, schema_field_type, in_rosfield_pytype, dictfield_pytype, out_rosfield_pytype", [
    # Bool
    # basic exmple : (std_msgs.msg.Bool(data=True), RosBool, bool, bool, bool),
    (test_opt_bool_as_array(data=[True]), lambda: RosOptAsList(RosBool()), list, bool, list),
    (test_opt_bool_as_array(data=[False]), lambda: RosOptAsList(RosBool()), list, bool, list),
    # also test [], None and default value
    (test_opt_bool_as_array(data=[]), lambda: RosOptAsList(RosBool()), list, bool, list),
    # Raises AttributeError Reason: pyros_msgs checks for data value to have proper type
    #pytest.mark.xfail(strict=True, raises=AttributeError, reason="None is not accepted as value for data")((test_opt_bool_as_array(data=None), lambda: RosOptAsList(RosBool()), list, bool, list)),
    (test_opt_bool_as_array(), lambda: RosOptAsList(RosBool()), list, bool, list),
])
def test_fromrosopt(msg, schema_field_type, in_rosfield_pytype, dictfield_pytype, out_rosfield_pytype):
    """
    Checking deserialization/serialization from a rosmsg
    :param msg: the message
    :param schema_field_type: the field type for that message 'data' field
    :param rosmsg_type: the actual rosmsg type
    :param rosmsgfield_pytype:
    :param dictfield_pytype:
    :param rosfield_pytype:
    :return:
    """

    # Schemas' Field constructor
    field = schema_field_type()

    assert hasattr(msg, 'data')
    # Making sure the data msg field is of the intended pytype
    # in case ROS messages do - or dont do - some conversions
    assert isinstance(msg.data, in_rosfield_pytype)
    deserialized = field.deserialize(msg.data)

    # check the deserialized version is the type we expect (or a missing optional field)
    assert isinstance(deserialized, dictfield_pytype) or deserialized == marshmallow.utils.missing
    if deserialized != marshmallow.utils.missing:
        # check the deserialized value is the same as the value of that field in the original message
        # We need the type conversion to deal with deserialized object in different format than ros data (like string)
        # we also need to deal with slots in case we have complex objects (only one level supported)
        if dictfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type]:
            if in_rosfield_pytype == genpy.rostime.Time or in_rosfield_pytype == genpy.rostime.Duration:  # non verbatim basic fields
                assert deserialized == dictfield_pytype(msg.data.to_sec())
            elif dictfield_pytype == list:  # TODO : improve this check
                # TODO : find a way to get rid of this special case...
                for idx, elem in enumerate(msg.data):
                    assert deserialized[idx] == elem
            else:
                assert deserialized == dictfield_pytype(msg.data[0])
        else:  # not a basic type for python (slots should be there though...)
            assert deserialized == dictfield_pytype([(s, getattr(msg.data, s)) for s in msg.data.__slots__])

        serialized = field.serialize(0, [deserialized])

        # Check the field value we obtain is the expected ros type and same value.
        assert isinstance(serialized, out_rosfield_pytype)
        assert serialized == msg.data


@pytest.mark.parametrize("pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype", [
    # Bool
    # basic exmple (True, RosBool, std_msgs.msg.Bool, bool, bool),
    (True, lambda: RosOptAsList(RosBool()), list, bool, list),
    (False, lambda: RosOptAsList(RosBool()), list, bool, list),
    # also test [], None and default value
    ([], lambda: RosOptAsList(RosBool()), list, bool, list),
    (None, lambda: RosOptAsList(RosBool()), list, bool, list),
    # careful : bool() defaults to False
    (bool(), lambda: RosOptAsList(RosBool()), list, bool, list),
])
def test_frompy(pyfield, schema_field_type, rosmsg_type, rosfield_pytype, pyfield_pytype):

    # Schemas' Field constructor
    field = schema_field_type()

    serialized = field.serialize(0, [pyfield])

    # Check the serialized field is the type we expect.
    assert isinstance(serialized[0], rosfield_pytype)
    # check the serialized value is the same as the value of that field in the original message
    # We need the type conversion to deal with serialized object in different format than ros data (like string)
    # we also need to deal with slots in case we have complex objects (only one level supported)
    if rosfield_pytype in [bool, int, six_long, float, six.binary_type, six.text_type]:
        assert serialized == pyfield
    else:  # not a basic type for python
        if pyfield_pytype in [int, six_long, float]:  # non verbatim basic fields
            assert serialized == rosfield_pytype(secs=int(pyfield), nsecs=int(pyfield * 1e9 - int(pyfield) * 1e9))
        elif pyfield_pytype == list:
            for idx, elem in enumerate(pyfield):
                assert serialized[idx] == elem
        else:  # dict format can be used though...
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

# TODO : all of the field types...

# TODO : these are property tests : use hypothesis for that...

# # Since the rospy message type member field is already a python int,
# # we do not need anything special here, we rely on marshmallow python type validation.
# # Yet we are specifying each on in case we want to extend it later...
#


# Just in case we run this directly
if __name__ == '__main__':
    pytest.main([
        'test_optional_fields.py::test_fromrosopt',
        'test_optional_fields.py::test_frompy',
    ])
