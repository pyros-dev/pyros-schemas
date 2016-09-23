from __future__ import absolute_import
from __future__ import print_function


"""
Defining marshmallow fields for ROS message fields

Ref : http://wiki.ros.org/msg

These Fields and Schema are meant to be used together with ROS message type serialization :
ROSTCP --deserialize in rospy--> std_msgs.msg.* --serialize (dump) in pyros_schemas--> dict
And reversely :
dict --deserialize (load) in pyros_schemas--> std_msgs.msg.* --serialize in rospy--> ROSTCP

This helps pyros deal with data only as dicts without worrying about the underlying ROS implementation.

"""

# This is useful only if we need relative imports. Ref : http://stackoverflow.com/a/28154841/4006172
# declaring __package__ if needed (this module is run individually)
if __package__ is None and not __name__.startswith('pyros_schemas.ros.'):
    import os
    import sys
    from pathlib2 import Path
    top = Path(__file__).resolve().parents[2]
    # Or
    # from os.path import abspath, dirname
    #
    # top = abspath(__file__)
    # for _ in range(4):
    #     top = dirname(top)
    if sys.path[0] == os.path.dirname(__file__):
        sys.path[0] = str(top)  # we replace first path in list (current module dir path) by the path of the package.
        # this avoid unintentional relative import (even without point notation).
    else:  # not sure in which case this could happen, but just in case we don't want to break stuff
        sys.path.append(str(top))

    if __name__ == '__main__':
        __name__ = 'fields'

    __package__ = 'pyros_schemas.ros'
    import pyros_schemas.ros


try:
    # To be able to run doctest directly we avoid relative import
    import genpy
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import genpy



# From here we can pick this up from ROS if missing in python env.

import functools
import marshmallow

from .decorators import with_explicitly_matched_type

# Keeping field declaration separate in case we want to extend it later.
# ROS doesnt allow missing field -> everything is required.

RosBool = functools.partial(marshmallow.fields.Boolean, required=True)


# Since the rospy message type member field is already a python int,
# we do not need anything special here, we rely on marshmallow python type validation.
# Yet we are specifying each on in case we want to extend it later...

RosInt8 = functools.partial(marshmallow.fields.Integer, required=True)
RosUInt8 = functools.partial(marshmallow.fields.Integer, required=True)
RosInt16 = functools.partial(marshmallow.fields.Integer, required=True)
RosUInt16 = functools.partial(marshmallow.fields.Integer, required=True)
RosInt32 = functools.partial(marshmallow.fields.Integer, required=True)
RosUInt32 = functools.partial(marshmallow.fields.Integer, required=True)
RosInt64 = functools.partial(marshmallow.fields.Integer, required=True)
RosUInt64 = functools.partial(marshmallow.fields.Integer, required=True)

RosFloat32 = functools.partial(marshmallow.fields.Float, required=True)
RosFloat64 = functools.partial(marshmallow.fields.Float, required=True)


# We need to be strict with strings, since ROS should have only str,
# and from python we should pass only unicode (like marshmallow string field, planning forward compat here)
# Anyway Careful with python 3 here:

def ensure_binary_type(val):
    if isinstance(val, marshmallow.compat.text_type):
        val = val.encode('utf-8')
    return marshmallow.compat.binary_type(val)


class RosString(marshmallow.fields.Field):
    """A ros string, serializing as ros python field type :
    Python <3.0 implies str, python >3.0 implies bytes

    >>> print('testdoc')
    'testdoc'

    If you need unicode serialization, have a look at RosTextString.

    No marshmallow field class for this, so we're declaring it here.

    :param kwargs: The same keyword arguments that :class:`Field` receives. required is set to True by default.
    """
    default_error_messages = {
        'invalid': 'Not a valid binary string.'
    }

    def __init__(self, **kwargs):
        super(RosString, self).__init__(required=True, **kwargs)

    def _serialize(self, value, attr, obj):
        if value is None:
            return None
        return ensure_binary_type(value)

    def _deserialize(self, value, attr, data):
        if not isinstance(value, marshmallow.compat.basestring):
            self.fail('invalid')
        return ensure_binary_type(value)


# We need to allow unicode string to force conversion to str for use in a ros message.
# ros messages currently dont do any check and assign unicode string in str and then things break.
# Anyway Careful with python 3 here...
class RosTextString(RosString):
    """A ros string, serializing into unicode.
     Python <3.0 means unicode --deserialize--> str, python >3.0 means str --deserialize--> bytes

    For planning for python3, we serialize this as unicode string
    For using with ROS we deserialize as str / bytes

    No marshmallow field class for this, so we're declaring it here.

    :param kwargs: The same keyword arguments that :class:`Field` receives. required is set to True by default.
    """
    default_error_messages = {
        'invalid': 'Not a valid text string.'
    }

    # we got this from marshmallow.fields.String
    def _serialize(self, value, attr, obj):
        if value is None:
            return None
        return marshmallow.utils.ensure_text_type(value)


# CAREFUL with RosNested : Ros works differently with nesting...
RosNested = functools.partial(marshmallow.fields.Nested, required=True)


def gen_ros_time_schema():
    # this can be a field for Ros definition, but is actually a schema that need to match rostime
    @with_explicitly_matched_type(genpy.rostime.Time)
    class _RosFieldTime(marshmallow.Schema):
        secs = RosInt32()
        nsecs = RosInt32()
    return _RosFieldTime

RosTime = functools.partial(RosNested, nested=gen_ros_time_schema())


def gen_ros_duration_schema():
    # this can be a field for Ros definition, but is actually a schema that need to match rosduration
    @with_explicitly_matched_type(genpy.rostime.Duration)
    class _RosFieldDuration(marshmallow.Schema):
        secs = RosInt32()
        nsecs = RosInt32()
    return _RosFieldDuration


RosDuration = functools.partial(RosNested, nested=gen_ros_duration_schema())



class RosOpt(marshmallow.fields.List):
    """Any ros field, optional in serialized form. In Ros is it represented by a list of that field type, and can be empty.

    :param kwargs: The same keyword arguments that :class:`List` receives. required is set to True by default.
    """
    def __init__(self, cls_or_instance, **kwargs):
        super(RosOpt, self).__init__(cls_or_instance, **kwargs)

    def _serialize(self, value, attr, obj):
        new_value = super(RosOpt, self)._serialize(value, attr, obj)
        if len(new_value) == 0:
            return marshmallow.missing  # if we have no element in value list this field is missing.
        else:
            return new_value[0]  # we only return the first element, this represent an optional serialized field.

    def _deserialize(self, value, attr, data):
        # value should not be a list : serialized for has one field (optional)
        # It seems there is no need to modify data here...
        return super(RosOpt, self)._deserialize([value], attr, data)
