from __future__ import absolute_import
from __future__ import print_function

import six

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

# Keeping field declaration separate in case we want to extend it later.
# ROS doesnt allow missing field -> everything is required.

# defining subclasses, to be able to extend them if needed


class RosBool(marshmallow.fields.Boolean):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosBool, self).__init__(*args, **kwargs)

    # def _serialize(self, value, attr, obj):
    #     if value is None:
    #         return None
    #     return ensure_binary_type(value)
    #
    # def _deserialize(self, value, attr, data):
    #     if not isinstance(value, marshmallow.compat.basestring):
    #         self.fail('invalid')
    #     return ensure_binary_type(value)



# Since the rospy message type member field is already a python int,
# we do not need anything special here, we rely on marshmallow python type validation.
# Yet we are specifying each on in case we want to extend it later...


class RosInt8(marshmallow.fields.Integer):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosInt8, self).__init__(*args, **kwargs)


class RosUInt8(marshmallow.fields.Integer):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosUInt8, self).__init__(*args, **kwargs)


class RosInt16(marshmallow.fields.Integer):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosInt16, self).__init__(*args, **kwargs)


class RosUInt16(marshmallow.fields.Integer):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosUInt16, self).__init__(*args, **kwargs)


class RosInt32(marshmallow.fields.Integer):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosInt32, self).__init__(*args, **kwargs)


class RosUInt32(marshmallow.fields.Integer):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosUInt32, self).__init__(*args, **kwargs)


# We need to introduce some python 2 / 3 compatibiilty for long
six_long = six.integer_types[-1]


class RosInt64(marshmallow.fields.Number):
    # Inspired from Marshmallow Integer field implementation
    num_type = six_long
    default_error_messages = {
        'invalid': 'Not a valid long integer.'
    }

    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosInt64, self).__init__(*args, **kwargs)


class RosUInt64(marshmallow.fields.Integer):
    # Inspired from Marshmallow Integer field implementation
    num_type = six_long
    default_error_messages = {
        'invalid': 'Not a valid long integer.'
    }

    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)  # setting required to true by default
        super(RosUInt64, self).__init__(*args, **kwargs)


class RosFloat32(marshmallow.fields.Float):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosFloat32, self).__init__(*args, **kwargs)


class RosFloat64(marshmallow.fields.Float):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        super(RosFloat64, self).__init__(*args, **kwargs)

# We need to be strict with strings, since ROS should have only str,
# and from python we should pass only unicode (like marshmallow string field, planning forward compat here)
# Anyway Careful with python 3 here:

def ensure_binary_type(val):
    if isinstance(val, marshmallow.compat.text_type):
        val = val.encode('utf-8')
    return marshmallow.compat.binary_type(val)


class RosString(marshmallow.fields.Field):
    """A ros string, deserializing as ros python field type :
    Python <3.0 implies str, python >3.0 implies bytes

    >>> print('testdoc')
    testdoc

    If you need unicode deserialization, have a look at RosTextString.

    No marshmallow field class for this, so we're declaring it here.
    """
    default_error_messages = {
        'invalid': 'Not a valid binary string.'
    }

    def __init__(self, **kwargs):
        """
        :param kwargs: The same keyword arguments that :class:`Field` receives. required is set to True by default.
        """
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
    """A ros string, deserializing into unicode.
     Python <3.0 means unicode --serialize--> str, python >3.0 means str --serialize--> bytes

    For planning for python3, we deserialize this as unicode string
    For using with ROS we serialize as str / bytes

    No marshmallow field class for this, so we're declaring it here.
    """
    default_error_messages = {
        'invalid': 'Not a valid text string.'
    }

    # Note we still serialize to ros format 'str'

    # we got this from marshmallow.fields.String
    def _deserialize(self, value, attr, data):
        if value is None:
            return None
        return marshmallow.utils.ensure_text_type(value)


class RosTime(marshmallow.fields.Integer):
    """
    A ros time, serialized into a long int.
    We avoid float precision issues (since we want exact equality between serialized and deserialized values).
    Since 4294967295 < 9223372036854775807 / 1e9, our representation of the time with a long nsec value is valid
    """

    default_error_messages = {
        'invalid': 'Not a valid time.'
    }

    def _serialize(self, value, attr, obj):
        # CAREFUL : genpy version has slots, rospy version doesnt...
        # since genpy has crappy algo :
        s = value // 1000000000
        ns = value - s * 1000000000
        return genpy.Time(secs=s, nsecs=ns)  # this will be "canonized" by genpy to fit message type

    def _deserialize(self, value, attr, obj):
        # we need to be careful here, we cannot use the fields directly,
        # and we need to use the proper method to have the desired meaningful data
        return six_long(value.to_nsec())


class RosDuration(marshmallow.fields.Integer):
    """
    A ros duration, serialized into a long int.
    We avoid float precision issues (since we want exact equality between serialized and deserialized values).
    Since 4294967295 < 9223372036854775807 / 1e9, our representation of the time with a long nsec value is valid
    """
    default_error_messages = {
        'invalid': 'Not a valid duration.'
    }

    def _serialize(self, value, attr, obj):
        # CAREFUL : genpy version has slots, rospy version doesnt...
        # since genpy has crappy algo :
        s = value // 1000000000
        ns = value - s * 1000000000
        return genpy.Duration(secs=s, nsecs=ns)  # this will be "canonized" by genpy to fit message type

    def _deserialize(self, value, attr, obj):
        # we need to be careful here, we cannot use the fields directly,
        # and we need to use the proper method to have the desired meaningful data
        return six_long(value.to_nsec())


# CAREFUL with RosList : Ros works differently with lists...
class RosList(marshmallow.fields.List):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)  # setting required to true by default
        super(RosList, self).__init__(*args, **kwargs)


# CAREFUL with RosNested : Ros works differently with nesting...
# Check RosTime and Rosduration for example of how to use it
class RosNested(marshmallow.fields.Nested):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)   # setting required to true by default
        # TODO : set the nested schema
        # kwargs['nested'] = _RosTimeVerbatim  # forcing nested to be our schema
        super(RosNested, self).__init__(*args, **kwargs)

    def _serialize(self, value, attr, obj):
        # tv_dict = super(RosTimeVerbatim, self)._serialize(value, attr, obj)
        # # TODO : generic conversion from dict to ROS python type
        # t = rospy.Time(**tv_dict)  # we let this explicitely except if some value is wrong here...
        # return t
        return super(RosNested, self)._serialize(value, attr, obj)

    def _deserialize(self, value, attr, obj):
        # # TODO : generic conversion from ROS python type to dict
        # value_dict = {'secs': value.secs, 'nsecs': value.nsecs}
        # v = super(RosTimeVerbatim, self)._deserialize(value_dict, attr, obj)
        # return v

        # MEMO : value : HttpRequestHeaders (ROS style)
        # MEMO : attr : 'headers' (string)
        # MEMO : obj : {'headers': <value>}

        return super(RosNested, self)._deserialize(value, attr, obj)

