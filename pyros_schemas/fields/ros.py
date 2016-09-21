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

try:
    # To be able to run doctest directly we avoid relative import
    import pyros_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import pyros_msgs

# This is useful only if we need relative imports. Ref : http://stackoverflow.com/a/28154841/4006172
# declaring __package__ if needed (this module is run individually)
if __package__ is None and not __name__.startswith('pyros_schemas.fields.ros'):
    import sys
    from pathlib2 import Path
    top = Path(__file__).resolve().parents[3]
    sys.path.append(str(top))
    # Or
    # from os.path import abspath, dirname
    #
    # top = abspath(__file__)
    # for _ in range(4):
    #     top = dirname(top)
    # sys.path.append(top)

    import pyros_schemas
    __package__ = 'pyros_schemas.fields.ros'



# From here we can pick this up from ROS if missing in python env.

import functools
import marshmallow

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



# CAREFUL with RosNested : Ros doesnt not allow
RosNested = functools.partial(marshmallow.fields.Nested, required=True)
