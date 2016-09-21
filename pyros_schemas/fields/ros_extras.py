from __future__ import absolute_import
from __future__ import print_function


"""
Defining marshmallow fields for extra ROS message fields

These are useful when we want to pass a different python type into the same ros field as another (python) type

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

import marshmallow

from .ros import RosString

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





