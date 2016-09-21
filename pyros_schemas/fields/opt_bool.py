from __future__ import absolute_import
from __future__ import print_function

"""
Defining Schema for optional ros types

These add a boolean "initialized_" field to basic ros types, to allow for a field being there, or not.

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
if __package__ is None and not __name__.startswith('pyros_schemas.'):
    import sys
    from pathlib2 import Path
    top = Path(__file__).resolve().parents[1]
    sys.path.append(str(top))
    # Or
    # from os.path import abspath, dirname
    #
    # top = abspath(__file__)
    # for _ in range(4):
    #     top = dirname(top)
    # sys.path.append(top)

    import pyros_schemas
    __package__ = 'pyros_schemas'


from .decorators import with_explicitly_matched_optional_type
from pyros_msgs import opt_bool

# From here we can pick this up from ROS if missing in python env.
import marshmallow

# Keeping field declaration separate in case we want to extend it later
RosFieldBool = marshmallow.fields.Boolean


@with_explicitly_matched_optional_type(opt_bool)
class PyrosMsgOptBool(marshmallow.Schema):
    """
    PyrosMsgOptBool Schema handles serialization from pyros_msgs.opt.Bool to python dict
    and deserialization from python dict to pyros_msgs.opt.Bool

    >>> schema = PyrosMsgOptBool()

    >>> rosmsgTrue = pyros_msgs.opt_bool(data=True)
    >>> marshalledTrue, errors = schema.dump(rosmsgTrue)
    >>> marshmallow.pprint(marshalledTrue) if not errors else print("ERRORS {0}".format(errors))
    {u'data': True}
    >>> value, errors = schema.load(marshalledTrue)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_bool.opt_bool'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True
    data: True

    >>> rosmsgFalse = pyros_msgs.opt_bool(data=False)
    >>> marshalledFalse, errors = schema.dump(rosmsgFalse)
    >>> marshmallow.pprint(marshalledFalse) if not errors else print("ERRORS {0}".format(errors))
    {u'data': False}
    >>> value, errors = schema.load(marshalledFalse)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_bool.opt_bool'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True
    data: False

    data field is optional. If not passed in original message object, it wont be in serialized object

    >>> rosmsgUninit = pyros_msgs.opt_bool()
    >>> marshalledUninit, errors = schema.dump(rosmsgUninit)
    >>> marshmallow.pprint(marshalledUninit) if not errors else print("ERRORS {0}".format(errors))
    {}
    >>> value, errors = schema.load(marshalledUninit)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_bool.opt_bool'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: False
    data: False

    Careful : passing 'initialized_' to the constructor will except.
    It is only an "internal" field and is not meant to be manipulated

    >>> rosmsgforcedInit = pyros_msgs.opt_bool(initialized_=True)
    Traceback (most recent call last):
     ...
    AttributeError: The field 'initialized_' is an internal field of pyros_msgs/opt_bool and should not be set by the user.

    If you want to pass fields default value, it should be done explicitly, like so :

    >>> pyros_msgs.opt_bool(data=bool())
    initialized_: True
    data: False

    Load is the inverse of dump (if we ignore possible errors):

    >>> import random
    >>> randomRosBool = pyros_msgs.opt_bool(data=random.choice([True, False]))
    >>> schema.load(schema.dump(randomRosBool).data).data == randomRosBool
    True
    """
    data = RosFieldBool()







