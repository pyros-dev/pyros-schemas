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
    # To be able to run doctest directly
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
from pyros_msgs import opt_string

# From here we can pick this up from ROS if missing in python env.
import marshmallow

# Keeping field declaration separate in case we want to extend it later
RosFieldString = marshmallow.fields.String


@with_explicitly_matched_optional_type(opt_string)
class PyrosMsgOptString(marshmallow.Schema):
    """
    PyrosMsgOptBool Schema handles serialization from pyros_msgs.opt.Bool to python dict
    and deserialization from python dict to pyros_msgs.opt.Bool

    >>> schema = PyrosMsgOptString(strict=True)

    >>> rosmsgTrue = pyros_msgs.opt_string(data='fortytwo')
    >>> marshalledAnswer, errors = schema.dump(rosmsgTrue)
    >>> marshmallow.pprint(marshalledAnswer) if not errors else print("ERRORS {0}".format(errors))
    {u'data': u'fortytwo'}
    >>> value, errors = schema.load(marshalledAnswer)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_string.opt_string'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True
    data: fortytwo

    data field is optional. If not passed in original message object, it will be set to a default value for ROS, but it will not be in serialized dict
    >>> rosmsgUninit = pyros_msgs.opt_string()
    >>> marshalledUninit, errors = schema.dump(rosmsgUninit)
    >>> marshmallow.pprint(marshalledUninit) if not errors else print("ERRORS {0}".format(errors))
    {}
    >>> value, errors = schema.load(marshalledUninit)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_string.opt_string'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: False
    data: ''

    Careful : passing 'initialized_' to the constructor will except.
    It is only an "internal" field and is not meant to be manipulated
    >>> rosmsgforcedInit = pyros_msgs.opt_bool(initialized_=True)
    Traceback (most recent call last):
     ...
    AttributeError: The field 'initialized_' is an internal field of pyros_msgs/opt_bool and should not be set by the user.

    Load is the inverse of dump (if we ignore possible errors):
    >>> import random
    >>> randomRosString = pyros_msgs.opt_string(data=random.choice(['fortytwo', 'twentyone']))
    >>> schema.load(schema.dump(randomRosString).data).data == randomRosString
    True


    Reversely if you start by loading from a python dict :
    >>> value, errors = schema.load({'data': 'fortytwo'})
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_string.opt_string'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True
    data: fortytwo

    >>> marshalledAnswer, errors = schema.dump(value)
    >>> marshmallow.pprint(marshalledAnswer) if not errors else print("ERRORS {0}".format(errors))
    {u'data': u'fortytwo'}


    Dump is the inverse of load (if we ignore possible errors):
    >>> import random
    >>> randomString = {'data' : random.choice(['fortytwo', 'twentyone'])}
    >>> w = schema.dump(schema.load(randomString).data).data
    >>> w == randomString
    True

    Note if you need to load from a python object, make use of the marshmallow pre_load decorator

    Note also that loading from a python string also works if type is compatible
    >>> value, errors = schema.load('fortytwo')
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_string.opt_string'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True
    data: fortytwo

    >>> marshalledAnswer, errors = schema.dump(value)
    >>> marshmallow.pprint(marshalledAnswer) if not errors else print("ERRORS {0}".format(errors))
    {u'data': u'fortytwo'}

    Or fail if not :
    >>> value, errors = schema.load(42)
    Traceback (most recent call last):
     ...
    ValidationError: {'data': [u'Not a valid string.']}
    """
    data = RosFieldString()







