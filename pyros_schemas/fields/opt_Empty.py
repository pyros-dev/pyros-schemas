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


# To be able to run doctest directly we avoid relative import
from .decorators import with_explicitly_matched_type

# From here we can pick this up from ROS if missing in python env.
import marshmallow

# Keeping field declaration separate in case we want to extend it later
RosFieldBool = marshmallow.fields.Boolean


@with_explicitly_matched_type(pyros_msgs.opt_empty)
class PyrosMsgOptEmpty(marshmallow.Schema):
    """
    PyrosMsgOptBool Schema handles serialization from pyros_msgs.opt.Bool to python dict
    and deserialization from python dict to pyros_msgs.opt.Bool

    >>> schema = PyrosMsgOptBool()

    >>> rosmsgTrue = pyros_msgs.opt_empty()
    >>> marshalledTrue, errors = schema.dump(rosmsgTrue)
    >>> marshmallow.pprint(marshalledTrue) if not errors else print("ERRORS {0}".format(errors))
    {}
    >>> value, errors = schema.load(marshalledTrue)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_bool.opt_bool'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True

    Careful : passing 'initialized_' to the constructor will except.
    It is only an "internal" field and is not meant to be manipulated
    >>> rosmsgforcedInit = pyros_msgs.opt_bool(initialized_=True)
    Traceback (most recent call last):
     ...
    AttributeError: initialized_ is not an attribute of pyros_msgs.opt_bool

    Load is the inverse of dump (if we ignore possible errors):
    >>> import random
    >>> randomRosEmpty = pyros_msgs.opt_empty()
    >>> schema.load(schema.dump(randomRosEmpty).data).data == randomRosEmpty
    True
    """
    # 'initialized_' is required for the schema, but the user doesnt have to set it explicitly.
    initialized_ = RosFieldBool(required=True, dump_only=True)

    @marshmallow.post_dump
    def unset_data(self, data):
        data.pop('initialized_')




