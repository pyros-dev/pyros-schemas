from __future__ import absolute_import
from __future__ import print_function

"""
Defining Schema for http request/response types

These Fields and Schema are meant to be used together with ROS message type serialization :
ROSTCP --deserialize in rospy--> std_msgs.msg.* --serialize (dump) in pyros_schemas--> dict
And reversely :
dict --deserialize (load) in pyros_schemas--> std_msgs.msg.* --serialize in rospy--> ROSTCP

This helps pyros deal with data only as dicts without worrying about the underlying ROS implementation.
"""

import marshmallow
try:
    import pyros_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import pyros_msgs


# To be able to run doctest directly we avoid relative import
from .decorators import with_explicitly_matched_type
from .std_Int import RosFieldUInt16


@with_explicitly_matched_type(pyros_msgs.HttpStatusCode)
class RosMsgHttpStatusCode(marshmallow.Schema):
    """
    RosMsgBool Schema handles serialization from std_msgs.msgs.Bool to python dict
    and deserialization from python dict to std_msgs.msgs.Bool

    >>> schema = RosMsgHttpStatusCode()

    >>> rosmsgTrue = pyros_msgs.HttpStatusCode(data=pyros_msgs.HttpStatusCode)
    >>> marshalledTrue, errors = schema.dump(rosmsgTrue)
    >>> marshmallow.pprint(marshalledTrue) if not errors else print("ERRORS {0}".format(errors))
    {u'data': True}
    >>> value, errors = schema.load(marshalledTrue)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Bool.Bool'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: True

    >>> rosmsgFalse = std_msgs.Bool(data=False)
    >>> marshalledFalse, errors = schema.dump(rosmsgFalse)
    >>> marshmallow.pprint(marshalledFalse) if not errors else print("ERRORS {0}".format(errors))
    {u'data': False}
    >>> value, errors = schema.load(marshalledFalse)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Bool.Bool'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: False

    Load is the inverse of dump (if we ignore possible errors):
    >>> import random
    >>> randomRosBool = std_msgs.Bool(data=random.choice([True, False]))
    >>> schema.load(schema.dump(randomRosBool).data).data == randomRosBool
    True
    """
    status_code = RosFieldUInt16()


