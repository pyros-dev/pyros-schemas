from __future__ import absolute_import
from __future__ import print_function


"""
Defining Schema for basic ros types

Detailed Ref : http://wiki.ros.org/std_msgs

These Fields and Schema are meant to be used together with ROS message type serialization :
ROSTCP --deserialize in rospy--> std_msgs.msg.Time --serialize (dump) in pyros_schemas--> dict
And reversely :
dict --deserialize (load) in pyros_schemas--> std_msgs.msg.Time --serialize in rospy--> ROSTCP

This helps pyros deal with data only as dicts without worrying about the underlying ROS implementation.
Also some serialization behavior adjustments have been done :

- optional fields

"""


import marshmallow
try:
    import std_msgs.msg as std_msgs
    import genpy
    import rospy
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs.msg as std_msgs
    import genpy
    import rospy


# To be able to run doctest directly we avoid relative import
from .decorators import with_explicitly_matched_type

from .std_Int import RosFieldUInt32
from .std_String import RosFieldString
from .std_Time import RosFieldTime

# Both pyros and rospy serialization could eventually be combined, to serialize only once and get a dict.
# TODO : investigate
# KISS as much as possible for now

#
# Schemas declaration
# Since we want to provide seamless but safe rospy message type <-> pyros dict conversion
# We need to validate on serialization (dump to dict)
# and create a rospy message type on deserialization (load from dict)
#


@with_explicitly_matched_type(std_msgs.Header)
class RosMsgHeader(marshmallow.Schema):
    """
    RosMsgTime handles serialization from std_msgs.Time to python dict
    and deserialization from python dict to std_msgs.Time

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgHeader(strict=True)

    >>> rosmsgFourtwo = std_msgs.Header(seq = 42, stamp= rospy.Time(secs=53, nsecs=123456), frame_id='fortytwo')
    >>> marshalledFourtwo, errors = schema.dump(rosmsgFourtwo)
    >>> marshmallow.pprint(marshalledFourtwo) if not errors else print("ERRORS {0}".format(errors))
    {u'frame_id': u'fortytwo',
     u'seq': 42,
     u'stamp': {u'nsecs': 123456, u'secs': 53}}
    >>> value, errors = schema.load(marshalledFourtwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Header.Header'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))  # doctest: +NORMALIZE_WHITESPACE
    seq: 42
    stamp:
      secs: 53
      nsecs:    123456
    frame_id: fortytwo

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosHeader = std_msgs.Header(seq = random.choice([42, 53]), stamp= rospy.Time(secs=random.choice([4, 2, 1]), nsecs=random.choice([123, 456, 789])), frame_id='random')
    >>> schema.load(schema.dump(randomRosHeader).data).data == randomRosHeader
    True
    """
    seq = RosFieldUInt32()
    stamp = RosFieldTime()
    frame_id = RosFieldString()
