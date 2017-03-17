from __future__ import absolute_import
from __future__ import print_function

"""
Defining marshmallow schemas for ROS message fields

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
        __name__ = 'schema_fields'

    __package__ = 'pyros_schemas.ros'
    import pyros_schemas.ros


import six
six_long = six.integer_types[-1]

try:
    # To be able to run doctest directly we avoid relative import
    import genpy
    import rospy
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import genpy
    import rospy


from .basic_fields import RosUInt32, RosInt32, RosNested
from .schema import RosSchema


# class _RosTimeSchema(RosSchema):
#     """A ros time schema.
#     -  rospy.Time -> load() -> dict
#     -  dict -> dump() -> rospy.Time
#     """
#
#     _valid_ros_msgtype = genpy.Time
#     _generated_ros_msgtype = genpy.Time
#
#     secs = RosUInt32()
#     nsecs = RosUInt32()  # TODO : maybe we should have only the nsecs as UInt64 since that is the meaning we want here for serialization/deserialization...
#
#
# class RosTimeVerbatim(RosNested):
#     """A ros time, serialized into a genpy.Time()"""
#     def __init__(self, *args, **kwargs):
#         kwargs['nested'] = _RosTimeSchema  # forcing nested to be our schema
#         super(RosTimeVerbatim, self).__init__(*args, **kwargs)
#
#
# class RosTime(RosTimeVerbatim):
#     """
#     A ros time, serialized into a long int.
#     We avoid float precision issues (since we want exact equality between serialized and deserialized values).
#     Since 4294967295 < 9223372036854775807 / 1e9, our representation of the time with a long nsec value is valid
#     """
#
#     _generated_ros_msgtype = genpy.Time
#
#     default_error_messages = {
#         'invalid': 'Not a valid time.'
#     }
#
#     def _serialize(self, value, attr, obj):
#         # we need to be careful here, we cannot use the fields directly,
#         # and we need to use the proper method to have the desired meaningful data
#         v = super(RosTime, self)._serialize({'nsecs': value}, attr, obj)
#         return v
#
#     def _deserialize(self, value, attr, obj):
#         v = super(RosTime, self)._deserialize(value.to_nsec(), attr, obj)
#         # we need to be careful here, we cannot use the fields directly,
#         # and we need to use the proper method to have the desired meaningful data
#         return v


# class _RosDurationSchema(RosSchema):
#     """A ros duration schema.
#     -  rospy.Duration -> load() -> dict
#     -  dict -> dump() -> rospy.Duration
#     """
#
#     _valid_ros_msgtype = genpy.Duration
#     _generated_ros_msgtype = genpy.Duration  # we also generate a genpy since it is compatible with rospy, and has the correct list of slots
#
#     secs = RosInt32()
#     nsecs = RosInt32()
#
#
# class RosDurationVerbatim(RosNested):
#     """A ros time, serialized into a genpy.Duration()"""
#     def __init__(self, *args, **kwargs):
#         kwargs['nested'] = _RosDurationSchema  # forcing nested to be our schema
#         super(RosDurationVerbatim, self).__init__(*args, **kwargs)
#
#
# class RosDuration(RosDurationVerbatim):
#     """
#     A ros duration, serialized into a long int.
#     We avoid float precision issues (since we want exact equality between serialized and deserialized values).
#     Since 4294967295 < 9223372036854775807 / 1e9, our representation of the time with a long nsec value is valid
#     """
#     default_error_messages = {
#         'invalid': 'Not a valid duration.'
#     }
#
#     def _serialize(self, value, attr, obj):
#         # we need to be careful here, we cannot use the fields directly,
#         # and we need to use the proper method to have the desired meaningful data
#         v = super(RosDuration, self)._serialize({'secs': secs, 'nsecs': nsecs}, attr, obj)
#         return v
#
#     def _deserialize(self, value, attr, obj):
#         v = super(RosDuration, self)._deserialize(value, attr, obj)
#         # we need to be careful here, we cannot use the fields directly,
#         # and we need to use the proper method to have the desired meaningful data
#         return v.get('nsecs')
