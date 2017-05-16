from __future__ import absolute_import
from __future__ import print_function

"""
Defining marshmallow schemas for ROS message fields

Ref : http://wiki.ros.org/msg

These Fields and Schema are meant to be used together with ROS message type serialization :
ROSTCP --deserialize in rospy--> std_msgs.msg.* --deserialize (load) in pyros_schemas--> dict
And reversely :
dict --serialize (dump) in pyros_schemas--> std_msgs.msg.* --serialize in rospy--> ROSTCP

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
        __name__ = 'optional_fields'

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
import inspect
import functools
import marshmallow.utils

from .basic_fields import RosNested, RosList


class RosOptAsList(RosList):
    """Any ros field, optional in deserialized (python dict) form. In Ros is it represented by a list of that field type, and can be empty.

    :param kwargs: The same keyword arguments that :class:`List` receives. required is set to True by default.
    """
    def __init__(self, cls_or_instance, **kwargs):
        super(RosOptAsList, self).__init__(cls_or_instance, **kwargs)

    def _serialize(self, value, attr, obj):
        if value is None:  # None here means we actually dont have this optional field
            value = []
        dumped = super(RosOptAsList, self)._serialize(value, attr, obj)
        return dumped  #  we always want a list for serialized (ROS format) field

    def _deserialize(self, value, attr, data):
        # value should always be a list here (since serialized format has to have a field)
        # It seems there is no need to modify data here...
        loaded = super(RosOptAsList, self)._deserialize(value, attr, data)
        if len(loaded) == 0:
            return marshmallow.utils.missing  # if we have no element in value list this field is missing.
        else:
            return loaded[0]


# TODO : fix this as possible way to implement optional field in ROS ( clearer for rOS dev than array )
class RosOptAsNested(RosNested):
    """Any ros field, optional in deserialized (python dict) form. In Ros is it represented by a nested message that adds a boolean.

    :param kwargs: The same keyword arguments that :class:`List` receives. required is set to True by default.
    """
    def __init__(self, cls_or_instance, **kwargs):
        super(RosOptAsNested, self).__init__(cls_or_instance, **kwargs)

    # TODO : Inverse Serialize / DEserialize
    def _serialize(self, value, attr, obj):
        new_value = super(RosOptAsNested, self)._serialize(value, attr, obj)
        if len(new_value) == 0:
            return marshmallow.utils.missing  # if we have no element in value list this field is missing.
        else:
            return new_value[0]  # we only return the first element, this represent an optional serialized field.

    def _deserialize(self, value, attr, data):
        # value should not be a list : serialized for has one field (optional)
        # It seems there is no need to modify data here...
        return super(RosOptAsNested, self)._deserialize([value], attr, data)


# default implementation for now
RosOpt = RosOptAsList