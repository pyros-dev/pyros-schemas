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


# From here we can pick this up from ROS if missing in python env.
import inspect
import functools
import marshmallow

from .decorators import with_explicitly_matched_type
from .fields import RosUInt32, RosInt32, RosNested, RosSchema


#@with_explicitly_matched_type(genpy.Time)  # genpy.Time is the type of data field
class _RosTimeVerbatim(RosSchema):
    valid_ros_type = genpy.Time
    generated_ros_type = rospy.Time
    """A ros time, serializing verbatim"""
    secs = RosUInt32()
    nsecs = RosUInt32()

    @marshmallow.pre_dump
    def _verify_ros_type(self, data):
        # introspect data
        if not isinstance(data, self.valid_ros_type):
            raise marshmallow.ValidationError('data type should be {0}'.format(self.matched_ros_type))
        return data

    # For use in RosNested
    @marshmallow.post_dump
    def _postdumper(self, data):
        data = self.valid_ros_type(**data)
        return data

    # For use in RosNested
    @marshmallow.pre_load
    def _preloader(self, data):
        if hasattr(data, '__dict__'):
            return vars(data)
        elif hasattr(data, '__slots__'):
            slots = set()  # need unicity of slots
            ancestors = inspect.getmro(type(data))
            for a in ancestors:
                slots |= set(a.__slots__) if hasattr(a, '__slots__') else set()
            data_dict = {
                slot: getattr(data, slot)
                for slot in slots
            }
            return data_dict
        else:  # this is a basic python type (including dict)
            return data

    @marshmallow.post_load
    def _make_ros_type(self, data):
        data = self.generated_ros_type(**data)
        return data


class RosTimeVerbatim(RosNested):
    """A ros time, serialized into a rospy.Time()"""
    def __init__(self, *args, **kwargs):
        kwargs['nested'] = _RosTimeVerbatim  # forcing nested to be our schema
        super(RosTimeVerbatim, self).__init__(*args, **kwargs)

    def _serialize(self, value, attr, obj):
        tv_dict = super(RosTimeVerbatim, self)._serialize(value, attr, obj)
        # t = rospy.Time(**tv_dict)  # we let this explicitely except if some value is wrong here...
        return tv_dict

    def _deserialize(self, value, attr, obj):
        # value_dict = {'secs': value.secs, 'nsecs': value.nsecs}
        v = super(RosTimeVerbatim, self)._deserialize(value, attr, obj)
        return v


class RosTime(RosTimeVerbatim):
    """A ros time, serialized into a float, like time.time()."""
    default_error_messages = {
        'invalid': 'Not a valid time.'
    }

    def _serialize(self, value, attr, obj):
        v = super(RosTime, self)._serialize(value, attr, obj)
        return v

    def _deserialize(self, value, attr, obj):
        v = super(RosTime, self)._deserialize(value, attr, obj)
        return v


# @with_explicitly_matched_type(genpy.rostime.Duration)
class _RosDurationVerbatim(RosSchema):
    """A ros duration, serializing verbatim"""
    secs = RosInt32()
    nsecs = RosInt32()


class RosDurationVerbatim(RosNested):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault('required', True)  # setting required to true by default
        kwargs['nested'] = _RosDurationVerbatim  # forcing nested to be our schema
        super(RosDurationVerbatim, self).__init__(*args, **kwargs)


class RosDuration(RosDurationVerbatim):
    """A ros duration, serialized into a rospy.Duration()."""
    default_error_messages = {
        'invalid': 'Not a valid duration.'
    }

    def _serialize(self, value, attr, obj):
        v = super(RosDuration, self)._serialize(value, attr, obj)
        return v

    def _deserialize(self, value, attr, obj):
        v = super(RosDuration, self)._deserialize(value, attr, obj)
        return v


class RosOptAsList(marshmallow.fields.List):
    """Any ros field, optional in serialized form. In Ros is it represented by a list of that field type, and can be empty.

    :param kwargs: The same keyword arguments that :class:`List` receives. required is set to True by default.
    """
    def __init__(self, cls_or_instance, **kwargs):
        super(RosOptAsList, self).__init__(cls_or_instance, **kwargs)

    def _serialize(self, value, attr, obj):
        new_value = super(RosOptAsList, self)._serialize(value, attr, obj)
        if len(new_value) == 0:
            return marshmallow.missing  # if we have no element in value list this field is missing.
        else:
            return new_value[0]  # we only return the first element, this represent an optional serialized field.

    def _deserialize(self, value, attr, data):
        # value should not be a list : serialized for has one field (optional)
        # It seems there is no need to modify data here...
        return super(RosOptAsList, self)._deserialize([value], attr, data)


# TODO : fix this as possible way to implement optional field in ROS ( clearer for rOS dev than array )
class RosOptAsNested(marshmallow.fields.Nested):
    """Any ros field, optional in serialized form. In Ros is it represented by a nested message that adds a boolean.

    :param kwargs: The same keyword arguments that :class:`List` receives. required is set to True by default.
    """
    def __init__(self, cls_or_instance, **kwargs):
        super(RosOptAsNested, self).__init__(cls_or_instance, **kwargs)

    def _serialize(self, value, attr, obj):
        new_value = super(RosOptAsNested, self)._serialize(value, attr, obj)
        if len(new_value) == 0:
            return marshmallow.missing  # if we have no element in value list this field is missing.
        else:
            return new_value[0]  # we only return the first element, this represent an optional serialized field.

    def _deserialize(self, value, attr, data):
        # value should not be a list : serialized for has one field (optional)
        # It seems there is no need to modify data here...
        return super(RosOptAsNested, self)._deserialize([value], attr, data)


# default implementation for now
RosOpt = RosOptAsList