# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function

import importlib

import six

from .exceptions import PyrosSchemasException, PyrosSchemasValidationError
from .fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString, RosTextString,
)
from . import RosSchema, pre_load, post_load, pre_dump, post_dump

import roslib


ros_msgtype_mapping = {
    'bool': RosBool,
    'int8': RosInt8, 'int16': RosInt16, 'int32': RosInt32, 'int64': RosInt64,
    'uint8': RosUInt8, 'uint16': RosUInt16, 'uint32': RosUInt32, 'uint64': RosUInt64,
    'float32': RosFloat32, 'float64': RosFloat64,
    # Note: both of these work for service response (check with ip callback)
    'string': RosString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
    #'string': RosTextString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
    # Time ???
}

__slots__ = ['headers']
_slot_types = ['pyros_rosclient/HttpRequestHeaders']


class InvalidTypeStringException(PyrosSchemasException):
    def __init__(self, typestring):
        Exception.__init__(self, "{0!s} is not a valid type string".format(typestring))



class InvalidPackageException(PyrosSchemasException):
    def __init__(self, package, original_exception):
        Exception.__init__(self,
           "Unable to load the manifest for package {0!s}. Caused by: {1!s}".format(package, original_exception.message)
       )


class InvalidModuleException(PyrosSchemasException):
    def __init__(self, modname, subname, original_exception):
        Exception.__init__(self,
           "Unable to import {0!s}.{1!s} from package {2!s}. Caused by: {3!s}".format(modname, subname, modname, str(original_exception))
        )


class InvalidClassException(PyrosSchemasException):
    def __init__(self, modname, subname, classname, original_exception):
        Exception.__init__(self,
           "Unable to import {0!s} class {1!s} from package {2!s}. Caused by {3!s}".format(subname, classname, modname, str(original_exception))
        )

def _splittype(typestring):
    """ Split the string the / delimiter and strip out empty strings
    Performs similar logic to roslib.names.package_resource_name but is a bit
    more forgiving about excess slashes
    """
    splits = [x for x in typestring.split("/") if x]
    if len(splits) == 2:
        return splits
    raise InvalidTypeStringException(typestring)


def _get_msg_class(typestring):
    """ If not loaded, loads the specified msg class then returns an instance
    of it
    Throws various exceptions if loading the msg class fails
    """
    return _get_class(typestring, "msg")


def _get_srv_class(typestring):
    """ If not loaded, loads the specified srv class then returns an instance
    of it

    Throws various exceptions if loading the srv class fails """
    global _loaded_srvs, _srvs_lock
    return _get_class(typestring, "srv")


def _get_class(typestring, subname):
    """ If not loaded, loads the specified class then returns an instance
    of it.
    Loaded classes are cached in the provided cache dict
    Throws various exceptions if loading the msg class fails
    """
    # normalise the typestring
    modname, classname = _splittype(typestring)
    norm_typestring = modname + "/" + classname

    # Load the class
    cls = importlib.import_module('{0!s}.{1!s}'.format(modname, subname))
    #cls = _load_class(modname, subname, classname)

    return cls


def create(ros_msg_class,
                pre_load_fun=None,
                post_load_fun=None,
                pre_dump_fun=None,
                post_dump_fun=None,
                **kwargs):
    """
    Factory method that creates a Schema class for this ROS message type by introspecting the ros_msg_class, and then instanciate it.
    :param ros_msg_class: the message class for which we need serialization
    :param pre_load_fun: a callable that will be run before load(). It should be of the form : schema, data -> data
    :param post_load_fun: a callable that will be run after load(). It should be of the form : schema, data -> data
    Note that type validation is already implemented internally. check with_explicitly_matched_type decorator for more details
    :param pre_dump_fun: a callable that will be run before dump(). It should be of the form : schema, data -> data
    Note that type validation is already implemented internally. check with_explicitly_matched_type decorator for more details
    :param post_dump_fun: a callable that will be run after dump(). It should be of the form : schema, data -> data
    :param kwargs: any keyword argument will be added to the schema class.
    :return: A Schema that handles all (dict --load()--> ros_msg_class --dump()--> dict) serialization
    """

    if isinstance(ros_msg_class, six.string_types):  # if we get a string it s a ros descritpion, not the class itself
        # ENDING RECURSION with well known type
        if ros_msg_class in ros_msgtype_mapping:
            return ros_msgtype_mapping[ros_msg_class]()
        else:
            # getting the actual class to be able to introspect it
            ros_msg_class = _get_msg_class(ros_msg_class)
            # and keep going

    # RECURSE to get all nested message classes
    members = dict(zip(ros_msg_class.__slots__, [create(t) for t in ros_msg_class._slot_types]))

    # Adding methods for type validation
    def _verify_ros_type(schema_instance, data):
        # introspect data
        if not isinstance(data, ros_msg_class):
            raise PyrosSchemasValidationError('data type should be {0}'.format(ros_msg_class))

    def _key_mod(schema_instance, data):
        # careful : headers key name are slightly different
        return {k.replace('-', '_'): v for k, v in data.items()}

    def _make_ros_type(schema_instance, data):
        data = ros_msg_class(**data)
        return data

    members['_verify_ros_type'] = pre_dump(_verify_ros_type)
    members['_key_mod'] = pre_load(_key_mod)
    members['_make_ros_type'] = post_load(_make_ros_type)

    # supporting extra customization of the serialization
    if pre_load_fun:
        members['_helper_pre_load'] = pre_load(pre_load_fun)
    if post_load_fun:
        members['_helper_post_load'] = post_load(post_load_fun)
    if pre_dump_fun:
        members['_helper_pre_dump'] = pre_dump(pre_dump_fun)
    if post_dump_fun:
        members['_helper_post_dump'] = post_dump(post_dump_fun)

    # adding extra members if needed
    for k, v in kwargs:
        members[k] = v

    MsgSchema = type('MsgSchema', (RosSchema,), members)

    # Generating the schema instance
    schema_instance = MsgSchema(strict=True)

    return schema_instance

