# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function

import importlib
import inspect

import six
import sys

import marshmallow
from marshmallow.schema import BaseSchema as marshmallow_BaseSchema
from marshmallow.schema import SchemaMeta as marshmallow_SchemaMeta
from .exceptions import PyrosSchemasException, PyrosSchemasValidationError
from .basic_fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString, RosTextString,
    RosNested,
    RosList,
)
from .optional_fields import (
    RosOpt,
)
from .decorators import pre_load, post_load, pre_dump, post_dump

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


# TODO  : check genpy.Message for get_message_class and get_service_class
def _get_msg_class(typestring):
    """ If not loaded, loads the specified msg class then returns an instance
    of it
    Throws various exceptions if loading the msg class fails
    """
    return _get_class(typestring, "msg")
    # return genpy.get_message_class(typestring, reload_on_error=True)


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
    cls = _load_class(modname, subname, classname)

    return cls


def _load_class(modname, subname, classname):
    """ Loads the manifest and imports the module that contains the specified
    type.

    Logic is similar to that of roslib.message.get_message_class, but we want
    more expressive exceptions.

    Returns the loaded module, or None on failure """
    global loaded_modules

    try:
        # roslib maintains a cache of loaded manifests, so no need to duplicate
        roslib.launcher.load_manifest(modname)
    except Exception as exc:
        raise InvalidPackageException(modname, exc)

    try:
        pymod = importlib.import_module('{0!s}.{1!s}'.format(modname, subname))
    except Exception as exc:
        raise InvalidModuleException(modname, subname, exc)

    try:
        return getattr(pymod, classname)
    except Exception as exc:
        raise InvalidClassException(modname, subname, classname, exc)




def _get_rosmsg_members_as_dict(inst):
    if hasattr(inst, '__slots__'):  # ROS style
        slots = []
        ancestors = inspect.getmro(type(inst))
        for a in ancestors:
            slots += set(a.__slots__) if hasattr(a, '__slots__') else set()
        # Remove special ROS slots
        if '_connection_header' in slots:
            slots.remove('_connection_header')
        data_dict = {
            slot: getattr(inst, slot)
            for slot in slots
        }
    elif hasattr(inst, '__dict__'):  # generic python object (for tests)
        data_dict = vars(inst)
    else:  # this is a basic python type (including dict)
        data_dict = inst
    return data_dict


def _get_rosmsg_fields_as_dict(cls):
    if hasattr(cls, '__slots__'):  # ROS style
        slots = []
        slots_types = []
        ancestors = inspect.getmro(cls)
        for a in ancestors:
            slots += a.__slots__ if hasattr(a, '__slots__') else []
            slots_types += a._slot_types if hasattr(a, '_slot_types') else []
        # Remove special ROS slots
        if '_connection_header' in slots:
            slots.remove('_connection_header')
        data_dict = dict(zip(slots, slots_types))
    elif hasattr(cls, '__dict__'):  # generic python object (for tests)
        data_dict = {k: type(v) for k, v in vars(cls).items()}
    else:  # this is a basic python type (including dict)
        data_dict = cls
    return data_dict


class RosSchema(marshmallow.Schema):
    """Inheriting the Marshmallow schema to extend behavior introspecting into slots for ROS messages
    Not using pre_load, post_load, pre_dump or post_dump here, to simplify things for when we need to create schemas dynamically.
    pre_load, post_load, pre_dump, post_dump should still be used in derived Schemas, to customize the serialization
    This class only factor serialization behavior required by ROS generated message types.
    """

    _valid_ros_msgtype = None  # fill this in your Schema class for enforcing msgtype validation on load
    _generated_ros_msgtype = None  # fill this in your Schema class for automatically generating msgtype on dump

    def __init__(self, strict=True, **kwargs):  # default to strict behavior
        super(RosSchema, self).__init__(strict=strict, **kwargs)

    def load(self, data, many=None, partial=None):
        """Overloading load function to transform a ROS msg type into a dict for marshmallow"""
        # early type validation if required
        if self.strict and self._valid_ros_msgtype and not isinstance(data, self._valid_ros_msgtype):
            raise PyrosSchemasValidationError('data type should be {0}'.format(self._valid_ros_msgtype))
        data_dict = _get_rosmsg_members_as_dict(data)
        try:
            unmarshal_result = super(RosSchema, self).load(data_dict, many=many, partial=partial)
        except marshmallow.ValidationError as ve:
            raise PyrosSchemasValidationError('ERROR occurred during deserialization: {ve}'.format(**locals()))
        return unmarshal_result

    def dump(self, obj, many=None, update_fields=True, **kwargs):
        """Overloading dump function to transform a dict into a ROS msg from marshmallow"""
        try:
            obj_dict = _get_rosmsg_members_as_dict(obj)  # in case we get something that is not a dict...
            # because ROS field naming conventions are different than python dict key conventions
            obj_rosfixed_dict = {k.replace('-', '_'): v for k, v in obj_dict.items()}  # TODO : come up with a generic <ROS_field encode> function
            data_dict, errors = super(RosSchema, self).dump(obj_rosfixed_dict, many=many, update_fields=update_fields, **kwargs)
        except marshmallow.ValidationError as ve:
            raise PyrosSchemasValidationError('ERROR occurred during serialization: {ve}'.format(**locals()))
        if self._generated_ros_msgtype and not errors:
            obj = self._generated_ros_msgtype(**data_dict)
        else:
            obj = data_dict  # we return directly
        return marshmallow.MarshalResult(obj, errors)


# TODO : find cleaner way, maybe a RosMagikSchema class like thing...
def create(ros_msg_class,
                pre_load_fun=None,
                post_load_fun=None,
                pre_dump_fun=None,
                post_dump_fun=None,
                **kwargs):
    """
    Factory method that creates a Schema class for this ROS message type by introspecting the ros_msg_class, and then instanciate it.
    :param ros_msg_class: the message class for which we need serialization. It can be the string specifying the message type or the type itself
    :param pre_load_fun: a callable that will be run before load(). It should be of the form : schema, data -> data
    :param post_load_fun: a callable that will be run after load(). It should be of the form : schema, data -> data
    Note that type validation is already implemented internally. check with_explicitly_matched_type decorator for more details
    :param pre_dump_fun: a callable that will be run before dump(). It should be of the form : schema, data -> data
    Note that type validation is already implemented internally. check with_explicitly_matched_type decorator for more details
    :param post_dump_fun: a callable that will be run after dump(). It should be of the form : schema, data -> data
    :param kwargs: any keyword argument will be added to the schema class.
    :return: A Schema that handles all (dict --load()--> ros_msg_class --dump()--> dict) serialization
    """

    if isinstance(ros_msg_class, six.string_types):  # if we get a string it s a ros description, not the class itself
        ros_msg_class = _get_msg_class(ros_msg_class)
        # and keep going

    members_types = _get_rosmsg_fields_as_dict(ros_msg_class)
    members = {}
    for s, stype in members_types.iteritems():
        # Note here we rely entirely on _opt_slots from the class to be set properly
        # for both Nested or List representation of optional fields
        ros_schema_inst = None
        if stype.endswith("[]"):
            if stype[:-2] in ros_msgtype_mapping:
                # ENDING RECURSION with well known array type
                if hasattr(ros_msg_class, '_opt_slots') and s in ros_msg_class._opt_slots:
                    ros_schema_inst = RosOpt(ros_msgtype_mapping[stype[:-2]]())
                else:
                    ros_schema_inst = RosList(ros_msgtype_mapping[stype[:-2]]())
            else:
                # RECURSING in Nested fields
                if hasattr(ros_msg_class, '_opt_slots') and s in ros_msg_class._opt_slots:
                    ros_schema_inst = RosOpt(RosNested(create(stype[:-2])))  # we need to nest the next (Ros)Schema
                else:
                    ros_schema_inst = RosList(RosNested(create(stype[:-2])))  # we need to nest the next (Ros)Schema
        else:
            if stype in ros_msgtype_mapping:
                # ENDING RECURSION with well known basic type
                ros_schema_inst = ros_msgtype_mapping[stype]()  # TODO : shouldn't we check for opt slots here ?
            else:
                # RECURSING in Nested fields
                if hasattr(ros_msg_class, '_opt_slots') and s in ros_msg_class._opt_slots:
                    ros_schema_inst = RosOpt(create(stype))
                else:
                    ros_schema_inst = RosNested(create(stype))  # we need to nest the next (Ros)Schema

        members.setdefault(s, ros_schema_inst)

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

    members['_valid_ros_msgtype'] = ros_msg_class
    members['_generated_ros_msgtype'] = ros_msg_class

    MsgSchema = type(ros_msg_class.__name__ + 'Schema', (RosSchema,), members)

    # Generating the schema instance
    # TODO : nicer way ?
    schema_instance = MsgSchema()

    return schema_instance

