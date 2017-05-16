# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function

import inspect
import sys

import marshmallow
import functools

"""
Defining decorators to help with Schema generation for ROS message type <-> pyros dict conversion
This is useful for detailed definition of field serialization, like in schema_fields.py module
"""


# defining a decorate to wrap classes.
# We are doing this because we want to add methods to a class via decorators
def wraps_cls(original_cls):
    def wrapper(wrapper_cls):
        """
        Update wrapper_cls to look like original_cls.
        If this docstring ends up in your decorated class, you should define the __doc__ when declaring that class.
        Like:

        @wraps_cls(cls)
        class Wrapper(cls):
            __doc__ = cls.__doc__
            pass

        Ref : http://bugs.python.org/issue12773
        """
        for attr in functools.WRAPPER_ASSIGNMENTS:
            try:
                value = getattr(original_cls, attr)
            except AttributeError:
                pass
            else:
                try:  # this fails on __doc__ with python 2.7
                    setattr(wrapper_cls, attr, value)
                except AttributeError:
                    if sys.version_info < (3, 2) and attr == '__doc__':  # skipping if doc is not writeable.
                        pass
                    else:
                        raise
        return wrapper_cls
    return wrapper
#
# From: http://stackoverflow.com/questions/28622235/functools-wraps-equivalent-for-class-decorator
#
# Usage:
#
# def some_class_decorator(cls_to_decorate):
#     @wraps_cls(cls_to_decorate)
#     class Wrapper(cls_to_decorate):
#         """Some Wrapper not important doc."""
#         pass
#     return Wrapper
#
#
# @some_class_decorator
# class MainClass:
#     """MainClass important doc."""
#     pass
#
#
# help(MainClass)
#
#


# This is for explicit matching types.

# Ref : http://wiki.ros.org/msg
if sys.version_info >= (3, 0):
    ros_python_type_mapping = {
        'bool': bool,
        'int8': int, 'int16': int, 'int32': int, 'int64': int,
        'uint8': int, 'uint16': int, 'uint32': int, 'uint64': int,
        'float32': float, 'float64': float,
        'string': str,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
        #'string': RosTextString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
        # Time ???
    }
else:  # 2.7
    ros_python_type_mapping = {
        'bool': bool,
        'int8': int, 'int16': int, 'int32': int, 'int64': long,
        'uint8': int, 'uint16': int, 'uint32': int, 'uint64': long,
        'float32': float, 'float64': float,
        'string': str,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
        #'string': RosTextString,  # CAREFUL between ROS who wants byte string, and python3 where everything is unicode...
        # Time ???
    }


# TODO : get rid of this. It is a bit overkill (ros schema class does the job now...)
def with_explicitly_matched_type(valid_ros_type, generated_ros_type=None):
    """
    Decorator to add type check and type creation for a schema
    :param ros_type: the ros_type to check for and generate
    :return:

    TODO  : doctest
    """

    def schema_explicitly_matched_type_decorator(cls):
        assert isinstance(cls, marshmallow.schema.SchemaMeta)

        @wraps_cls(cls)
        class Wrapper(cls):
            # TODO : closure (IN PROGRESS check schema.py module)
            # TODO : proxy ?
            # This wrapper inherits. Maybe a proxy would be better ?
            # We cannot have a doc here, because it is not writeable in python 2.7
            # instead we reuse the one from the wrapped class
            __doc__ = cls.__doc__
            _valid_ros_type = valid_ros_type
            _generated_ros_type = generated_ros_type or valid_ros_type

            @marshmallow.validates_schema
            def _validate_ros_type(self, data):
                # extracting members from ROS type (we do not check internal type, we will just try conversion - python style)
                if hasattr(self._valid_ros_type, '_fields'):  # named tuples - CAREFUL they also have an empty __slots__
                    rtkeys = list(self._valid_ros_type._fields)
                elif hasattr(self._valid_ros_type, '__slots__'):  # ROS style
                    slots = []
                    ancestors = inspect.getmro(self._valid_ros_type)
                    for a in ancestors:
                        slots += a.__slots__ if hasattr(a, '__slots__') else []
                    # Remove special ROS slots
                    if '_connection_header' in slots:
                        slots.remove('_connection_header')
                    rtkeys = slots
                elif hasattr(self._valid_ros_type, '__dict__'):  # generic python object (for tests)
                    rtkeys = [k for k in vars(self._valid_ros_type).keys() if not k.startswith('_')]
                else:  # this is a basic python type (including dict)
                    rtkeys = ['data']  # this is intended to support decorating nested fields (????)
                    # OR except ???

                # here data is always a dict
                for dk, dv in data.iteritems():
                    if dk not in rtkeys:
                        raise marshmallow.ValidationError(
                            'loaded data has unexpected field {dk}'.format(**locals()))
                for k in rtkeys:
                    if k not in data:
                        raise marshmallow.ValidationError('loaded data missing field {k}'.format(**locals()))
                        # R should we let marshallow manage (the point of required parameter)

            @marshmallow.pre_load
            def _from_ros_type_to_dict(self, data):
                if hasattr(data, '_asdict'):  # named tuples (for tests) - Careful they also have an empty __slots__
                    return data._asdict()
                elif hasattr(data, '__slots__'):  # ROS style
                    slots = []
                    ancestors = inspect.getmro(type(data))
                    for a in ancestors:
                        slots += set(a.__slots__) if hasattr(a, '__slots__') else set()
                    # Remove special ROS slots
                    if '_connection_header' in slots:
                        slots.remove('_connection_header')
                    data_dict = {
                        slot: getattr(data, slot)
                        for slot in slots
                        }
                    return data_dict
                elif hasattr(data, '__dict__'):  # generic python object (for tests)
                    return vars(data)
                else:  # this is a basic python type (including dict)
                    return data

            @marshmallow.post_dump
            def _build_ros_type(self, data):
                data = self._generated_ros_type(**data)
                return data

        return Wrapper
    return schema_explicitly_matched_type_decorator




from .exceptions import PyrosSchemasServiceRequestException, PyrosSchemasServiceResponseException
from .schemagic import create

from functools import wraps


# TODO : exceptions with meaningful web error code explanation OR proper Exception forwarding...


def with_service_schemas(service_class):
    def with_service_schemas_decorator(func):
        @wraps(func)
        # TODO : handle funcitons AND methods ?
        def func_wrapper(*data):  # we need to expose only one argument for ROS or two for methods

            try:
                request_schema = create(service_class._request_class)
                data_dict, errors = request_schema.load(data[-1])  # we assume the last argument always contains the ROS data
            except Exception as e:
                raise PyrosSchemasServiceRequestException(e)

            # we should call the function with original and parsed argument,
            # including potential errors, just in case, at least temporarily...
            data_extended = data + (data_dict, errors)
            response = func(*data_extended)
            #  we also let the function trigger its own exceptions

            try:
                response_schema = create(service_class._response_class)
                response_ros, errors = response_schema.dump(response)

                return response_ros
            except Exception as e:
                raise PyrosSchemasServiceResponseException(e)

        return func_wrapper
    return with_service_schemas_decorator


# TODO : decorators for pub/subs
