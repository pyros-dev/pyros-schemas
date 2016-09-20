from __future__ import absolute_import
from __future__ import print_function

import sys

import marshmallow
import functools

"""
Defining decorators to help with Schema generation for ROS message type <-> pyros dict conversion

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

def with_explicitly_matched_type(ros_type):
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
            # TODO : closure
            # TODO : proxy ?
            # This wrapper inherits. Maybe a proxy would be better ?
            # We cannot have a doc here, because it is not writeable in python 2.7
            # instead we reuse the one from the wrapped class
            __doc__ = cls.__doc__

            @marshmallow.pre_dump
            def _verify_ros_type(self, data, pass_original=True):
                # introspect data
                if not isinstance(data, ros_type):
                    raise marshmallow.ValidationError('data type should be {0}'.format(ros_type))

            @marshmallow.post_load
            def _make_ros_type(self, data):
                data = ros_type(**data)
                return data

        return Wrapper
    return schema_explicitly_matched_type_decorator


def with_explicitly_matched_optional_type(ros_type, opt_field_names=['data']):

    """
    Decorator to add type check and type creation for a schema
    :param ros_type: the ros_type to check for and generate
    :param opt_field_names: list of names for the optional fields
    :return:

    TODO  : doctest
    """

    def schema_explicitly_matched_optional_type_decorator(cls):
        assert isinstance(cls, marshmallow.schema.SchemaMeta)

        # dynamically defining a post_dump method if we are dealing with an optional schema

        @wraps_cls(cls)
        class Wrapper(cls):
            # TODO : closure
            # TODO : proxy ?
            # This wrapper inherits. Maybe a proxy would be better ?
            # We cannot have a doc here, because it is not writeable in python 2.7
            # instead we reuse the one from the wrapped class
            __doc__ = cls.__doc__

            # 'initialized_' is required for the schema, but the user doesnt have to set it explicitly.
            initialized_ = marshmallow.fields.Boolean(required=True, dump_only=True)

            @marshmallow.pre_dump
            def _verify_ros_type(self, data, pass_original=True):
                # introspect data
                if not isinstance(data, ros_type):
                    raise marshmallow.ValidationError('data type should be {0}'.format(ros_type))

            @marshmallow.post_dump
            def unset_data(self, data):
                # if data was not initialized, it means the intent was to not send it in the first place
                if not data['initialized_']:
                    for fn in opt_field_names:
                        data.pop(fn, None)
                data.pop('initialized_')

            @marshmallow.pre_load
            def set_initialized(self, data):
                for f in self.fields:
                    if f != 'initialized_':
                        try:
                            data.get(f)  # if we can access the field it means we are dealing with a dict (or at least something with a get method)
                        except AttributeError as ae:
                            if len(self.fields) <= 2:  # auto assign here, only one field
                                data = {f: data}
                            else:  # give up and raise
                                raise
                return data


            @marshmallow.post_load
            def _make_ros_type(self, data):
                data = ros_type(**data)
                return data

        return Wrapper
    return schema_explicitly_matched_optional_type_decorator
