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
    RosNested,
    RosList,
)
from .optional_fields import (
    RosOpt,
)

from .utils import _get_rosmsg_members_as_dict

# Statically proxying marshmallow useful decorators for methods
pre_load = marshmallow.pre_load
post_load = marshmallow.post_load
pre_dump = marshmallow.pre_dump
post_dump = marshmallow.post_dump



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
            if isinstance(obj_dict, dict):  # if we are actually a dict
                # because ROS field naming conventions are different than python dict key conventions
                obj_rosfixed_dict = {k.replace('-', '_'): v for k, v in obj_dict.items()}  # TODO : come up with a generic <ROS_field encode> function
            else:  # can be a list -> do nothing
                obj_rosfixed_dict = obj_dict
            data_dict, errors = super(RosSchema, self).dump(obj_rosfixed_dict, many=many, update_fields=update_fields, **kwargs)
        except marshmallow.ValidationError as ve:
            raise PyrosSchemasValidationError('ERROR occurred during serialization: {ve}'.format(**locals()))
        if self._generated_ros_msgtype and not errors:
            obj = self._generated_ros_msgtype(**data_dict)
        else:
            obj = data_dict  # we return directly
        return marshmallow.MarshalResult(obj, errors)


