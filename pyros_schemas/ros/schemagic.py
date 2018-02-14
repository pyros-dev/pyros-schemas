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

from .types_mapping import ros_msgtype_mapping
from .utils import _get_msg_class, _get_rosmsg_fields_as_dict
from .schema import RosSchema, pre_load, post_load, pre_dump, post_dump


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
    for s, stype in six.iteritems(members_types):
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
