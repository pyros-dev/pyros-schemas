# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function

import sys


"""
Defining decorators to help with Schema generation for ROS message type <-> pyros dict conversion
This is useful for detailed definition of field serialization, like in schema_fields.py module
"""


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
