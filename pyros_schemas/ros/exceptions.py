# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function


class PyrosSchemasException(Exception):
    """Base class for pyros_schemas exceptions"""
    pass


class PyrosSchemasValidationError(PyrosSchemasException):
    """Exception class to wrap the marshmallow ValidationError Exception"""
    pass


class PyrosSchemasServiceRequestException(PyrosSchemasException):
    """Base class for pyros_schemas service-related exceptions"""
    pass


class PyrosSchemasServiceResponseException(PyrosSchemasException):
    """Base class for pyros_schemas service-related exceptions"""
    pass