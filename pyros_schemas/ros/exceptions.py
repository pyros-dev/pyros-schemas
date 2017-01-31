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
