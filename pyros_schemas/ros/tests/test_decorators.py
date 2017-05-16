from __future__ import absolute_import
from __future__ import print_function

import collections

import marshmallow
import nose


# "private" decorators
from pyros_schemas.ros.decorators import wraps_cls

# public decorators
from pyros_schemas.ros.decorators import with_explicitly_matched_type #, with_explicitly_matched_optional_type

# TODO : test with_service_schemas

#
# Testing generic class wraps_cls decorator
#

class WrappedCheck(object):
    """ test doc value """
    pass


@wraps_cls(WrappedCheck)
class WrapperCheck(WrappedCheck):
    __doc__ = WrappedCheck.__doc__  # to work with python 2.7 check http://bugs.python.org/issue12773
    # TODO : dynamically define this using functools assignments

    @classmethod
    def get_module(cls):
        return cls.__module__

    @classmethod
    def get_name(cls):
        return cls.__name__

    @classmethod
    def get_doc(cls):
        return cls.__doc__


def test_wrap_cls():
    # TODO : dynamically define this using functools assignments
    # TODO : check these may be trivial... but better be safe
    assert WrappedCheck.__module__ == WrapperCheck.get_module()
    assert WrappedCheck.__doc__ == WrapperCheck.get_doc()
    assert WrappedCheck.__name__ == WrapperCheck.get_name()

    assert WrapperCheck.__module__ == __name__
    assert WrapperCheck.__doc__ == """ test doc value """
    assert WrapperCheck.__name__ == "WrappedCheck"



#
# Testing with_explicitly_matched_type decorator
#

# answer is assumed int here (although not enforced)
Original = collections.namedtuple("Original", "answer")


@with_explicitly_matched_type(Original)
class SchemaWithValidatedGeneratedType(marshmallow.Schema):
    """ doc test """
    answer = marshmallow.fields.Integer()


def test_with_validated_generated_type():

    assert SchemaWithValidatedGeneratedType.__module__ == __name__
    assert SchemaWithValidatedGeneratedType.__doc__ == """ doc test """
    assert SchemaWithValidatedGeneratedType.__name__ == "SchemaWithValidatedGeneratedType"

    original_ok = Original(answer=42)
    original_invalid = Original(answer='fortytwo')
    schema = SchemaWithValidatedGeneratedType(strict=True)  # we usually want to be strict and explicitely fail.

    # Testing deserialization
    unmarshalled = schema.load(original_ok)

    assert len(unmarshalled.errors) == 0
    assert unmarshalled.data == {'answer': 42}, "unmarshalled.data is {unmarshalled.data}".format(**locals())

    # Verifying validation actually happens
    with nose.tools.assert_raises(marshmallow.ValidationError) as cm:
        schema.load(original_invalid)

    # Testing serialization
    marshalled = schema.dump(unmarshalled.data)
    assert len(marshalled.errors) == 0
    assert marshalled.data == original_ok


# Just in case we run this directly
if __name__ == '__main__':
    nose.runmodule(__name__)
