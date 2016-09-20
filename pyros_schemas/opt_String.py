from __future__ import absolute_import
from __future__ import print_function

"""
Defining Schema for optional ros types

These add a boolean "initialized_" field to basic ros types, to allow for a field being there, or not.

These Fields and Schema are meant to be used together with ROS message type serialization :
ROSTCP --deserialize in rospy--> std_msgs.msg.* --serialize (dump) in pyros_schemas--> dict
And reversely :
dict --deserialize (load) in pyros_schemas--> std_msgs.msg.* --serialize in rospy--> ROSTCP

This helps pyros deal with data only as dicts without worrying about the underlying ROS implementation.

"""

try:
    # To be able to run doctest directly
    import pyros_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import pyros_msgs

# This is useful only if we need relative imports. Ref : http://stackoverflow.com/a/28154841/4006172
# declaring __package__ if needed (this module is run individually)
if __package__ is None and not __name__.startswith('pyros_schemas.'):
    import sys
    from pathlib2 import Path
    top = Path(__file__).resolve().parents[1]
    sys.path.append(str(top))
    # Or
    # from os.path import abspath, dirname
    #
    # top = abspath(__file__)
    # for _ in range(4):
    #     top = dirname(top)
    # sys.path.append(top)

    import pyros_schemas
    __package__ = 'pyros_schemas'


from .decorators import with_explicitly_matched_type, with_explicitly_matched_optional_type
from pyros_msgs import opt_string

# From here we can pick this up from ROS if missing in python env.
import marshmallow

# Keeping field declaration separate in case we want to extend it later
RosFieldString = marshmallow.fields.String


class OptString(marshmallow.fields.Nested):
    # Ref : http://marshmallow.readthedocs.io/en/latest/_modules/marshmallow/fields.html#Nested

    @with_explicitly_matched_type(opt_string)
    class RosMsgOptStringSchema(marshmallow.Schema):
        """
        Internal Schema to handle optional string

        RosMsgOptStringSchema Schema handles serialization from pyros_msgs.opt_string to python dict
        and deserialization from python dict to pyros_msgs.opt_string

        >>> schema = RosMsgOptStringSchema(strict=True)

        >>> rosmsgAnswer = pyros_msgs.opt_string(data='fortytwo')
        >>> marshalledAnswer, errors = schema.dump(rosmsgAnswer)
        >>> marshmallow.pprint(marshalledAnswer) if not errors else print("ERRORS {0}".format(errors))
        {u'data': u'fortytwo'}
        >>> value, errors = schema.load(marshalledAnswer)
        >>> type(value) if not errors else print("ERRORS {0}".format(errors))
        <class 'pyros_msgs.msg._opt_string.opt_string'>
        >>> print(value) if not errors else print("ERRORS {0}".format(errors))
        initialized_: True
        data: fortytwo

        data field is optional. If not passed in original message object, it will be set to a default value for ROS, but it will not be in serialized dict
        >>> rosmsgUninit = pyros_msgs.opt_string()
        >>> marshalledUninit, errors = schema.dump(rosmsgUninit)
        >>> marshmallow.pprint(marshalledUninit) if not errors else print("ERRORS {0}".format(errors))
        {}
        >>> value, errors = schema.load(marshalledUninit)
        >>> type(value) if not errors else print("ERRORS {0}".format(errors))
        <class 'pyros_msgs.msg._opt_string.opt_string'>
        >>> print(value) if not errors else print("ERRORS {0}".format(errors))
        initialized_: False
        data: ''

        Careful : passing 'initialized_' to the constructor will except.
        It is only an "internal" field and is not meant to be manipulated
        >>> rosmsgforcedInit = pyros_msgs.opt_bool(initialized_=True)
        Traceback (most recent call last):
         ...
        AttributeError: The field 'initialized_' is an internal field of pyros_msgs/opt_bool and should not be set by the user.

        Load is the inverse of dump (if we ignore possible errors):
        >>> import random
        >>> randomRosString = pyros_msgs.opt_string(data=random.choice(['fortytwo', 'twentyone']))
        >>> schema.load(schema.dump(randomRosString).data).data == randomRosString
        True


        Reversely if you start by loading from a python dict :
        >>> value, errors = schema.load({'data': 'fortytwo'})
        >>> type(value) if not errors else print("ERRORS {0}".format(errors))
        <class 'pyros_msgs.msg._opt_string.opt_string'>
        >>> print(value) if not errors else print("ERRORS {0}".format(errors))
        initialized_: True
        data: fortytwo

        >>> marshalledAnswer, errors = schema.dump(value)
        >>> marshmallow.pprint(marshalledAnswer) if not errors else print("ERRORS {0}".format(errors))
        {u'data': u'fortytwo'}


        Dump is the inverse of load (if we ignore possible errors):
        >>> import random
        >>> randomString = {'data' : random.choice(['fortytwo', 'twentyone'])}
        >>> w = schema.dump(schema.load(randomString).data).data
        >>> w == randomString
        True

        Note if you need to load from a python object, make use of the marshmallow pre_load decorator

        """
        initialized_ = marshmallow.fields.Boolean(required=True, dump_only=True)
        data = marshmallow.fields.String()

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
                        data.get(
                            f)  # if we can access the field it means we are dealing with a dict (or at least something with a get method)
                    except AttributeError as ae:
                        if len(self.fields) <= 2:  # auto assign here, only one field
                            data = {f: data}
                        else:  # give up and raise
                            raise
            return data


    def _serialize(self, nested_obj, attr, obj):
        # Load up the schema first. This allows a RegistryError to be raised
        # if an invalid schema name was passed
        schema = self.schema
        if nested_obj is None:
            return None
        if not self.__updated_fields:
            schema._update_fields(obj=nested_obj, many=self.many)
            self.__updated_fields = True
        ret, errors = schema.dump(nested_obj, many=self.many,
                                  update_fields=not self.__updated_fields)
        if isinstance(self.only, basestring):  # self.only is a field name
            if self.many:
                return utils.pluck(ret, key=self.only)
            else:
                return ret[self.only]
        if errors:
            raise ValidationError(errors, data=ret)
        return ret

    # TODO : after dump for optional msgs:
    # def remove_empty(self, headers_dict):
    #     # careful : empty dict (from Nested field) must be removed from serialized output
    #     return {h: v for h, v in headers_dict.items() if v}
    # TODO : also collapse the 'data' fields with parent schema


    def _deserialize(self, value, attr, data):
        if self.many and not utils.is_collection(value):
            self.fail('type', input=value, type=value.__class__.__name__)

        data, errors = self.schema.load(value)
        if errors:
            raise ValidationError(errors, data=data)
        return data

    # TODO : after load for optional msgs:



@with_explicitly_matched_optional_type(opt_string)
class PyrosMsgOptString(marshmallow.Schema):
    """
    PyrosMsgOptBool Schema handles serialization from pyros_msgs.opt.Bool to python dict
    and deserialization from python dict to pyros_msgs.opt.Bool

    >>> schema = PyrosMsgOptString(strict=True)

    >>> rosmsgTrue = pyros_msgs.opt_string(data='fortytwo')
    >>> marshalledAnswer, errors = schema.dump(rosmsgTrue)
    >>> marshmallow.pprint(marshalledAnswer) if not errors else print("ERRORS {0}".format(errors))
    {u'data': u'fortytwo'}
    >>> value, errors = schema.load(marshalledAnswer)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_string.opt_string'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True
    data: fortytwo

    data field is optional. If not passed in original message object, it will be set to a default value for ROS, but it will not be in serialized dict
    >>> rosmsgUninit = pyros_msgs.opt_string()
    >>> marshalledUninit, errors = schema.dump(rosmsgUninit)
    >>> marshmallow.pprint(marshalledUninit) if not errors else print("ERRORS {0}".format(errors))
    {}
    >>> value, errors = schema.load(marshalledUninit)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_string.opt_string'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: False
    data: ''

    Careful : passing 'initialized_' to the constructor will except.
    It is only an "internal" field and is not meant to be manipulated
    >>> rosmsgforcedInit = pyros_msgs.opt_bool(initialized_=True)
    Traceback (most recent call last):
     ...
    AttributeError: The field 'initialized_' is an internal field of pyros_msgs/opt_bool and should not be set by the user.

    Load is the inverse of dump (if we ignore possible errors):
    >>> import random
    >>> randomRosString = pyros_msgs.opt_string(data=random.choice(['fortytwo', 'twentyone']))
    >>> schema.load(schema.dump(randomRosString).data).data == randomRosString
    True


    Reversely if you start by loading from a python dict :
    >>> value, errors = schema.load({'data': 'fortytwo'})
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_string.opt_string'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True
    data: fortytwo

    >>> marshalledAnswer, errors = schema.dump(value)
    >>> marshmallow.pprint(marshalledAnswer) if not errors else print("ERRORS {0}".format(errors))
    {u'data': u'fortytwo'}


    Dump is the inverse of load (if we ignore possible errors):
    >>> import random
    >>> randomString = {'data' : random.choice(['fortytwo', 'twentyone'])}
    >>> w = schema.dump(schema.load(randomString).data).data
    >>> w == randomString
    True

    Note if you need to load from a python object, make use of the marshmallow pre_load decorator

    Note also that loading from a python string also works if type is compatible
    >>> value, errors = schema.load('fortytwo')
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._opt_string.opt_string'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    initialized_: True
    data: fortytwo

    >>> marshalledAnswer, errors = schema.dump(value)
    >>> marshmallow.pprint(marshalledAnswer) if not errors else print("ERRORS {0}".format(errors))
    {u'data': u'fortytwo'}

    Or fail if not :
    >>> value, errors = schema.load(42)
    Traceback (most recent call last):
     ...
    ValidationError: {'data': [u'Not a valid string.']}
    """
    data = RosFieldString()







