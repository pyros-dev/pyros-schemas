from __future__ import absolute_import
from __future__ import print_function

"""
Defining field for pyros_msgs optional ros message types

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
if __package__ is None and not __name__.startswith('pyros_schemas.fields.'):
    import sys
    from pathlib2 import Path
    top = Path(__file__).resolve().parents[2]
    sys.path.append(str(top))
    # Or
    # from os.path import abspath, dirname
    #
    # top = abspath(__file__)
    # for _ in range(4):
    #     top = dirname(top)
    # sys.path.append(top)

    import pyros_schemas
    __package__ = 'pyros_schemas.fields'


from ..decorators import with_explicitly_matched_type, with_explicitly_matched_optional_type
from pyros_msgs import opt_string

# From here we can pick this up from ROS if missing in python env.
import marshmallow

# Keeping field declaration separate in case we want to extend it later
RosFieldString = marshmallow.fields.String


@with_explicitly_matched_optional_type(opt_string, opt_field_names=['data'])
class RosMsgOptStringSchema(marshmallow.Schema):
    """
    Internal Schema to handle optional string ROS message type

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


class OptString(marshmallow.fields.Nested):
    """
    This Custom field handles optional String Schema, to make the usage transparent for the developer

    First, lets create a fake ros message class, just for the sake of this documentation:
    >>> import genpy
    >>> class fake_ros_string_msg(genpy.Message):
    ...    _full_text = "pyros_msgs/opt_string a_string"
    ...    __slots__ = ['a_string']
    ...    _slot_types = ['string']
    ...    def __init__(self, *args, **kwds):
    ...      if args or kwds:
    ...        super(fake_ros_string_msg, self).__init__(*args, **kwds)
    ...        #message fields cannot be None, assign default values for those that are
    ...        if self.a_string is None:
    ...          self.a_string = ''
    ...      else:
    ...        self.a_string = ''

    Then we define the corresponding Schema
    >>> class MsgSchemaWithOptString(marshmallow.Schema):
    ...     a_string = OptString()

    Lets not forget to decorate our Schema
    >>> MsgSchemaWithOptString = with_explicitly_matched_type(fake_ros_string_msg)(MsgSchemaWithOptString)

    We can now use this schema
    >>> schema = MsgSchemaWithOptString(strict=True)

    This schema accept a string as usual
    >>> msgdict = {'a_string': 'fortytwo'}
    >>> unmarshalled, errors = schema.load(msgdict)
    >>> type(unmarshalled) if not errors else print("ERRORS {0}".format(errors))
    <class 'opt_string.fake_ros_string_msg'>
    >>> print(unmarshalled) if not errors else print("ERRORS {0}".format(errors))  # doctest: +NORMALIZE_WHITESPACE
    a_string:
      initialized_: True
      data: fortytwo

    >>> marshalled, errors = schema.dump(unmarshalled)
    >>> marshmallow.pprint(marshalled) if not errors else print("ERRORS {0}".format(errors))
    {u'a_string': u'fortytwo'}

    And not something else :
    >>> msgdict = {'a_string': 42}
    >>> unmarshalled, errors = schema.load(msgdict)
    Traceback (most recent call last):
    ...
    ValidationError: {'a_string': {'data': [u'Not a valid string.']}}

    But that string is completely optional in dictionary while producing default in ros message type
    >>> msgdict = {}
    >>> unmarshalled, errors = schema.load(msgdict)
    >>> type(unmarshalled) if not errors else print("ERRORS {0}".format(errors))
    <class 'opt_string.fake_ros_string_msg'>
    >>> print(unmarshalled) if not errors else print("ERRORS {0}".format(errors))
    a_string: ''

    """

    # Ref : http://marshmallow.readthedocs.io/en/latest/_modules/marshmallow/fields.html#Nested

    def __init__(self):
        super(OptString, self).__init__(RosMsgOptStringSchema)

    def _serialize(self, nested_obj, attr, obj):
        # Adding the schema field

        serialized = super(OptString, self)._serialize(nested_obj, attr, obj)
        # if data is not in serialized this whole nested field should be considered missing
        serialized = serialized.get('data', marshmallow.utils.missing)
        return serialized

    # TODO : after dump for optional msgs:
    # def remove_empty(self, headers_dict):
    #     # careful : empty dict (from Nested field) must be removed from serialized output
    #     return {h: v for h, v in headers_dict.items() if v}

    def _deserialize(self, value, attr, data):
        # embedding an additional level to be able to have a internal schema transparently
        value = {'data': value}

        deserialized = super(OptString, self)._deserialize(value, attr, data)
        return deserialized

