from __future__ import absolute_import
from __future__ import print_function

"""
Defining field for pyros_msgs optional nested types

It adds logic to handle default, none, null and missing values in a nested schema, given that ROS does NOT allow optional message field.

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


# From here we can pick this up from ROS if missing in python env.
import marshmallow


class OptNested(marshmallow.fields.Nested):
    """
    This Custom field handles optional Nested Schema, to make the usage transparent for the developer
    """

    # Ref : http://marshmallow.readthedocs.io/en/latest/_modules/marshmallow/fields.html#Nested

    def __init__(self, nested, opt_field_name, **kwargs):
        self._opt_field_name = opt_field_name
        super(OptNested, self).__init__(nested, **kwargs)

    def _serialize(self, nested_obj, attr, obj):
        # Adding the schema field

        serialized = super(OptNested, self)._serialize(nested_obj, attr, obj)
        # if data is not in serialized this whole nested field should be considered missing
        serialized = serialized.get(self._opt_field_name, marshmallow.missing)
        return serialized

    def _deserialize(self, value, attr, data):
        # embedding an additional level to be able to have a internal schema transparently
        value = {self._opt_field_name: value}

        deserialized = super(OptNested, self)._deserialize(value, attr, data)
        return deserialized

