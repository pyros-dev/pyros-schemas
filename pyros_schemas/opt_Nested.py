from __future__ import absolute_import
from __future__ import print_function

"""
Defining Nested field for optional ros types.

This is useful to make optional ROS types (message) behave like a field (the data field exactly)

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


from .decorators import with_explicitly_matched_optional_type
from pyros_msgs import opt_string

# From here we can pick this up from ROS if missing in python env.
import marshmallow


# Keeping field declaration separate in case we want to extend it later
class RosFieldOptNested(marshmallow.fields.Nested):
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


    def _validate_missing(self, value):
        """Validate missing values. Raise a :exc:`ValidationError` if
        `value` should be considered missing.
        """
        if value is missing_ and hasattr(self, 'required'):
            if self.nested == _RECURSIVE_NESTED:
                self.fail('required')
            errors = self._check_required()
            if errors:
                raise ValidationError(errors)
        else:
            super(Nested, self)._validate_missing(value)


    def _check_required(self):
        errors = {}
        if self.required:
            for field_name, field in self.schema.fields.items():
                if not field.required:
                    continue
                error_field_name = field.load_from or field_name
                if (
                                isinstance(field, Nested) and
                                    self.nested != _RECURSIVE_NESTED and
                                field.nested != _RECURSIVE_NESTED
                ):
                    errors[error_field_name] = field._check_required()
                else:
                    try:
                        field._validate_missing(field.missing)
                    except ValidationError as ve:
                        errors[error_field_name] = ve.messages
            if self.many and errors:
                errors = {0: errors}
            # No inner errors; just raise required error like normal
            if not errors:
                self.fail('required')
        return errors
