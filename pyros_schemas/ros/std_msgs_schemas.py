from __future__ import absolute_import
from __future__ import print_function

"""
Defining Schema for basic ros types

Ref : http://wiki.ros.org/std_msgs

These Fields and Schema are meant to be used together with ROS message type serialization :
ROSTCP --deserialize in rospy--> std_msgs.msg.* --serialize (dump) in pyros_schemas--> dict
And reversely :
dict --deserialize (load) in pyros_schemas--> std_msgs.msg.* --serialize in rospy--> ROSTCP

This helps pyros deal with data only as dicts without worrying about the underlying ROS implementation.
Also some serialization behavior adjustments have been done :

- optional fields

"""

# This is useful only if we need relative imports. Ref : http://stackoverflow.com/a/28154841/4006172
# declaring __package__ if needed (this module is run individually)
if __package__ is None and not __name__.startswith('pyros_schemas.ros.'):
    import os
    import sys
    from pathlib2 import Path
    top = Path(__file__).resolve().parents[2]
    # Or
    # from os.path import abspath, dirname
    #
    # top = abspath(__file__)
    # for _ in range(4):
    #     top = dirname(top)
    if sys.path[0] == os.path.dirname(__file__):
        sys.path[0] = str(top)  # we replace first path in list (current module dir path) by the path of the package.
        # this avoid unintentional relative import (even without point notation).
    else:  # not sure in which case this could happen, but just in case we don't want to break stuff
        sys.path.append(str(top))
    # sys.path.append(top)

    # If the caller didn't set name properly :
    if __name__ == '__main__':
        __name__ = 'std_msgs_schemas'

    __package__ = 'pyros_schemas.ros'
    # Need import here to get it in sys.modules list. Otherwise we get exception:
    #   Parent module 'pyros_schemas.ros' not loaded, cannot perform relative import
    import pyros_schemas.ros


try:
    import std_msgs.msg as std_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs.msg as std_msgs

# we do this late to give priority to the version from ros package if available.
# It s more recent than system version, and if running from python venv, venv version should take priority.
import marshmallow

# To be able to run doctest directly we avoid relative import
from .decorators import with_explicitly_matched_type
from .fields import (
    RosBool,
    RosInt8, RosInt16, RosInt32, RosInt64,
    RosUInt8, RosUInt16, RosUInt32, RosUInt64,
    RosFloat32, RosFloat64,
    RosString, RosTextString,
    RosTime,
    RosDuration,
)


#
# Schemas declaration
# Since we want to provide seamless but safe rospy message type <-> pyros dict conversion
# We need to validate on serialization (dump to dict)
# and create a rospy message type on deserialization (load from dict)
#

@with_explicitly_matched_type(std_msgs.Empty)
class RosMsgEmpty(marshmallow.Schema):
    """
    RosMsgBool Schema handles serialization from std_msgs.msgs.Bool to python dict
    and deserialization from python dict to std_msgs.msgs.Bool

    >>> schema = RosMsgEmpty()

    >>> rosmsg = std_msgs.Empty()
    >>> marshalled, errors = schema.dump(rosmsg)
    >>> marshmallow.pprint(marshalled) if not errors else print("ERRORS {0}".format(errors))
    {}
    >>> value, errors = schema.load(marshalled)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Empty.Empty'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    <BLANKLINE>

    Load is the inverse of dump (if we ignore possible errors):
    >>> import random
    >>> randomRosEmpty = std_msgs.Empty()
    >>> schema.load(schema.dump(randomRosEmpty).data).data == randomRosEmpty
    True
    """
    pass


@with_explicitly_matched_type(std_msgs.Bool)
class RosMsgBool(marshmallow.Schema):
    """
    RosMsgBool Schema handles serialization from std_msgs.msgs.Bool to python dict
    and deserialization from python dict to std_msgs.msgs.Bool

    >>> schema = RosMsgBool()

    >>> rosmsgTrue = std_msgs.Bool(data=True)
    >>> marshalledTrue, errors = schema.dump(rosmsgTrue)
    >>> marshmallow.pprint(marshalledTrue) if not errors else print("ERRORS {0}".format(errors))
    {u'data': True}
    >>> value, errors = schema.load(marshalledTrue)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Bool.Bool'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: True

    >>> rosmsgFalse = std_msgs.Bool(data=False)
    >>> marshalledFalse, errors = schema.dump(rosmsgFalse)
    >>> marshmallow.pprint(marshalledFalse) if not errors else print("ERRORS {0}".format(errors))
    {u'data': False}
    >>> value, errors = schema.load(marshalledFalse)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Bool.Bool'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: False

    Load is the inverse of dump (if we ignore possible errors):
    >>> import random
    >>> randomRosBool = std_msgs.Bool(data=random.choice([True, False]))
    >>> schema.load(schema.dump(randomRosBool).data).data == randomRosBool
    True
    """
    data = RosBool()


@with_explicitly_matched_type(std_msgs.Int8)
class RosMsgInt8(marshmallow.Schema):
    """
    RosMsgInt8 handles serialization from std_msgs.Int8 to python dict
    and deserialization from python dict to std_msgs.Int8

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgInt8(strict=True)

    >>> rosmsgFortytwo = std_msgs.Int8(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 42}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Int8.Int8'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 42

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt16(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._Int8.Int8'>

    Note since PEP https://www.python.org/dev/peps/pep-0237/  (python 2.4) int and long should mean the same thing for python

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosInt = std_msgs.Int8(random.choice([4, 2, 1]))
    >>> schema.load(schema.dump(randomRosInt).data).data == randomRosInt
    True
    """
    data = RosInt8()


@with_explicitly_matched_type(std_msgs.UInt8)
class RosMsgUInt8(marshmallow.Schema):
    """
    RosMsgUInt8 handles serialization from std_msgs.UInt8 to python dict
    and deserialization from python dict to std_msgs.UInt8

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgUInt8(strict=True)

    >>> rosmsgFortytwo = std_msgs.UInt8(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 42}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._UInt8.UInt8'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 42

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt16(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._UInt8.UInt8'>

    Note since PEP https://www.python.org/dev/peps/pep-0237/  (python 2.4) int and long should mean the same thing for python

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosInt = std_msgs.UInt8(random.choice([4, 2, 1]))
    >>> schema.load(schema.dump(randomRosInt).data).data == randomRosInt
    True
    """
    data = RosUInt8()


@with_explicitly_matched_type(std_msgs.Int16)
class RosMsgInt16(marshmallow.Schema):
    """
    RosMsgInt16 handles serialization from std_msgs.Int16 to python dict
    and deserialization from python dict to std_msgs.Int16

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgInt16(strict=True)

    >>> rosmsgFortytwo = std_msgs.Int16(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 42}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Int16.Int16'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 42

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt8(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._Int16.Int16'>

    Note since PEP https://www.python.org/dev/peps/pep-0237/  (python 2.4) int and long should mean the same thing for python

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosInt = std_msgs.Int16(random.choice([4, 2, 1]))
    >>> schema.load(schema.dump(randomRosInt).data).data == randomRosInt
    True
    """
    data = RosInt16()


@with_explicitly_matched_type(std_msgs.UInt16)
class RosMsgUInt16(marshmallow.Schema):
    """
    RosMsgUInt16 handles serialization from std_msgs.UInt16 to python dict
    and deserialization from python dict to std_msgs.UInt16

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgUInt16(strict=True)

    >>> rosmsgFortytwo = std_msgs.UInt16(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 42}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._UInt16.UInt16'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 42

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt8(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._UInt16.UInt16'>

    Note since PEP https://www.python.org/dev/peps/pep-0237/  (python 2.4) int and long should mean the same thing for python

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosInt = std_msgs.UInt16(random.choice([4, 2, 1]))
    >>> schema.load(schema.dump(randomRosInt).data).data == randomRosInt
    True
    """
    data = RosUInt16()


@with_explicitly_matched_type(std_msgs.Int32)
class RosMsgInt32(marshmallow.Schema):
    """
    RosMsgInt32 handles serialization from std_msgs.Int32 to python dict
    and deserialization from python dict to std_msgs.Int32

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgInt32(strict=True)

    >>> rosmsgFortytwo = std_msgs.Int32(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 42}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Int32.Int32'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 42

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt8(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._Int32.Int32'>

    Note since PEP https://www.python.org/dev/peps/pep-0237/  (python 2.4) int and long should mean the same thing for python

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosInt = std_msgs.Int32(random.choice([4, 2, 1]))
    >>> schema.load(schema.dump(randomRosInt).data).data == randomRosInt
    True
    """
    data = RosInt32()


@with_explicitly_matched_type(std_msgs.UInt32)
class RosMsgUInt32(marshmallow.Schema):
    """
    RosMsgUInt32 handles serialization from std_msgs.UInt32 to python dict
    and deserialization from python dict to std_msgs.UInt32

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgUInt32(strict=True)

    >>> rosmsgFortytwo = std_msgs.UInt32(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 42}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._UInt32.UInt32'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 42

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt8(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._UInt32.UInt32'>

    Note since PEP https://www.python.org/dev/peps/pep-0237/  (python 2.4) int and long should mean the same thing for python

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosInt = std_msgs.UInt32(random.choice([4, 2, 1]))
    >>> schema.load(schema.dump(randomRosInt).data).data == randomRosInt
    True
    """
    data = RosUInt32()


@with_explicitly_matched_type(std_msgs.Int64)
class RosMsgInt64(marshmallow.Schema):
    """
    RosMsgInt64 handles serialization from std_msgs.Int64 to python dict
    and deserialization from python dict to std_msgs.Int64

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgInt64(strict=True)

    >>> rosmsgFortytwo = std_msgs.Int64(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 42}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Int64.Int64'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 42

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt8(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._Int64.Int64'>

    Note since PEP https://www.python.org/dev/peps/pep-0237/  (python 2.4) int and long should mean the same thing for python

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosInt = std_msgs.Int64(random.choice([4, 2, 1]))
    >>> schema.load(schema.dump(randomRosInt).data).data == randomRosInt
    True
    """
    data = RosInt64()


@with_explicitly_matched_type(std_msgs.UInt64)
class RosMsgUInt64(marshmallow.Schema):
    """
    RosMsgUInt64 handles serialization from std_msgs.UInt64 to python dict
    and deserialization from python dict to std_msgs.UInt64

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgUInt64(strict=True)

    >>> rosmsgFortytwo = std_msgs.UInt64(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 42}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._UInt64.UInt64'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 42

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt8(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._UInt64.UInt64'>

    Note since PEP https://www.python.org/dev/peps/pep-0237/  (python 2.4) int and long should mean the same thing for python

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosInt = std_msgs.UInt64(random.choice([4, 2, 1]))
    >>> schema.load(schema.dump(randomRosInt).data).data == randomRosInt
    True
    """
    data = RosUInt64()


@with_explicitly_matched_type(std_msgs.Float32)
class RosMsgFloat32(marshmallow.Schema):
    """
    RosMsgFloat32 handles serialization from std_msgs.Float32 to python dict
    and deserialization from python dict to std_msgs.Float32

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgFloat32(strict=True)

    >>> rosmsgFourtwo = std_msgs.Float32(data=4.2)
    >>> marshalledFourtwo, errors = schema.dump(rosmsgFourtwo)
    >>> marshmallow.pprint(marshalledFourtwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 4.2}
    >>> value, errors = schema.load(marshalledFourtwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Float32.Float32'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 4.2

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt16(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._Float32.Float32'>

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosFloat = std_msgs.Float32(random.choice([4.2, 2.1, 1.0]))
    >>> schema.load(schema.dump(randomRosFloat).data).data == randomRosFloat
    True
    """
    data = RosFloat32()


@with_explicitly_matched_type(std_msgs.Float64)
class RosMsgFloat64(marshmallow.Schema):
    """
    RosMsgFloat64 handles serialization from std_msgs.Float64 to python dict
    and deserialization from python dict to std_msgs.Float64

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgFloat64(strict=True)

    >>> rosmsgFourtwo = std_msgs.Float64(data=4.2)
    >>> marshalledFourtwo, errors = schema.dump(rosmsgFourtwo)
    >>> marshmallow.pprint(marshalledFourtwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 4.2}
    >>> value, errors = schema.load(marshalledFourtwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Float64.Float64'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: 4.2

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt16(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._Float64.Float64'>

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosFloat = std_msgs.Float64(random.choice([4.2, 2.1, 1.0]))
    >>> schema.load(schema.dump(randomRosFloat).data).data == randomRosFloat
    True
    """
    data = RosFloat64()


@with_explicitly_matched_type(std_msgs.String)
class RosMsgString(marshmallow.Schema):
    """
    RosMsgString handles serialization from std_msgs.String to python dict
    and deserialization from python dict to std_msgs.String

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgString(strict=True)

    >>> rosmsgFortytwo = std_msgs.String(data='fortytwo')
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    >>> marshmallow.pprint(marshalledFortytwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': 'fortytwo'}
    >>> value, errors = schema.load(marshalledFortytwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._String.String'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    data: fortytwo

    Invalidate message would report:
    >>> rosmsgFortytwo = std_msgs.UInt16(data=42)
    >>> marshalledFortytwo, errors = schema.dump(rosmsgFortytwo)
    Traceback (most recent call last):
     ...
    ValidationError: data type should be <class 'std_msgs.msg._String.String'>

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosString = std_msgs.String(random.choice(['four', 'two', 'one']))
    >>> schema.load(schema.dump(randomRosString).data).data == randomRosString
    True
    """
    data = RosString()


@with_explicitly_matched_type(std_msgs.Time)
class RosMsgTime(marshmallow.Schema):
    """
    RosMsgTime handles serialization from std_msgs.Time to python dict
    and deserialization from python dict to std_msgs.Time

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgTime(strict=True)
    >>> import rospy
    >>> rosmsgFourtwo = std_msgs.Time(rospy.Time(secs = 42, nsecs = 123456789))
    >>> marshalledFourtwo, errors = schema.dump(rosmsgFourtwo)
    >>> marshmallow.pprint(marshalledFourtwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': {u'nsecs': 123456789, u'secs': 42}}
    >>> value, errors = schema.load(marshalledFourtwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Time.Time'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))  # doctest: +NORMALIZE_WHITESPACE
    data:
      secs: 42
      nsecs: 123456789

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosTime = std_msgs.Time()
    >>> randomRosTime.data.secs = random.choice([4, 2, 1])
    >>> randomRosTime.data.nsecs = random.choice([123, 456, 789])
    >>> schema.load(schema.dump(randomRosTime).data).data == randomRosTime
    True
    """
    data = RosTime()


@with_explicitly_matched_type(std_msgs.Duration)
class RosMsgDuration(marshmallow.Schema):
    """
    RosMsgTime handles serialization from std_msgs.Time to python dict
    and deserialization from python dict to std_msgs.Time

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgDuration(strict=True)

    >>> rosmsgFourtwo = std_msgs.Duration()
    >>> rosmsgFourtwo.data.secs = 42
    >>> rosmsgFourtwo.data.nsecs = 123456789
    >>> marshalledFourtwo, errors = schema.dump(rosmsgFourtwo)
    >>> marshmallow.pprint(marshalledFourtwo) if not errors else print("ERRORS {0}".format(errors))
    {u'data': {u'nsecs': 123456789, u'secs': 42}}
    >>> value, errors = schema.load(marshalledFourtwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Duration.Duration'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))  # doctest: +NORMALIZE_WHITESPACE
    data:
      secs: 42
      nsecs: 123456789

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosDuration = std_msgs.Duration()
    >>> randomRosDuration.data.secs = random.choice([4, 2, 1])
    >>> randomRosDuration.data.nsecs = random.choice([123, 456, 789])
    >>> schema.load(schema.dump(randomRosDuration).data).data == randomRosDuration
    True
    """
    data = RosDuration()


@with_explicitly_matched_type(std_msgs.Header)
class RosMsgHeader(marshmallow.Schema):
    """
    RosMsgTime handles serialization from std_msgs.Time to python dict
    and deserialization from python dict to std_msgs.Time

    You should use strict Schema to trigger exceptions when trying to manipulate an unexpected type.

    >>> schema = RosMsgHeader(strict=True)
    >>> import rospy
    >>> rosmsgFourtwo = std_msgs.Header(seq = 42, stamp= rospy.Time(secs=53, nsecs=123456), frame_id='fortytwo')
    >>> marshalledFourtwo, errors = schema.dump(rosmsgFourtwo)
    >>> marshmallow.pprint(marshalledFourtwo) if not errors else print("ERRORS {0}".format(errors))
    {u'frame_id': u'fortytwo',
     u'seq': 42,
     u'stamp': {u'nsecs': 123456, u'secs': 53}}
    >>> value, errors = schema.load(marshalledFourtwo)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'std_msgs.msg._Header.Header'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))  # doctest: +NORMALIZE_WHITESPACE
    seq: 42
    stamp:
      secs: 53
      nsecs:    123456
    frame_id: fortytwo

    Load is the inverse of dump (getting only data member):
    >>> import random
    >>> randomRosHeader = std_msgs.Header(seq = random.choice([42, 53]), stamp= rospy.Time(secs=random.choice([4, 2, 1]), nsecs=random.choice([123, 456, 789])), frame_id='random')
    >>> schema.load(schema.dump(randomRosHeader).data).data == randomRosHeader
    True
    """
    seq = RosUInt32()
    stamp = RosTime()
    frame_id = RosTextString()
