from __future__ import absolute_import
from __future__ import print_function

"""
Defining Schema for http request/response types

These Fields and Schema are meant to be used together with ROS message type serialization :
ROSTCP --deserialize in rospy--> std_msgs.msg.* --serialize (dump) in pyros_schemas--> dict
And reversely :
dict --deserialize (load) in pyros_schemas--> std_msgs.msg.* --serialize in rospy--> ROSTCP

This helps pyros deal with data only as dicts without worrying about the underlying ROS implementation.
"""

import marshmallow
try:
    import pyros_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import pyros_msgs

# This is useful only if we need relative imports. Ref : http://stackoverflow.com/a/28154841/4006172
# declaring __package__ if needed (this module is run individually, via doctest or python call)
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


# To be able to run doctest directly we avoid relative import
from .decorators import with_explicitly_matched_type
from .std_Int import RosFieldUInt16


@with_explicitly_matched_type(pyros_msgs.HttpStatusCode)
class RosMsgHttpStatusCode(marshmallow.Schema):
    """
    RosMsgBool Schema handles serialization from std_msgs.msgs.Bool to python dict
    and deserialization from python dict to std_msgs.msgs.Bool

    >>> schema = RosMsgHttpStatusCode()

    >>> rosmsgTrue = pyros_msgs.HttpStatusCode(code=pyros_msgs.HttpStatusCode.OK)
    >>> marshalledTrue, errors = schema.dump(rosmsgTrue)
    >>> marshmallow.pprint(marshalledTrue) if not errors else print("ERRORS {0}".format(errors))
    {u'code': 200}
    >>> value, errors = schema.load(marshalledTrue)
    >>> type(value) if not errors else print("ERRORS {0}".format(errors))
    <class 'pyros_msgs.msg._HttpStatusCode.HttpStatusCode'>
    >>> print(value) if not errors else print("ERRORS {0}".format(errors))
    code: 200

    Load is the inverse of dump (if we ignore possible errors):
    >>> import random
    >>> randomStatus = pyros_msgs.HttpStatusCode(code=random.choice([pyros_msgs.HttpStatusCode.OK, pyros_msgs.HttpStatusCode.BAD]))
    >>> schema.load(schema.dump(randomStatus).data).data == randomStatus
    True
    """
    code = RosFieldUInt16()


