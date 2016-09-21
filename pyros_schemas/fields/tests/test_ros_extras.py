from __future__ import absolute_import
from __future__ import print_function

import functools
import nose

try:
    import std_msgs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us ot the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs


# import ros field types
from pyros_schemas.fields import (
    RosTextString,
)

from pyros_schemas.fields.tests.test_ros import fromros, frompy


#
# Testing each ros extra field type
#

def test_ros_field_textstring():
    yield fromros, std_msgs.msg.String(data='fortytwo'), RosTextString, std_msgs.msg.String, unicode, str
    # this should except since Ros string field accepts unicode as data without validation,
    # but then something will break later on...
    yield fromros, std_msgs.msg.String(data=u'fortytwo'), RosTextString, std_msgs.msg.String, unicode, str, (AssertionError,)
    # also test None and default value
    yield fromros, std_msgs.msg.String(data=None), RosTextString, std_msgs.msg.String, unicode, str
    yield fromros, std_msgs.msg.String(), RosTextString, std_msgs.msg.String, unicode, str

    # Reverse test

    # Test all explicit values when possible, or pick a few meaningful ones
    yield frompy, 'fortytwo', RosTextString, std_msgs.msg.String, unicode, str
    yield frompy, u'fortytwo', RosTextString, std_msgs.msg.String, unicode, str
    # also test (None ?) and default value
    # TMP not doing this for now... until we define proper behavior
    # yield frompy, None, RosString, std_msgs.msg.String, unicode, str
    yield frompy, str(), RosTextString, std_msgs.msg.String, unicode, str
    yield frompy, unicode(), RosTextString, std_msgs.msg.String, unicode, str

# # Since the rospy message type member field is already a python int,
# # we do not need anything special here, we rely on marshmallow python type validation.
# # Yet we are specifying each on in case we want to extend it later...
#

# RosString = functools.partial(marshmallow.fields.String, required=True)
#
# # CAREFUL with RosNested : Ros doesnt not allow
# RosNested = functools.partial(marshmallow.fields.Nested, required=True)


# Just in case we run this directly
if __name__ == '__main__':
    nose.runmodule(__name__)
