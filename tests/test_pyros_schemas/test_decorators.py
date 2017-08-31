from __future__ import absolute_import
from __future__ import print_function

import pytest

try:
    import std_msgs.msg as std_msgs
    import std_srvs.srv as std_srvs
except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration to point us to the proper distro
    pyros_setup.configurable_import().configure().activate()
    import std_msgs.msg as std_msgs  # TODO
    import std_srvs.srv as std_srvs

# public decorators
from pyros_schemas.ros import with_service_schemas


#
# Testing with_service_schemas decorator
#
@with_service_schemas(std_srvs.Trigger)
def service_callback(data, data_dict, error):
    # From spec http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
    assert len(data_dict) == len(data.__slots__) == 0

    return {
        'success': True,
        'message': 'fortytwo',
    }


def test_decorated_service():
    resp = service_callback(std_srvs.TriggerRequest())

    assert isinstance(resp, std_srvs.TriggerResponse)
    assert resp.success == True
    assert resp.message == 'fortytwo'


# Just in case we run this directly
if __name__ == '__main__':
    pytest.main([
        'test_decorators.py::test_decorated_service'
    ])
