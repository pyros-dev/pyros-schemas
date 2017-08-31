from __future__ import absolute_import
from __future__ import print_function

import pytest

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
