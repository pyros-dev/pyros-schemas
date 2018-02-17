from __future__ import absolute_import, division, print_function, unicode_literals

import os
import site

added_site_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'rosdeps')
srvs_site_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'rosdeps', 'ros_comm_msgs')
print("Adding site directory {0} to access std_msgs".format(added_site_dir))
site.addsitedir(added_site_dir)
site.addsitedir(srvs_site_dir)

import rosimport
rosimport.activate()


from . import msg as pyros_schemas_test_msgs

# patching (need to know the field name)
import pyros_msgs.opt_as_array
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_bool_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_int8_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_int16_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_int32_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_int64_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_uint8_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_uint16_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_uint32_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_uint64_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_float32_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_float64_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_string_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_time_as_array, ['data'])
pyros_msgs.opt_as_array.duck_punch(pyros_schemas_test_msgs.test_opt_duration_as_array, ['data'])


import hypothesis
import hypothesis.strategies as st

import six
six_long = six.integer_types[-1]


def maybe_list(l):
    """Return list of one element if ``l`` is a scalar."""
    return l if l is None or isinstance(l, list) else [l]





from .strategies.ros import std_msgs_types_strat_ok, std_msgs_dicts_strat_ok



from .strategies.ros import pyros_schemas_opttypes_strat_ok, pyros_schemas_dicts_strat_ok




#
# def proper_basic_msg_strategy_selector(*msg_types):
#     """Accept a (list of) rostype and return it with the matching strategy for ros message"""
#     # TODO : break on error (type not in map)
#     # we use a list comprehension here to avoid creating a generator (tuple comprehension)
#     return tuple([(msg_type, std_msgs_types_strat_ok.get(msg_type)) for msg_type in msg_types])
#
#
# def proper_basic_dict_strategy_selector(*msg_types):
#     """Accept a (list of) rostype and return it with the matching strategy for dict"""
#     # TODO : break on error (type not in map)
#     # we use a list comprehension here to avoid creating a generator (tuple comprehension)
#     return tuple([(msg_type, std_msgs_dicts_strat_ok.get(msg_type)) for msg_type in msg_types])


# def proper_basic_optmsg_strategy_selector(*msg_types):
#     """Accept a (list of) rostype and return it with the matching strategy for ros message"""
#     # TODO : break on error (type not in map)
#     # we use a list comprehension here to avoid creating a generator (tuple comprehension)
#     return tuple([(msg_type, pyros_schemas_opttypes_strat_ok.get(msg_type)) for msg_type in msg_types])
#
#
# def proper_basic_optdict_strategy_selector(*msg_types):
#     """Accept a (list of) rostype and return it with the matching strategy for dict"""
#     # TODO : break on error (type not in map)
#     # we use a list comprehension here to avoid creating a generator (tuple comprehension)
#     return tuple([(msg_type, pyros_schemas_dicts_strat_ok.get(msg_type)) for msg_type in msg_types])
#
#
# def proper_basic_optdata_strategy_selector(*field_types):
#     """Accept a (list of) rostype and return it with the matching strategy for data"""
#     # TODO : break on error (type not in map)
#     # we use a list comprehension here to avoid creating a generator (tuple comprehension)
#     return tuple([(field_type, optfield_strat_ok.get(field_type)) for field_type in field_types])


