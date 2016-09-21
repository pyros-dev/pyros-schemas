from __future__ import absolute_import
from __future__ import print_function

import collections

import marshmallow
import nose


import sys
from pathlib2 import Path

# USEFUL TRICK : keep it around, at least until we have a proper use for it somewhere...
# # This is useful only if we need relative imports. Ref : http://stackoverflow.com/a/28154841/4006172
# # declaring __package__ if needed (this module is run individually)
# if __name__ == '__main__' and __package__ is None:
#     top = Path(__file__).resolve().parents[1]
#     sys.path.append(str(top))
#     # Or
#     # from os.path import abspath, dirname
#     #
#     # top = abspath(__file__)
#     # for _ in range(4):
#     #     top = dirname(top)
#     # sys.path.append(top)
#
#     import pyros_schemas
#     __package__ = 'pyros_schemas.tests'

# "private" decorators
from pyros_schemas import wraps_cls

# public decorators
from pyros_schemas import with_explicitly_matched_type, with_explicitly_matched_optional_type


#
# Testing generic class wraps_cls decorator
#


# Just in case we run this directly
if __name__ == '__main__':
    nose.runmodule(__name__)
