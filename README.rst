|Build Status|  |Code Health|  |Pyup Updates|  |Pyup Py3|

Pyros-schemas
=============

Package implementing serialization for Pyros multiprocess systems.

Features
--------

ROS
~~~

-  serializes everything as a dict, flatten base field types if possible.


.. |Build Status| image:: https://travis-ci.org/pyros-dev/pyros-schemas.svg?branch=master
                  :target: https://travis-ci.org/pyros-dev/pyros-schemas
                  :alt: Build Status

.. |Code Health| image:: https://landscape.io/github/pyros-dev/pyros-schemas/master/landscape.svg?style=flat
                 :target: https://landscape.io/github/pyros-dev/pyros-schemas/master
                 :alt: Code Health

.. |Pyup Updates| image:: https://pyup.io/repos/github/pyros-dev/pyros-schemas/shield.svg
                  :target: https://pyup.io/repos/github/pyros-dev/pyros-schemas/
                  :alt: Updates

.. |Pyup Py3| image:: https://pyup.io/repos/github/pyros-dev/pyros-schemas/python-3-shield.svg
              :target: https://pyup.io/repos/github/pyros-dev/pyros-schemas/
              :alt: Python 3

Testing
-------

1) make sure you have downloaded the submodules (ros message definitions)
2) check `tox -l` to list the test environments
3) choose the tests matching your platform and run them

The tests are also run on travis, so any pull request need to have tests failing at first ( create test to illustrate the problem if needed).
Then add commits to fix the broken tests, and all must pass before merging.
