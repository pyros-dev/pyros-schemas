Changelog
=========


0.1.0 (2018-02-17)
------------------
- Merge pull request #5 from pyros-dev/nested_merged. [AlexV]

  Nested merged
- Changing pyup update schedule. [AlexV]
- Removed duplicated version module. [AlexV]
- Incrementing version number, preparing for python release. [AlexV]
- Merge pull request #12 from pyros-dev/refactoring_tests. [AlexV]

  Refactoring tests
- Disabling currently failing py3 tests and adding comment bout it.
  [AlexV]
- Removing pyros-msgs repo from requirements. [AlexV]
- Restructuring tests to make data generation faster. [AlexV]
- Removing old pyros-msgs branch in requirements. [AlexV]
- Adding README badges following pyros-msgs example. [AlexV]
- Fixing excluded envs for travis. [AlexV]
- Fixing tox params for travis, adding pyup settings. [AlexV]
- Attempting tox params depending on ros distro. [AlexV]
- Improving tests and CI. [AlexV]
- Fixing python3 iteritems. [AlexV]
- Fixing hypothesis options for travis test profile. [AlexV]
- Fixing travis link in README. [AlexV]
- Fixing CMakeLists for tests. [AlexV]
- Added README comments about CI, tests and PR merges. [AlexV]
- Merge pull request #6 from asmodehn/pymutate. [AlexV]

  adding files for python package structure.
- Registering hypothesis profile earlier, at import time. [alexv]
- Setting hypothesis profile in function to not break at import time.
  [alexv]
- Moving hypothesis profiles into pytest conftest. [alexv]
- Cleanup settings and now using hypothesis profiles moved std_msgs
  tests. [alexv]
- No need for pyros-setup in tests now, we rely on venv only. [alexv]
- Moving tests outside of package and fixing tox. [alexv]
- Fixing test environments with travis and tox. [alexv]
- Adding files for python package structure. not moving tests yet.
  [alexv]
- Making basic and opt_as_array tests pass. [alexv]
- Handle case where we get a list to change into a ROSmsg... [AlexV]
- Merge branch 'hypothesis' into nested_merged. [AlexV]

  Conflicts:
  	CMakeLists.txt
  	package.xml
  	pyros_schemas/ros/decorators.py
  	pyros_schemas/ros/tests/test_basic_fields.py
  	pyros_schemas/ros/tests/test_decorators.py
  	pyros_schemas/ros/tests/test_optional_as_array_fields.py
  	setup.py
  	travis_checks.bash
- Fixing travis to use ros-shadow-fixed repos. [AlexV]
- Fixing dependencies and travis install test check. [AlexV]
- Moved hypothesis tests inside package. [AlexV]
- Fixed hypothesis tests. [AlexV]
- WIP adding tests for optional fields. [AlexV]
- Fixed some time tests, added hypothesis requirement. [AlexV]
- Added hypothesis test for std schemas. removed unused time fields
  module. [AlexV]
- Fixing test_schema. [AlexV]
- Adding first hypothesis tests. fixed time and duration field classes.
  [AlexV]
- Merge branch 'master' into hypothesis. [AlexV]
- Cleaning up test for time messages. [AlexV]
- Fixed install. improved tests by moving to pytest. [AlexV]
- Adding dependency on pyros_msgs. [alexv]
- Fixing travis install tests. [alexv]
- Removing roslint build requirement. [alexv]
- Adding catkin_pip dependency. [alexv]
- Adding nested test messages. fixing existing tests. [alexv]
- Migrated to catkin_pip + pytests. added basic structure for JiT
  message generation for tests. [alexv]
- Merge pull request #4 from naveedhd/temp_setup_install. [AlexV]

  temporary commit to install pyros_schemas/ros to install space
- Temporary commit to install pyros_schemas/ros to install space.
  [Naveed Usmani]

  this will be integrated with prepare_release branch
- Merge pull request #1 from asmodehn/http. [AlexV]

  Http
- Fixing imports. [alexv]
- Small module restructure to fix circular import. [alexv]
- Adding python-six dependency. [alexv]
- Adding todo list... [alexv]
- Adding decorators to parse/unparse ROS service messages to/from dict.
  [alexv]
- Now all httpbin tests are passing. [alexv]
- Getting nested and list fields to work. [alexv]
- Small fixes for httpbin tests. [alexv]
- Small refactoring to make ros schemas simpler to manage. [alexv]
- WIP small refactor, Rostime schema tested. [alexv]
- Removed unused pyros_msgs subpackage. [alexv]
- Adding roslist. fixed import statements. [alexv]
- Implemented list for optional field. tests for optional string
  passing. [alexv]
- Refactored. now unit testing all basic fields for ROS field types.
  [alexv]
- Adding sphinx documentation. [alexv]
- Fixes for opt_string. [alexv]
- Improved optional string field. [alexv]
- WIP. Commit before changing internal dict representation of optional
  message. [alexv]
- Fixing tests to import properly. cosmetics. [alexv]
- WIP making http statuscode work... [alexv]
- Defining schema for http status code. [alexv]
- First version of pyros_schemas, extracted from pyros_msgs. [alexv]
- Initial commit. [AlexV]


