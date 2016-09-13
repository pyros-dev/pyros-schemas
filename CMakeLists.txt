##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(pyros_schemas)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin REQUIRED COMPONENTS
    roslint
    std_msgs
)


catkin_python_setup()


# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} pyros_schemas/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})



##############################################################################
# Catkin
##############################################################################

catkin_package()

#########
# Tests
#######

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(pyros_schemas/tests)
endif()


