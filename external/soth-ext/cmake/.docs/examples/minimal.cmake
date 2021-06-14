CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(cmake/base.cmake)

# These variables have to be defined before running SETUP_PROJECT
SET(PROJECT_NAME jrl-cmakemodules-minimal-working-example)
SET(PROJECT_DESCRIPTION "A project description")
SET(PROJECT_URL http://jrl-cmakemodules.readthedocs.io)

SETUP_PROJECT()

# Configure the build of your project here
# ADD_SUBDIRECTORY(src)

# This should be called at the end
SETUP_PROJECT_FINALIZE()
