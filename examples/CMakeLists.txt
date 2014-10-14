cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME examples)
project(${PROJECTNAME})

# Copyright: (C) 2014 Walkman Consortium
# Authors: Enrico Mingo, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(OpenSoT)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(drc_shared REQUIRED)
FIND_PACKAGE(PCL 1.2 REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

ADD_DEFINITIONS(${PCL_DEFINITIONS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

include_directories("${qpOASES_INCLUDE_DIRS}")

ADD_EXECUTABLE(example_cartesian example_cartesian.cpp)
ADD_EXECUTABLE(example_com example_com.cpp)
ADD_EXECUTABLE(example_mininum_effort example_minimum_effort.cpp)
ADD_EXECUTABLE(example_postural example_postural.cpp)

TARGET_LINK_LIBRARIES(example_cartesian PUBLIC  OpenSoT ${drc_shared_LIBRARIES})
TARGET_LINK_LIBRARIES(example_com PUBLIC  OpenSoT ${drc_shared_LIBRARIES} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(example_postural PUBLIC  OpenSoT ${drc_shared_LIBRARIES})
