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

FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(idynutils REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED COMPONENTS filters surface)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(YARP REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

ADD_DEFINITIONS(${PCL_DEFINITIONS})

add_definitions(-DOPENSOT_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/../tests/robots/")

include_directories("${qpOASES_INCLUDE_DIRS}")

ADD_EXECUTABLE(example_autostack example_autostack.cpp)
ADD_EXECUTABLE(example_cartesian example_cartesian.cpp)
ADD_EXECUTABLE(example_com example_com.cpp)
ADD_EXECUTABLE(example_minimum_effort example_minimum_effort.cpp)
ADD_EXECUTABLE(example_postural example_postural.cpp)
ADD_EXECUTABLE(example_python example_python.cpp)
ADD_EXECUTABLE(example_python_crouching example_python_crouching.cpp)
ADD_EXECUTABLE(example_python_distance_query example_python_distance_query.cpp)
ADD_EXECUTABLE(example_previewer example_previewer.cpp)
ADD_EXECUTABLE(example_velocity_allocation example_velocity_allocation.cpp)

ADD_DEPENDENCIES(example_autostack OpenSoT)
ADD_DEPENDENCIES(example_cartesian OpenSoT)
ADD_DEPENDENCIES(example_com OpenSoT)
ADD_DEPENDENCIES(example_minimum_effort OpenSoT)
ADD_DEPENDENCIES(example_postural OpenSoT)
ADD_DEPENDENCIES(example_python OpenSoT)
ADD_DEPENDENCIES(example_python_crouching OpenSoT)
ADD_DEPENDENCIES(example_python_distance_query OpenSoT)
ADD_DEPENDENCIES(example_previewer OpenSoT)
ADD_DEPENDENCIES(example_velocity_allocation OpenSoT)

TARGET_LINK_LIBRARIES(example_autostack PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(example_cartesian PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(example_com PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(example_minimum_effort PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(example_postural PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(example_python PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(example_python_crouching PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(example_python_distance_query ${iDynTree_LIBRARIES} ${idynutils_LIBRARIES} 
                                                    ${Boost_LIBRARIES} ${tf_LIBRARIES})
TARGET_LINK_LIBRARIES(example_previewer PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(example_velocity_allocation PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})
