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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(idynutils REQUIRED)
FIND_PACKAGE(Klampt QUIET)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(YARP REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(. include ${YARP_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS})

include_directories("${qpOASES_INCLUDE_DIRS}")

ADD_EXECUTABLE(klampt_controller main.cpp)

ADD_LIBRARY(example_klampt_controller SHARED
                                      example_klampt_controller.cpp
                                      example_klampt_controller.h
                                      src/KlamptController.cpp
                                      include/KlamptController.h
                                      include/utils.h)

ADD_DEPENDENCIES(example_klampt_controller OpenSoT)
ADD_DEPENDENCIES(klampt_controller example_klampt_controller)

TARGET_LINK_LIBRARIES(example_klampt_controller PUBLIC OpenSoT ${idynutils_LIBRARIES} ${Boost_LIBRARIES})

TARGET_LINK_LIBRARIES(klampt_controller         PUBLIC example_klampt_controller)

install(TARGETS example_klampt_controller DESTINATION lib)

find_package(SWIG)
if(SWIG_FOUND)
    include(UseSWIG)

    set_source_files_properties(KlamptController.i PROPERTIES CPLUSPLUS ON)

    include_directories(. include)
    link_libraries(example_klampt_controller)

    set(CMAKE_SWIG_FLAGS "-Wall;-module;ExampleKlamptController;-threads")

        set(OPENSOT_USES_PYTHON_VERSION "" CACHE STRING "Specify python version to use" )

        find_package(PythonInterp ${OPENSOT_USES_PYTHON_VERSION} REQUIRED)
        set(OPENSOT_USES_PYTHON_VERSION_ARGS)
        if(NOT OPENSOT_USES_PYTHON_VERSION)
          set (OPENSOT_USES_PYTHON_VERSION ${PYTHON_VERSION_STRING})
        endif()

        find_package(PythonLibs ${OPENSOT_USES_PYTHON_VERSION} EXACT)
        include_directories(${PYTHON_INCLUDE_PATH})
        link_libraries(${PYTHON_LIBRARY})

        set(target_name ExampleKlamptController)

        swig_add_module(${target_name} python KlamptController.i)
        set_target_properties(_${target_name} PROPERTIES OUTPUT_NAME "_ExampleKlamptController")
        set_target_properties(_${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python)

        # installation path is determined reliably on most platforms using distutils
        execute_process(COMMAND ${PYTHON_EXECUTABLE}
            -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
        OUTPUT_VARIABLE PYTHON_INSTDIR OUTPUT_STRIP_TRAILING_WHITESPACE )
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ExampleKlamptController.py DESTINATION ${PYTHON_INSTDIR} )
        install(TARGETS _${target_name} DESTINATION ${PYTHON_INSTDIR} )

endif(SWIG_FOUND)
