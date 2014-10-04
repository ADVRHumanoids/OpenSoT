# Copyright: (C) 2014 Walkman Consortium
# Authors: Enrico Mingo, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
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
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(kdl_codyco REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(drc_shared REQUIRED)
FIND_PACKAGE(roslib REQUIRED)
FIND_PACKAGE(rospack REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
FIND_PACKAGE(PCL 1.2 REQUIRED)
FIND_PACKAGE(tf REQUIRED)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${paramHelp_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

ADD_DEFINITIONS(${PCL_DEFINITIONS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

# Find package qpOASES or build it using ExternalProject
find_package(qpOASES QUIET)
if(NOT qpOASES_FOUND)
    set(qpOASES_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/qpOASES-ext/")
    set(qpOASES_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external/src/qpOASES-ext-build/")

    ExternalProject_Add(qpOASES-ext SOURCE_DIR "${qpOASES_SOURCE_DIR}"
                                    PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                    INSTALL_COMMAND ""
                                    CMAKE_ARGS -DCMAKE_CXX_FLAGS:STRING="-fPIC")
    link_directories("${qpOASES_BINARY_DIR}/libs/")
    set(qpOASES_INCLUDE_DIRS "${qpOASES_SOURCE_DIR}/include")
    set(qpOASES_LIBRARIES qpOASES)
endif()

SET(qpOASES_LIBRARIES_WHOLE_ARCHIVE -Wl,--whole-archive ${qpOASES_LIBRARIES} -Wl,--no-whole-archive)

include_directories("${qpOASES_INCLUDE_DIRS}")

# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)
file(GLOB_RECURSE sot_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/python" *.py)

ADD_LIBRARY(OpenSoT SHARED
                    src/constraints/Aggregated.cpp
                    src/constraints/BilateralConstraint.cpp
                    src/constraints/velocity/CoMVelocity.cpp
                    src/constraints/velocity/ConvexHull.cpp
                    src/constraints/velocity/JointLimits.cpp
                    src/constraints/velocity/VelocityLimits.cpp
                    src/solvers/QPOases.cpp
                    src/tasks/Aggregated.cpp
                    src/tasks/velocity/Cartesian.cpp
                    src/tasks/velocity/CoM.cpp
                    src/tasks/velocity/MinimumEffort.cpp
                    src/tasks/velocity/Postural.cpp
                    ${sot_INCLUDES})


# we must wait for qpOASES-ext to be ready before building OpenSoT
if(TARGET qpOASES-ext)
    add_dependencies(OpenSoT qpOASES-ext)
endif()

# ADD_EXECUTABLE(uTest1 src/sot_VelKinCon_ctrl.cpp src/yarp_interface.cpp src/tests/unitTest1.cpp src/task_solver2.cpp)
# ADD_EXECUTABLE(uTest2 src/tests/unitTest2.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(OpenSoT ${drc_shared_LIBRARIES} ${iDynTree_LIBRARIES} ${PCL_LIBRARIES}
                              ${qpOASES_LIBRARIES_WHOLE_ARCHIVE})

#TARGET_LINK_LIBRARIES(uTest1  wb_sot ${drc_shared_LIBRARIES} ${moveit_core_LIBRARIES} ${srdfdom_LIBRARIES} ${rospack_LIBRARIES} ${rosconsole_LIBRARIES} ${roslib_LIBRARIES}
#                              ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${paramHelp_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdf_LIBRARIES} ${kdl_parser_LIBRARIES} ${PCL_LIBRARIES}
#                      qpOASES ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
#TARGET_LINK_LIBRARIES(uTest2 ${drc_shared_LIBRARIES} ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdf_LIBRARIES} ${kdl_parser_LIBRARIES} ${PCL_LIBRARIES}
#                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})

#add_custom_target(  copy_conf_files_to_build ALL
#    COMMAND ${CMAKE_COMMAND} -E copy_directory
#        "${PROJECT_SOURCE_DIR}/app/conf"
#        "${CMAKE_BINARY_DIR}/")

#add_custom_target( copy_python_to_build ALL
#    COMMAND ${CMAKE_COMMAND} -E copy_directory
#        "${CMAKE_CURRENT_SOURCE_DIR}/python"
#        "${CMAKE_BINARY_DIR}/")

#add_custom_target(generate_paramsTuner_gui ALL
#    COMMAND "${CMAKE_BINARY_DIR}/createParamTuner" > "${CMAKE_CURRENT_SOURCE_DIR}/app/resources/tuner.xml")
#add_dependencies(generate_paramsTuner_gui createParamTuner)


########################################################################
# use YCM to export OpenSoT so taht it can be found using find_package #
########################################################################

set(VARS_PREFIX "OPENSOT")

set(OPENSOT_MAJOR_VERSION 0)
set(OPENSOT_MINOR_VERSION 0)
set(OPENSOT_PATCH_VERSION 1)
set(OPENSOT_VERSION ${OPENSOT_MAJOR_VERSION}.${OPENSOT_MINOR_VERSION}.${OPENSOT_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(OpenSoT PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                          "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(OpenSoT PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                              SOVERSION ${${VARS_PREFIX}_VERSION}
                              PUBLIC_HEADER "${sot_INCLUDES}")
install(DIRECTORY include/ 
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h")

install(TARGETS OpenSoT  
        EXPORT opensot
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)
        
#enabling it will add all OpenSoT dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS OpenSoT)


include(InstallBasicPackageFiles)
install_basic_package_files(OpenSoT VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

add_subdirectory(app)
add_subdirectory(doc)

#######################
# Add Testing target  #
#######################
enable_testing()
add_subdirectory(tests)
