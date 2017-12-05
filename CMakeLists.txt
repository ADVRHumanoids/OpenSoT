# Copyright: (C) 2014 Walkman Consortium
# Authors: Enrico Mingo Hoffman, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(open_sot)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

option(SET_SSE4_FLAG "set -msse4 flag to gcc" OFF)

if(${SET_SSE4_FLAG})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type -Werror=address -Werror=parentheses " )
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(YCM REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(srdfdom_advr REQUIRED)
FIND_PACKAGE(moveit_core QUIET)
FIND_PACKAGE(PCL 1.7 QUIET COMPONENTS    common
                                            filters
                                            surface
                                            search
                                            io)
FIND_PACKAGE(catkin REQUIRED)
FIND_PACKAGE(eigen_conversions REQUIRED)
FIND_PACKAGE(XBotInterface REQUIRED)
FIND_PACKAGE(fcl QUIET)

# compilation flags
option(OPENSOT_COMPILE_EXAMPLES "Compile OpenSoT examples" TRUE)
option(OPENSOT_COMPILE_TESTS "Compile OpenSoT tests" FALSE)

# add include directories
INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIR}
                            ${PCL_INCLUDE_DIRS}
                            ${XBotInterface_INCLUDE_DIRS}
                            ${srdfdom_advr_INCLUDE_DIRS}
                            )

ADD_DEFINITIONS(${PCL_DEFINITIONS})

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

include_directories("${qpOASES_INCLUDE_DIRS}")

# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)
file(GLOB_RECURSE sot_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/python" *.py)

##CONSTRAINTS
set(OPENSOT_CONSTRAINTS_SOURCES src/constraints/Aggregated.cpp
                    src/constraints/BilateralConstraint.cpp
                    src/constraints/TaskToConstraint.cpp
                    src/constraints/acceleration/DynamicFeasibility.cpp
                    src/constraints/velocity/CartesianPositionConstraint.cpp
                    src/constraints/velocity/CapturePoint.cpp
                    src/constraints/velocity/CartesianVelocity.cpp
                    src/constraints/velocity/CoMVelocity.cpp
                    src/constraints/velocity/JointLimits.cpp
                    src/constraints/velocity/VelocityLimits.cpp
                    src/constraints/torque/TorqueLimits.cpp
                    src/constraints/torque/JointLimits.cpp
                    src/constraints/force/FrictionCone.cpp
                    src/constraints/force/WrenchLimits.cpp
                    src/constraints/GenericConstraint.cpp
                    )
if(${PCL_FOUND} AND ${moveit_core_FOUND})
    set(OPENSOT_CONSTRAINTS_SOURCES ${OPENSOT_CONSTRAINTS_SOURCES}
        src/constraints/velocity/ConvexHull.cpp)
endif()
if(${fcl_FOUND} AND ${moveit_core_FOUND})
    set(OPENSOT_CONSTRAINTS_SOURCES ${OPENSOT_CONSTRAINTS_SOURCES}
        src/constraints/velocity/SelfCollisionAvoidance.cpp)
endif()

##TASKS
set(OPENSOT_TASKS_SOURCES src/tasks/Aggregated.cpp
                    src/tasks/SubTask.cpp
                    src/tasks/MinimizeVariable.cpp
                    src/tasks/acceleration/Postural.cpp
                    src/tasks/acceleration/Cartesian.cpp
                    src/tasks/acceleration/Contact.cpp
                    src/tasks/velocity/Cartesian.cpp
                    src/tasks/velocity/PureRolling.cpp
                    src/tasks/velocity/RigidRotation.cpp
                    src/tasks/velocity/Gaze.cpp
                    src/tasks/velocity/Contact.cpp
                    src/tasks/velocity/CoM.cpp
                    src/tasks/velocity/AngularMomentum.cpp
                    src/tasks/velocity/LinearMomentum.cpp                    
                    src/tasks/velocity/Manipulability.cpp
                    src/tasks/velocity/MinimizeAcceleration.cpp
                    src/tasks/velocity/MinimumEffort.cpp
                    src/tasks/velocity/MinimumVelocity.cpp
                    src/tasks/velocity/Postural.cpp
                    src/tasks/torque/JointImpedanceCtrl.cpp
                    src/tasks/torque/CartesianImpedanceCtrl.cpp
                    src/tasks/force/CoM.cpp
                    src/tasks/force/Wrench.cpp
                    )
##SOLVERS
set(OPENSOT_SOLVERS_SOURCES src/solvers/QPOases.cpp
                    src/solvers/QPOasesProblem.cpp
                    src/solvers/DampedPseudoInverse.cpp)
##UTILS
set(OPENSOT_UTILS_SOURCES src/utils/AutoStack.cpp
                    src/utils/DefaultHumanoidStack.cpp
                    src/utils/Affine.cpp
                    src/utils/Indices.cpp
                    src/utils/VelocityAllocation.cpp
                    src/utils/cartesian_utils.cpp)
if(${moveit_core_FOUND})
    if(${fcl_FOUND})
        set(OPENSOT_UTILS_SOURCES ${OPENSOT_UTILS_SOURCES}
            src/utils/collision_utils.cpp)
    endif()
endif()
if(${PCL_FOUND})
    set(OPENSOT_UTILS_SOURCES ${OPENSOT_UTILS_SOURCES}
        src/utils/convex_hull_utils.cpp)
endif()

##VARIABLES
set(OPENSOT_VARIABLES_SOURCES src/variables/Torque.cpp)

ADD_LIBRARY(OpenSoT SHARED
                    ${OPENSOT_CONSTRAINTS_SOURCES}
                    ${OPENSOT_TASKS_SOURCES}
                    ${OPENSOT_SOLVERS_SOURCES}
                    ${OPENSOT_UTILS_SOURCES}
                    ${OPENSOT_VARIABLES_SOURCES}
#                    src/constraints/velocity/Dynamics.cpp
#                    src/stacks/velocity/ManipulationStack.cpp
#                    src/stacks/velocity/WalkingStack.cpp
#                    src/tasks/velocity/Interaction.cpp
                    ${sot_INCLUDES})

# we must wait for qpOASES-ext to be ready before building OpenSoT
if(TARGET qpOASES-ext)
    add_dependencies(OpenSoT qpOASES-ext)
endif()



TARGET_LINK_LIBRARIES(OpenSoT PUBLIC
                              ${srdfdom_advr_LIBRARIES}
                              ${eigen_conversions_LIBRARIES}
                              PRIVATE
                              ${qpOASES_LIBRARIES}
                              ${PCL_LIBRARIES}
                              ${XBotInterface_LIBRARIES}
                              ${fcl_LIBRARIES}
                              ${moveit_core_LIBRARIES}
                              )

########################################################################
# use YCM to export OpenSoT so taht it can be found using find_package #
########################################################################

set(VARS_PREFIX "OPENSOT")

set(OPENSOT_MAJOR_VERSION 0)
set(OPENSOT_MINOR_VERSION 2)
set(OPENSOT_PATCH_VERSION 0)
set(OPENSOT_VERSION ${OPENSOT_MAJOR_VERSION}.${OPENSOT_MINOR_VERSION}.${OPENSOT_PATCH_VERSION})


include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(OpenSoT PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                          "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(OpenSoT PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                              SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
    DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
    FILES_MATCHING PATTERN "*.h")

if(NOT qpOASES_FOUND)
    install(DIRECTORY "${qpOASES_INCLUDE_DIRS}"
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/..")
endif()

install(TARGETS OpenSoT
        EXPORT OpenSoT
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES idyn_ros_interface
    CATKIN_DEPENDS roscpp rospy roslib srdfdom
    DEPENDS system_lib)

install(TARGETS OpenSoT
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#enabling it will add all OpenSoT dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS OpenSoT)


include(InstallBasicPackageFiles)
install_basic_package_files(OpenSoT VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)


add_subdirectory(doc)

########################
# Add Examples target  #
########################
#if(OPENSOT_COMPILE_EXAMPLES)
#    add_subdirectory(examples)
#endif()

#######################
# Add Testing target  #
#######################
if(OPENSOT_COMPILE_TESTS)
  enable_testing()
    add_subdirectory(tests)
endif()

if(OPENSOT_COMPILE_TESTS OR OPENSOT_COMPILE_EXAMPLES)
    add_custom_target(copy_robot_model_files ALL
                      ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/tests/robots" "${CMAKE_CURRENT_BINARY_DIR}/tests/robots")
endif()
