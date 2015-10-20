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

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type -Werror=address -Werror=parentheses " )
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(Boost QUIET)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(idynutils REQUIRED)
FIND_PACKAGE(roslib REQUIRED)
FIND_PACKAGE(rospack REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
FIND_PACKAGE(PythonLibs)
FIND_PACKAGE(PCL 1.7 REQUIRED COMPONENTS filters surface)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(interactive_markers REQUIRED)
FIND_PACKAGE(catkin REQUIRED)
FIND_PACKAGE(eigen_conversions REQUIRED)

# compilation flags
option(OPENSOT_COMPILE_EXAMPLES "Compile OpenSoT examples" TRUE)
option(OPENSOT_COMPILE_TESTS "Compile OpenSoT tests" FALSE)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS}
                            ${paramHelp_INCLUDE_DIRS} ${eigen_conversions_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${tf_INCLUDE_DIRS}
                            ${interactive_markers_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

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

ADD_LIBRARY(OpenSoT SHARED
                    src/constraints/Aggregated.cpp
                    src/constraints/BilateralConstraint.cpp
                    src/constraints/TaskToConstraint.cpp
                    src/constraints/velocity/CartesianPositionConstraint.cpp
                    src/constraints/velocity/CoMVelocity.cpp
                    src/constraints/velocity/ConvexHull.cpp
                    src/constraints/velocity/JointLimits.cpp
                    src/constraints/velocity/VelocityLimits.cpp
                    src/constraints/velocity/SelfCollisionAvoidance.cpp
                    src/constraints/velocity/Dynamics.cpp
                    src/constraints/force/FrictionCone.cpp
                    src/interfaces/yarp/tasks/YCartesian.cpp
                    src/interfaces/yarp/tasks/YCoM.cpp
                    src/interfaces/yarp/tasks/YPostural.cpp
                    src/solvers/QPOases.cpp
                    src/solvers/QPOasesProblem.cpp
                    src/tasks/Aggregated.cpp
                    src/tasks/SubTask.cpp
                    src/tasks/velocity/Cartesian.cpp
                    src/tasks/velocity/CoM.cpp
                    src/tasks/velocity/Interaction.cpp
                    src/tasks/velocity/Manipulability.cpp
                    src/tasks/velocity/MinimizeAcceleration.cpp
                    src/tasks/velocity/MinimumEffort.cpp
                    src/tasks/velocity/MinimumVelocity.cpp
                    src/tasks/velocity/Postural.cpp
                    src/tasks/force/CoM.cpp
                    src/utils/AutoStack.cpp
                    src/utils/DefaultHumanoidStack.cpp
                    src/utils/PreviewerUtils.cpp
                    src/utils/VelocityAllocation.cpp
                    ${sot_INCLUDES})

# we must wait for qpOASES-ext to be ready before building OpenSoT
if(TARGET qpOASES-ext)
    add_dependencies(OpenSoT qpOASES-ext)
endif()


# we now add the YARP libraries to our project.
TARGET_LINK_LIBRARIES(OpenSoT PUBLIC  ${idynutils_LIBRARIES} ${iDynTree_LIBRARIES} ${eigen_conversions_LIBRARIES}
                              PRIVATE ${qpOASES_LIBRARIES} ${PCL_LIBRARIES} ${tf_LIBRARIES} ${interactive_markers_LIBRARIES} ${PYTHON_LIBRARIES})

if(Boost_VERSION VERSION_GREATER 104701)
    ADD_EXECUTABLE(ros_yarp_Cartesian_interactive_marker src/interfaces/ros/tasks/ros_yarp_Cartesian_interactive_marker.cpp)
    ADD_EXECUTABLE(ros_yarp_gaze_interactive_marker src/interfaces/ros/tasks/ros_yarp_gaze_interactive_marker.cpp)
    ADD_EXECUTABLE(ros_yarp_postural_interactive_marker src/interfaces/ros/tasks/ros_yarp_postural_interactive_marker.cpp)

    TARGET_LINK_LIBRARIES(ros_yarp_Cartesian_interactive_marker PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${iDynTree_LIBRARIES} ${catkin_LIBRARIES}
                                  PRIVATE ${qpOASES_LIBRARIES} ${PCL_LIBRARIES} ${tf_LIBRARIES} ${interactive_markers_LIBRARIES})
    add_dependencies(ros_yarp_Cartesian_interactive_marker OpenSoT)

    TARGET_LINK_LIBRARIES(ros_yarp_gaze_interactive_marker PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${iDynTree_LIBRARIES} ${catkin_LIBRARIES}
                                  PRIVATE ${qpOASES_LIBRARIES} ${PCL_LIBRARIES} ${tf_LIBRARIES} ${interactive_markers_LIBRARIES})
    add_dependencies(ros_yarp_gaze_interactive_marker OpenSoT)

    TARGET_LINK_LIBRARIES(ros_yarp_postural_interactive_marker PUBLIC  OpenSoT ${idynutils_LIBRARIES} ${iDynTree_LIBRARIES} ${catkin_LIBRARIES}
                                  PRIVATE ${qpOASES_LIBRARIES} ${PCL_LIBRARIES} ${tf_LIBRARIES} ${interactive_markers_LIBRARIES})
    add_dependencies(ros_yarp_postural_interactive_marker OpenSoT)
endif()



########################################################################
# use YCM to export OpenSoT so taht it can be found using find_package #
########################################################################

set(VARS_PREFIX "OPENSOT")

set(OPENSOT_MAJOR_VERSION 0)
set(OPENSOT_MINOR_VERSION 2)
set(OPENSOT_PATCH_VERSION 0)
set(OPENSOT_VERSION ${OPENSOT_MAJOR_VERSION}.${OPENSOT_MINOR_VERSION}.${OPENSOT_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(OpenSoT PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                          "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(OpenSoT PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                              SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/ 
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h")

install(TARGETS OpenSoT
        EXPORT OpenSoT
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES idyn_ros_interface
 CATKIN_DEPENDS roscpp rospy roslib srdfdom
 DEPENDS system_lib
)

if(Boost_VERSION VERSION_GREATER 104701)
    message(STATUS "Compiling ros_yarp interactive markers for OpenSoT")

    install(TARGETS OpenSoT ros_yarp_Cartesian_interactive_marker ros_yarp_gaze_interactive_marker
    ros_yarp_postural_interactive_marker
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
else()
    install(TARGETS OpenSoT
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()

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

add_subdirectory(app)
add_subdirectory(doc)

########################
# Add Examples target  #
########################
if(OPENSOT_COMPILE_EXAMPLES)
  add_subdirectory(examples)
endif()

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
