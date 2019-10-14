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
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 nor C++14 support. Please use a different C++ compiler.")
endif()

option(SET_SSE4_FLAG "set -msse4 flag to gcc" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

# Set a default build type for single-configuration
# CMake generators if no build type is set.
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)


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



FIND_PACKAGE(FCL 0.6 QUIET)



FIND_PACKAGE(PkgConfig REQUIRED)

# compilation flags
option(OPENSOT_COMPILE_EXAMPLES "Compile OpenSoT examples" TRUE)
option(OPENSOT_COMPILE_TESTS "Compile OpenSoT tests" FALSE)
option(OPENSOT_VERBOSE "Some additional prints" FALSE)
option(OPENSOT_DISABLE_VECTORIZATION "Disable Eigen3 vectorization" FALSE)

if(${OPENSOT_DISABLE_VECTORIZATION})
  add_definitions(-DEIGEN_DONT_VECTORIZE)
  add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
endif()
 
 
if(${OPENSOT_VERBOSE})
    add_definitions(-DOPENSOT_VERBOSE)
endif()

# add include directories
INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIR}
                            ${PCL_INCLUDE_DIRS}
                            ${XBotInterface_INCLUDE_DIRS}
                            ${srdfdom_advr_INCLUDE_DIRS}
                            )

ADD_DEFINITIONS(${PCL_DEFINITIONS})


# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

##CONSTRAINTS
set(OPENSOT_CONSTRAINTS_SOURCES src/constraints/Aggregated.cpp
                    src/constraints/BilateralConstraint.cpp
                    src/constraints/TaskToConstraint.cpp
                    src/constraints/acceleration/DynamicFeasibility.cpp
                    src/constraints/acceleration/VelocityLimits.cpp
                    src/constraints/velocity/CartesianPositionConstraint.cpp
                    src/constraints/velocity/CapturePoint.cpp
                    src/constraints/velocity/CartesianVelocity.cpp
                    src/constraints/velocity/CoMVelocity.cpp
                    src/constraints/velocity/JointLimits.cpp
                    src/constraints/velocity/affine/JointLimits.cpp
                    src/constraints/velocity/VelocityLimits.cpp
                    src/constraints/velocity/affine/VelocityLimits.cpp
                    src/constraints/torque/TorqueLimits.cpp
                    src/constraints/torque/JointLimits.cpp
                    src/constraints/force/FrictionCone.cpp
                    src/constraints/force/WrenchLimits.cpp
                    src/constraints/force/CoP.cpp
                    src/constraints/force/StaticConstraint.cpp
                    src/constraints/GenericConstraint.cpp
                    src/constraints/acceleration/JointLimits.cpp
                    )

if(${PCL_FOUND} AND ${moveit_core_FOUND})
    message("Adding src/constraints/velocity/ConvexHull.cpp to compilation")
    set(OPENSOT_CONSTRAINTS_SOURCES ${OPENSOT_CONSTRAINTS_SOURCES}
        src/constraints/velocity/ConvexHull.cpp)
endif()

if(${FCL_FOUND} AND ${moveit_core_FOUND})
    message("Adding src/constraints/velocity/SelfCollisionAvoidance.cpp to compilation")
    message("Adding src/constraints/velocity/CollisionAvoidance.cpp to compilation")
    set(OPENSOT_CONSTRAINTS_SOURCES ${OPENSOT_CONSTRAINTS_SOURCES}
        src/constraints/velocity/SelfCollisionAvoidance.cpp
        src/constraints/velocity/CollisionAvoidance.cpp
        )
endif()

##TASKS
set(OPENSOT_TASKS_SOURCES src/tasks/Aggregated.cpp
                    src/tasks/SubTask.cpp
                    src/tasks/MinimizeVariable.cpp
                    src/tasks/GenericTask.cpp
                    src/tasks/GenericLPTask.cpp
                    src/tasks/acceleration/Postural.cpp
                    src/tasks/acceleration/Cartesian.cpp
                    src/tasks/acceleration/Contact.cpp
                    src/tasks/acceleration/CoM.cpp
                    src/tasks/acceleration/MinJointVel.cpp
                    src/tasks/velocity/Cartesian.cpp
                    src/tasks/velocity/affine/Cartesian.cpp
                    src/tasks/velocity/CartesianAdmittance.cpp
                    src/tasks/velocity/PureRolling.cpp
                    src/tasks/velocity/RigidRotation.cpp
                    src/tasks/velocity/Unicycle.cpp
                    src/tasks/velocity/Gaze.cpp
                    src/tasks/velocity/Contact.cpp
                    src/tasks/velocity/CoM.cpp
                    src/tasks/velocity/affine/CoM.cpp
                    src/tasks/velocity/AngularMomentum.cpp
                    src/tasks/velocity/LinearMomentum.cpp                    
                    src/tasks/velocity/Manipulability.cpp
                    src/tasks/velocity/MinimizeAcceleration.cpp
                    src/tasks/velocity/MinimumEffort.cpp
                    src/tasks/velocity/MinimumVelocity.cpp
                    src/tasks/velocity/Postural.cpp
                    src/tasks/velocity/JointAdmittance.cpp
                    src/tasks/velocity/CentauroAnkleSteering.cpp
                    src/tasks/torque/JointImpedanceCtrl.cpp
                    src/tasks/torque/CartesianImpedanceCtrl.cpp
                    src/tasks/force/CoM.cpp
                    src/tasks/force/FloatingBase.cpp
                    src/tasks/force/Force.cpp
                    src/tasks/force/Cartesian.cpp
                    src/tasks/floating_base/Contact.cpp
                    src/tasks/floating_base/IMU.cpp
                    )
##SOLVERS
set(OPENSOT_SOLVERS_SOURCES src/solvers/BackEnd.cpp
                            src/solvers/BackEndFactory.cpp
                            src/solvers/iHQP.cpp
                            src/solvers/nHQP.cpp
                            src/solvers/eHQP.cpp)

##UTILS
set(OPENSOT_UTILS_SOURCES src/utils/AutoStack.cpp
                    src/utils/DefaultHumanoidStack.cpp
                    src/utils/Affine.cpp
                    src/utils/Indices.cpp
                    src/utils/VelocityAllocation.cpp
                    src/utils/cartesian_utils.cpp 
                    src/utils/ForceOptimization.cpp
                    src/utils/InverseDynamics.cpp)

if(${FCL_FOUND} AND ${moveit_core_FOUND})
        message("Adding src/utils/collision_utils.cpp to compilation")
        set(OPENSOT_UTILS_SOURCES ${OPENSOT_UTILS_SOURCES}
            src/utils/collision_utils.cpp)
endif()

if(${PCL_FOUND})
    message("Adding src/utils/convex_hull_utils.cpp to compilation")
    set(OPENSOT_UTILS_SOURCES ${OPENSOT_UTILS_SOURCES}
        src/utils/convex_hull_utils.cpp)
endif()

##VARIABLES
set(OPENSOT_VARIABLES_SOURCES src/variables/Torque.cpp)

##VARIABLES
set(OPENSOT_FLOATING_BASE_ESTIMATION_SOURCES src/floating_base_estimation/qp_estimation.cpp)

ADD_LIBRARY(OpenSoT SHARED
                    ${OPENSOT_CONSTRAINTS_SOURCES}
                    ${OPENSOT_TASKS_SOURCES}
                    ${OPENSOT_SOLVERS_SOURCES}
                    ${OPENSOT_UTILS_SOURCES}
                    ${OPENSOT_VARIABLES_SOURCES}
                    ${OPENSOT_FLOATING_BASE_ESTIMATION_SOURCES}
#                    src/constraints/velocity/Dynamics.cpp
#                    src/stacks/velocity/ManipulationStack.cpp
#                    src/stacks/velocity/WalkingStack.cpp
                    ${sot_INCLUDES})

set(PRIVATE_TLL ${XBotInterface_LIBRARIES})
if(${FCL_FOUND} AND ${moveit_core_FOUND})
    set(PRIVATE_TLL ${PRIVATE_TLL} ${FCL_LIBRARIES}
                              ${moveit_core_LIBRARIES})
endif()
if(${PCL_FOUND})
    set(PRIVATE_TLL ${PRIVATE_TLL} ${PCL_LIBRARIES})
endif()


TARGET_LINK_LIBRARIES(OpenSoT PUBLIC
                              ${srdfdom_advr_LIBRARIES}
                              ${eigen_conversions_LIBRARIES}
			      
                              PRIVATE
                              ${PRIVATE_TLL})


########################################################################
# Compile and install back ends                                        #
########################################################################
# Find package qpOASES or build it using ExternalProject
find_package(qpOASES QUIET)
if(NOT qpOASES_FOUND)
    message("Internal qpOASES will be used!")
    set(qpOASES_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/qpOASES-ext/")
    set(qpOASES_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external/src/qpOASES-ext-build/")

    ExternalProject_Add(qpOASES-ext SOURCE_DIR "${qpOASES_SOURCE_DIR}"
                                    PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                    INSTALL_COMMAND ""
                                    CMAKE_ARGS -DCMAKE_CXX_FLAGS:STRING="-fPIC")
    link_directories("${qpOASES_BINARY_DIR}/libs/")
    set(qpOASES_INCLUDE_DIRS "${qpOASES_SOURCE_DIR}/include")
    set(qpOASES_LIBRARIES qpOASES)
    set(qpOASES_FOUND TRUE)
endif()

include_directories("${qpOASES_INCLUDE_DIRS}")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroInstallLib)

if(${qpOASES_FOUND})
    message("Adding src/solvers/QPOasesBackEnd.cpp to compilation")
    add_library(OpenSotBackEndQPOases SHARED src/solvers/QPOasesBackEnd.cpp)

    if(TARGET qpOASES-ext)
        add_dependencies(OpenSotBackEndQPOases qpOASES-ext)
    endif()

    target_link_libraries(OpenSotBackEndQPOases OpenSoT qpOASES)
    library_install(OpenSotBackEndQPOases 1 0 0)

endif()

FIND_PACKAGE(osqp QUIET)
if(${osqp_FOUND})
    message("Adding src/solvers/OSQPBackEnd.cpp to compilation")

    add_library(OpenSotBackEndOSQP SHARED src/solvers/OSQPBackEnd.cpp)
    target_link_libraries(OpenSotBackEndOSQP OpenSoT osqp)
    library_install(OpenSotBackEndOSQP 1 0 0)

endif()

FIND_PACKAGE(GLPK QUIET)
if(${GLPK_FOUND})
    message("Adding src/solvers/GLPKBackEnd.cpp to compilation")

    add_library(OpenSotBackEndGLPK SHARED src/solvers/GLPKBackEnd.cpp)
    target_link_libraries(OpenSotBackEndGLPK OpenSoT -lglpk)
    library_install(OpenSotBackEndGLPK 1 0 0)
endif()


add_subdirectory(external/eiQuadProg-ext/)
include_directories(external/eiQuadProg-ext/include)
message("Adding src/solvers/eiQuadProgBackEnd.cpp to compilation")
add_library(OpenSotBackEndeiQuadProg SHARED src/solvers/eiQuadProgBackEnd.cpp)
target_link_libraries(OpenSotBackEndeiQuadProg OpenSoT eiQuadProg)
library_install(OpenSotBackEndeiQuadProg 1 0 0)

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
    CATKIN_DEPENDS roscpp rospy roslib srdfdom
    DEPENDS system_lib)

install(TARGETS OpenSoT
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

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
