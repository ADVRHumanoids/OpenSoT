# Copyright: (C) 2014 Walkman Consortium
# Authors: Enrico Mingo Hoffman, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.0)
include(ExternalProject)
project(open_sot VERSION 3.0.0)

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
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

set(${CMAKE_PROJECT_NAME}_CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})


find_package(Boost COMPONENTS system REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_core QUIET)
find_package(PCL 1.7 QUIET COMPONENTS   common
                                        filters
                                        surface
                                        search
                                        io)
find_package(eigen_conversions REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(FCL 0.6 QUIET)
find_package(PkgConfig REQUIRED)
find_package(matlogger2 REQUIRED)

# compilation flags
option(OPENSOT_COMPILE_EXAMPLES "Compile OpenSoT examples" FALSE)
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
    )

ADD_DEFINITIONS(${PCL_DEFINITIONS})


# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

##CONSTRAINTS
set(OPENSOT_CONSTRAINTS_SOURCES src/constraints/Aggregated.cpp
    src/constraints/BilateralConstraint.cpp
    src/constraints/TaskToConstraint.cpp
    src/constraints/acceleration/VelocityLimits.cpp
    src/constraints/velocity/CartesianPositionConstraint.cpp
    src/constraints/velocity/CapturePoint.cpp
    src/constraints/velocity/CartesianVelocity.cpp
    src/constraints/velocity/CoMVelocity.cpp
    src/constraints/velocity/JointLimits.cpp
    src/constraints/velocity/affine/JointLimits.cpp
    src/constraints/velocity/VelocityLimits.cpp
    src/constraints/velocity/affine/VelocityLimits.cpp
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
    src/tasks/acceleration/DynamicFeasibility.cpp
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
    src/solvers/eHQP.cpp
    src/solvers/l1HQP.cpp)

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
    ${sot_INCLUDES})

set(PRIVATE_TLL ${eigen_conversions_LIBRARIES})
if(${FCL_FOUND} AND ${moveit_core_FOUND})
    set(PRIVATE_TLL ${PRIVATE_TLL} ${FCL_LIBRARIES}
        ${moveit_core_LIBRARIES})
endif()
if(${PCL_FOUND})
    set(PRIVATE_TLL ${PRIVATE_TLL} ${PCL_LIBRARIES})
endif()


TARGET_LINK_LIBRARIES(OpenSoT
    PUBLIC
    XBotInterface::XBotInterface
    matlogger2::matlogger2
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
        CMAKE_ARGS -DCMAKE_CXX_FLAGS:STRING="-fPIC" -DCMAKE_BUILD_TYPE=${${CMAKE_PROJECT_NAME}_CMAKE_BUILD_TYPE})
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

find_package(osqp QUIET)
if(${osqp_FOUND})
    message("Adding src/solvers/OSQPBackEnd.cpp to compilation")

    add_library(OpenSotBackEndOSQP SHARED src/solvers/OSQPBackEnd.cpp)
    target_link_libraries(OpenSotBackEndOSQP OpenSoT osqp::osqpstatic)
    library_install(OpenSotBackEndOSQP 1 0 0)

endif()

find_package(GLPK QUIET)
if(${GLPK_FOUND})
    message("Adding src/solvers/GLPKBackEnd.cpp to compilation")
    add_library(OpenSotBackEndGLPK SHARED src/solvers/GLPKBackEnd.cpp)
    target_link_libraries(OpenSotBackEndGLPK OpenSoT ${GLPK_LIBRARY})
    library_install(OpenSotBackEndGLPK 1 0 0)
endif()


add_subdirectory(external/eiQuadProg-ext/)
include_directories(external/eiQuadProg-ext/include)
message("Adding src/solvers/eiQuadProgBackEnd.cpp to compilation")
add_library(OpenSotBackEndeiQuadProg SHARED src/solvers/eiQuadProgBackEnd.cpp)
target_link_libraries(OpenSotBackEndeiQuadProg OpenSoT eiQuadProg)
library_install(OpenSotBackEndeiQuadProg 1 0 0)

#############################################################
# Export OpenSoT so that it can be found using find_package #
#############################################################

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h")

include(AddUninstallTarget)
include(MacroInstallLib)

library_install(OpenSoT
    ${PROJECT_VERSION_MAJOR}
    ${PROJECT_VERSION_MINOR}
    ${PROJECT_VERSION_PATCH})


add_subdirectory(doc)

include(GenerateDeb)


#######################
# Add Testing and Examples target  #
#######################
if(OPENSOT_COMPILE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(OPENSOT_COMPILE_EXAMPLES)
    add_subdirectory(examples)
endif()

if(OPENSOT_COMPILE_TESTS OR OPENSOT_COMPILE_EXAMPLES)
    add_custom_target(copy_robot_model_files ALL
        ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/tests/robots" "${CMAKE_CURRENT_BINARY_DIR}/tests/robots")
endif()
