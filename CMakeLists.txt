# Copyright: (C) 2014 Walkman Consortium
# Authors: Enrico Mingo Hoffman, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
include(ExternalProject)
project(open_sot VERSION 4.0.0)

set(CMAKE_CXX_STANDARD 20)

option(SET_SSE4_FLAG "set -msse4 flag to gcc" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")


# Set a default build type for single-configuration
# CMake generators if no build type is set.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

set(${CMAKE_PROJECT_NAME}_CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})

add_subdirectory(doc)

find_package(Boost COMPONENTS system REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_core QUIET)

find_package(PCL 1.7 QUIET COMPONENTS   common
                                        filters
                                        surface)
                                        #search
                                        #io)
                                                                                
find_package(tf2_eigen_kdl REQUIRED)
find_package(xbot2_interface REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(matlogger2 REQUIRED)


# compilation flags
option(OPENSOT_COMPILE_EXAMPLES "Compile OpenSoT examples" FALSE)
option(OPENSOT_COMPILE_TESTS "Compile OpenSoT tests" FALSE)
option(OPENSOT_VERBOSE "Some additional prints" FALSE)
option(OPENSOT_VERBOSE_MATLOG "Log all aggregated tasks/constraints to MAT-file" FALSE)
option(OPENSOT_DISABLE_VECTORIZATION "Disable Eigen3 vectorization" FALSE)

option(OPENSOT_COMPILE_COLLISION "Compile OpenSoT collision avoidance" TRUE)

if(${OPENSOT_DISABLE_VECTORIZATION})
    add_definitions(-DEIGEN_DONT_VECTORIZE)
    add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
endif()

if(${OPENSOT_VERBOSE_MATLOG})
    add_definitions(-DOPENSOT_VERBOSE_MATLOG)
endif()

if(${OPENSOT_VERBOSE})
    add_definitions(-DOPENSOT_VERBOSE)
endif()

# add include directories
INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS} ${tf2_eigen_kdl_INCLUDE_DIRS}
    )

ADD_DEFINITIONS(${PCL_DEFINITIONS})
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)


# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

##CONSTRAINTS
set(OPENSOT_CONSTRAINTS_SOURCES src/constraints/Aggregated.cpp
    src/constraints/SubConstraint.cpp
    src/constraints/BilateralConstraint.cpp
    src/constraints/TaskToConstraint.cpp
    src/constraints/acceleration/VelocityLimits.cpp
    src/constraints/acceleration/TorqueLimits.cpp
    src/constraints/velocity/CartesianPositionConstraint.cpp
    src/constraints/velocity/CartesianVelocity.cpp
    src/constraints/velocity/JointLimits.cpp
    src/constraints/velocity/JointLimitsInvariance.cpp
    src/constraints/velocity/VelocityLimits.cpp
    src/constraints/velocity/OmniWheels4X.cpp
    src/constraints/force/FrictionCone.cpp
    src/constraints/force/WrenchLimits.cpp
    src/constraints/force/CoP.cpp
    src/constraints/force/StaticConstraint.cpp
    src/constraints/force/NormalTorque.cpp
    src/constraints/GenericConstraint.cpp
    src/constraints/acceleration/JointLimits.cpp
    src/constraints/acceleration/JointLimitsViability.cpp
    src/constraints/acceleration/JointLimitsECBF.cpp
    )

if(${PCL_FOUND} AND ${moveit_core_FOUND})
    message("Adding src/constraints/velocity/ConvexHull.cpp to compilation")
    set(OPENSOT_CONSTRAINTS_SOURCES ${OPENSOT_CONSTRAINTS_SOURCES}
        src/constraints/velocity/ConvexHull.cpp)
endif()

if(${OPENSOT_COMPILE_COLLISION})
    message("Adding src/constraints/velocity/SelfCollisionAvoidance.cpp to compilation")
    set(OPENSOT_CONSTRAINTS_SOURCES ${OPENSOT_CONSTRAINTS_SOURCES}
        src/constraints/velocity/CollisionAvoidance.cpp
        src/tasks/velocity/CollisionAvoidance.cpp)

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
    src/tasks/acceleration/AngularMomentum.cpp
    src/tasks/velocity/Cartesian.cpp
    src/tasks/velocity/CartesianAdmittance.cpp #TODO: refactor to better implementation of admittance, not using filters
    src/tasks/velocity/PureRolling.cpp
    src/tasks/velocity/Gaze.cpp
    src/tasks/velocity/Contact.cpp
    src/tasks/velocity/CoM.cpp
    src/tasks/velocity/AngularMomentum.cpp
    src/tasks/velocity/LinearMomentum.cpp
    src/tasks/velocity/Manipulability.cpp
    src/tasks/velocity/MinimumEffort.cpp
    src/tasks/velocity/Postural.cpp
    src/tasks/velocity/JointAdmittance.cpp #TODO: refactor to better implementation of admittance, not using filters
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

option(OPENSOT_SOTH_FRONT_END "Add to compilation soth and HCOD front-end" ON)
if(${OPENSOT_SOTH_FRONT_END})
    message("Internal soth will be used!")
    add_subdirectory(external/soth-ext/)
    include_directories(external/soth-ext/include)
    message("Adding src/solvers/HCOD.cpp to compilation")
    set(OPENSOT_SOLVERS_SOURCES ${OPENSOT_SOLVERS_SOURCES} src/solvers/HCOD.cpp)
    set(PRIVATE_TLL hcod_wrapper soth)
endif()


##UTILS
set(OPENSOT_UTILS_SOURCES src/utils/AutoStack.cpp
    src/utils/Affine.cpp
    src/utils/AffineUtils.cpp
    src/utils/Indices.cpp
    src/utils/cartesian_utils.cpp
    src/utils/InverseDynamics.cpp)

if(${PCL_FOUND})
    message("Adding src/utils/convex_hull_utils.cpp to compilation")
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
    ${sot_INCLUDES})

set(PRIVATE_TLL ${PRIVATE_TLL} tf2_eigen_kdl::tf2_eigen_kdl)
if(${OPENSOT_COMPILE_COLLISION})
    target_link_libraries(OpenSoT PUBLIC xbot2_interface::collision)
endif()
if(${PCL_FOUND})
    set(PRIVATE_TLL ${PRIVATE_TLL} ${QHULL_LIBRARIES} ${PCL_LIBRARIES})
endif()


configure_file(version.h.in include/OpenSoT/version.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
install(FILES ${CMAKE_BINARY_DIR}/include/OpenSoT/version.h
        DESTINATION include/OpenSoT)



TARGET_LINK_LIBRARIES(OpenSoT
    PUBLIC
    xbot2_interface::xbot2_interface
    matlogger2::matlogger2
    PRIVATE
    ${PRIVATE_TLL})
    
if(${OPENSOT_SOTH_FRONT_END})
	target_compile_definitions(OpenSoT
        	PUBLIC
        	-DOPENSOT_HAS_SOTH_FRONT_END)
endif()


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

find_package(qpSWIFT QUIET)
if(${qpSWIFT_FOUND})
    message("Adding src/solvers/qpSWIFTBackEnd.cpp to compilation")

    add_library(OpenSotBackEndqpSWIFT SHARED src/solvers/qpSWIFTBackEnd.cpp)
    target_link_libraries(OpenSotBackEndqpSWIFT OpenSoT qpSWIFT::qpSWIFT-static)
    library_install(OpenSotBackEndqpSWIFT 1 0 0)

endif()

find_package(proxsuite QUIET)
if(${proxsuite_FOUND})
    message("Adding src/solvers/proxQPBackEnd.cpp to compilation")

    add_library(OpenSotBackEndproxQP SHARED src/solvers/proxQPBackEnd.cpp)
    target_link_libraries(OpenSotBackEndproxQP OpenSoT proxsuite::proxsuite)
    library_install(OpenSotBackEndproxQP 1 0 0)
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

#include(AddUninstallTarget)
include(MacroInstallLib)

library_install(OpenSoT
    ${PROJECT_VERSION_MAJOR}
    ${PROJECT_VERSION_MINOR}
    ${PROJECT_VERSION_PATCH})


include(GenerateDeb)


configure_file(version.h.in include/OpenSoT/version.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
install(FILES ${CMAKE_BINARY_DIR}/include/OpenSoT/version.h
        DESTINATION include/OpenSoT)

#######################
# Add Testing and Examples target  #
#######################
if(OPENSOT_COMPILE_TESTS)
    add_subdirectory(tests)
endif()

if(OPENSOT_COMPILE_EXAMPLES)
    add_subdirectory(examples)
endif()

##############
## Bindings ##
##############
add_subdirectory(bindings/python)

if(OPENSOT_COMPILE_TESTS OR OPENSOT_COMPILE_EXAMPLES)
    add_custom_target(copy_robot_model_files ALL
        ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/tests/robots" "${CMAKE_CURRENT_BINARY_DIR}/tests/robots")
endif()

