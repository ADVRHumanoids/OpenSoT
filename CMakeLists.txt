# Copyright: (C) 2014 Walkman Consortium
# Authors: Enrico Mingo, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(sot_VelKinCon)

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(paramHelp REQUIRED)
FIND_PACKAGE(kdl_codyco REQUIRED)
FIND_PACKAGE(drc_shared REQUIRED)
FIND_PACKAGE(roslib REQUIRED)
FIND_PACKAGE(rospack REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)
FIND_PACKAGE(PCL 1.2 REQUIRED)
FIND_PACKAGE(tf REQUIRED)

enable_testing()

set(qpOASES_SRC_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/external/qpOASES-ext/")
set(qpOASES_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/external/src/qpOASES-ext-build/")
set(ROS_LIB_FOLDER $ENV{ROS_LIB_FOLDER} CACHE STRING "ros lib folder")

ExternalProject_Add(qpOASES-ext SOURCE_DIR "${qpOASES_SRC_FOLDER}"
                            PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                            INSTALL_COMMAND "")
                            #CMAKE_ARGS -DCMAKE_CXX_FLAGS:STRING="-fPIC")


#finding GTest
#FIND_PACKAGE(GTest REQUIRED)
set(gTestSource "/usr/src/gtest")
ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                              PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                              INSTALL_COMMAND "")
set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
set(GTEST_INCLUDE_DIRS ${gTestSource})
set(GTEST_BOTH_LIBRARIES gtest gtest_main)

SET(ROS_HYDRO_LIB_FOLDER /opt/ros/hydro/lib/)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${paramHelp_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} "${qpOASES_SRC_FOLDER}/include/" ${PCL_INCLUDE_DIRS}
${GTEST_INCLUDE_DIRS})
LINK_DIRECTORIES(${ROS_LIB_FOLDER} "${qpOASES_FOLDER}/libs/" ${PCL_LIBRARY_DIRS} ${GTEST_LIB_DIRS})

ADD_DEFINITIONS(${PCL_DEFINITIONS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)
file(GLOB_RECURSE sot_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/python" *.py)

ADD_LIBRARY(wb_sot src/wb_sot/bounds/velocity/Aggregated.cpp
                   src/wb_sot/bounds/velocity/CoMVelocity.cpp
                   src/wb_sot/bounds/velocity/JointLimits.cpp
                   src/wb_sot/bounds/velocity/VelocityLimits.cpp)

add_custom_target(  wb_sot_test ALL
                    COMMAND ${CMAKE_CTEST_COMMAND}
                    DEPENDS wb_sot)

ADD_EXECUTABLE(sot_VelKinCon src/main.cpp src/sot_VelKinCon_ctrl.cpp src/yarp_interface.cpp src/task_solver.cpp src/convex_hull.cpp  ${sot_INCLUDES} ${sot_SCRIPTS})
ADD_EXECUTABLE(createMonitorScope src/tests/createMonitorScope.cpp)
ADD_EXECUTABLE(createParamTuner src/tests/createParamTuner.cpp)
ADD_EXECUTABLE(uTest1 src/sot_VelKinCon_ctrl.cpp src/yarp_interface.cpp src/tests/unitTest1.cpp src/task_solver.cpp src/convex_hull.cpp)
ADD_EXECUTABLE(uTest2 src/tests/unitTest2.cpp)

# gTests for Velocity Bounds
SET(VelBLibs  wb_sot ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES}
             ${kdl_codyco_LIBRARIES} orocos-kdl kdl_parser
             ${iDynTree_LIBRARIES} ${YARP_LIBRARIES}
             icubmod ${ICUB_LIBRARIES}
             ${GTEST_BOTH_LIBRARIES})

ADD_EXECUTABLE(testAggregatedVelB     src/tests/wb_sot/bounds/velocity/TestAggregated.cpp)
TARGET_LINK_LIBRARIES(testAggregatedVelB ${VelBLibs})
add_dependencies(testAggregatedVelB GTest-ext wb_sot)

ADD_EXECUTABLE(testJointLimitsVelB    src/tests/wb_sot/bounds/velocity/TestJointLimits.cpp)
TARGET_LINK_LIBRARIES(testJointLimitsVelB ${VelBLibs})
add_dependencies(testJointLimitsVelB GTest-ext wb_sot)

ADD_EXECUTABLE(testVelocityLimitsVelB src/tests/wb_sot/bounds/velocity/TestVelocityLimits.cpp)
TARGET_LINK_LIBRARIES(testVelocityLimitsVelB ${VelBLibs})
add_dependencies(testVelocityLimitsVelB GTest-ext wb_sot)

ADD_EXECUTABLE(testCoMVelocityVelB src/tests/wb_sot/bounds/velocity/TestCoMVelocity.cpp)
TARGET_LINK_LIBRARIES(testCoMVelocityVelB ${VelBLibs})
add_dependencies(testCoMVelocityVelB GTest-ext wb_sot)

SET(SolverLibs wb_sot qpOASES ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES}
               ${kdl_codyco_LIBRARIES} orocos-kdl kdl_parser
               ${iDynTree_LIBRARIES} ${YARP_LIBRARIES}
               icubmod ${ICUB_LIBRARIES}
               ${GTEST_BOTH_LIBRARIES})
ADD_EXECUTABLE(testSolverQPOases src/tests/wb_sot/solvers/TestQPOasis.cpp)
TARGET_LINK_LIBRARIES(testSolverQPOases ${SolverLibs})
add_dependencies(testSolverQPOases GTest-ext wb_sot qpOASES-ext)


add_test(wb_sot_bounds_velocity_Aggregated testAggregatedVelB)
add_test(wb_sot_bounds_velocity_JointLimits  testJointLimitsVelB)
add_test(wb_sot_bounds_velocity_VelocityLimit testVelocityLimitsVelB)
add_test(wb_sot_bounds_velocity_CoMVelocity testCoMVelocityVelB)
add_test(wb_sot_bounds_solvers_qpOases testSolverQPOases)

# we must wait for qpOASES-ext to be ready before building sot_VelKinCon_ctrl
add_dependencies(sot_VelKinCon qpOASES-ext)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(sot_VelKinCon ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES}  ${moveit_core_LIBRARIES} ${srdfdom_LIBRARIES} ${rospack_LIBRARIES} ${rosconsole_LIBRARIES} ${roslib_LIBRARIES}
                                    ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${paramHelp_LIBRARIES} orocos-kdl urdf kdl_parser ${PCL_LIBRARIES}
                      qpOASES ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES} )
TARGET_LINK_LIBRARIES(uTest1  ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES}  ${moveit_core_LIBRARIES} ${srdfdom_LIBRARIES} ${rospack_LIBRARIES} ${rosconsole_LIBRARIES} ${roslib_LIBRARIES}
                              ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${paramHelp_LIBRARIES} orocos-kdl urdf kdl_parser ${PCL_LIBRARIES}
                      qpOASES ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(uTest2 ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES} ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} orocos-kdl urdf kdl_parser ${PCL_LIBRARIES}
                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(createMonitorScope ${YARP_LIBRARIES} ${paramHelp_LIBRARIES})

TARGET_LINK_LIBRARIES(createParamTuner ${YARP_LIBRARIES} ${paramHelp_LIBRARIES})


install(TARGETS sot_VelKinCon DESTINATION bin)
install(TARGETS wb_sot DESTINATION lib)

add_subdirectory(app)

add_custom_target(  copy_conf_files_to_build ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/app/conf"
        "${CMAKE_BINARY_DIR}/")

add_custom_target( copy_python_to_build ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/python"
        "${CMAKE_BINARY_DIR}/")

add_custom_target(generate_paramsTuner_gui ALL
    COMMAND "${CMAKE_BINARY_DIR}/createParamTuner" > "${CMAKE_CURRENT_SOURCE_DIR}/app/resources/tuner.xml")
add_dependencies(generate_paramsTuner_gui createParamTuner)
