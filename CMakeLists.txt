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

enable_testing()

set(qpOASES_SRC_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/external/qpOASES-ext/")
set(qpOASES_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/external/src/qpOASES-ext-build/")

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

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${paramHelp_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} "${qpOASES_SRC_FOLDER}/include/" ${PCL_INCLUDE_DIRS}
${GTEST_INCLUDE_DIRS})
LINK_DIRECTORIES("${qpOASES_FOLDER}/libs/" ${GTEST_LIB_DIRS})

ADD_DEFINITIONS(${PCL_DEFINITIONS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)
file(GLOB_RECURSE sot_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/python" *.py)

ADD_LIBRARY(wb_sot src/wb_sot/bounds/Aggregated.cpp
                   src/wb_sot/bounds/BilateralConstraint.cpp
                   src/wb_sot/bounds/velocity/CoMVelocity.cpp
                   src/wb_sot/bounds/velocity/ConvexHull.cpp
                   src/wb_sot/bounds/velocity/JointLimits.cpp
                   src/wb_sot/bounds/velocity/VelocityLimits.cpp
                   src/wb_sot/solvers/QPOases.cpp
                   src/wb_sot/tasks/Aggregated.cpp
                   src/wb_sot/tasks/velocity/Cartesian.cpp
                   src/wb_sot/tasks/velocity/CoM.cpp
                   src/wb_sot/tasks/velocity/MinimumEffort.cpp
                   src/wb_sot/tasks/velocity/Postural.cpp
                   )

add_custom_target(  wb_sot_test ALL
                    COMMAND ${CMAKE_CTEST_COMMAND}
                    DEPENDS wb_sot
                    testCartesianTask
                    TestComVelocityTask
                    testMinimumEffortVelocityTask
                    testPosturalVelocityTask)

ADD_EXECUTABLE(sot_VelKinCon src/main.cpp src/sot_VelKinCon_ctrl.cpp src/yarp_interface.cpp src/task_solver2.cpp ${sot_INCLUDES} ${sot_SCRIPTS})
ADD_EXECUTABLE(createMonitorScope src/tests/createMonitorScope.cpp)
ADD_EXECUTABLE(createParamTuner src/tests/createParamTuner.cpp)
ADD_EXECUTABLE(uTest1 src/sot_VelKinCon_ctrl.cpp src/yarp_interface.cpp src/tests/unitTest1.cpp src/task_solver2.cpp)
ADD_EXECUTABLE(uTest2 src/tests/unitTest2.cpp)

# gTests for Velocity Bounds
SET(VelocityBoundsLibs  wb_sot qpOASES ${drc_shared_LIBRARIES}
             ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES}
             ${iDynTree_LIBRARIES} ${YARP_LIBRARIES}
             icubmod ${ICUB_LIBRARIES}
             ${GTEST_BOTH_LIBRARIES})

ADD_EXECUTABLE(testAggregatedVelocityBounds     src/tests/wb_sot/bounds/TestAggregated.cpp)
TARGET_LINK_LIBRARIES(testAggregatedVelocityBounds ${VelocityBoundsLibs})
add_dependencies(testAggregatedVelocityBounds GTest-ext wb_sot)

ADD_EXECUTABLE(testJointLimitsVelocityBounds    src/tests/wb_sot/bounds/velocity/TestJointLimits.cpp)
TARGET_LINK_LIBRARIES(testJointLimitsVelocityBounds ${VelocityBoundsLibs})
add_dependencies(testJointLimitsVelocityBounds GTest-ext wb_sot)

ADD_EXECUTABLE(testVelocityLimitsVelocityBounds src/tests/wb_sot/bounds/velocity/TestVelocityLimits.cpp)
TARGET_LINK_LIBRARIES(testVelocityLimitsVelocityBounds ${VelocityBoundsLibs})
add_dependencies(testVelocityLimitsVelocityBounds GTest-ext wb_sot)

ADD_EXECUTABLE(testCoMVelocityVelocityBounds src/tests/wb_sot/bounds/velocity/TestCoMVelocity.cpp)
TARGET_LINK_LIBRARIES(testCoMVelocityVelocityBounds ${VelocityBoundsLibs})
add_dependencies(testCoMVelocityVelocityBounds GTest-ext wb_sot)

ADD_EXECUTABLE(testConvexHullVelocityBounds src/tests/wb_sot/bounds/velocity/TestConvexHull.cpp)
TARGET_LINK_LIBRARIES(testConvexHullVelocityBounds ${VelocityBoundsLibs} ${PCL_LIBRARIES})
add_dependencies(testConvexHullVelocityBounds GTest-ext wb_sot)

ADD_EXECUTABLE(testAggregatedTask src/tests/wb_sot/tasks/TestAggregated.cpp)
TARGET_LINK_LIBRARIES(testAggregatedTask ${VelocityBoundsLibs} ${PCL_LIBRARIES})
add_dependencies(testAggregatedTask GTest-ext wb_sot)

ADD_EXECUTABLE(testCartesianTask src/tests/wb_sot/tasks/velocity/TestCartesian.cpp)
TARGET_LINK_LIBRARIES(testCartesianTask ${VelocityBoundsLibs} ${PCL_LIBRARIES})
add_dependencies(testCartesianTask GTest-ext wb_sot)

ADD_EXECUTABLE(TestComVelocityTask src/tests/wb_sot/tasks/velocity/TestCoM.cpp)
TARGET_LINK_LIBRARIES(TestComVelocityTask ${VelocityBoundsLibs} ${PCL_LIBRARIES})
add_dependencies(TestComVelocityTask GTest-ext wb_sot)

ADD_EXECUTABLE(testMinimumEffortVelocityTask src/tests/wb_sot/tasks/velocity/TestMinimumEffort.cpp)
TARGET_LINK_LIBRARIES(testMinimumEffortVelocityTask ${VelocityBoundsLibs} ${PCL_LIBRARIES})
add_dependencies(testMinimumEffortVelocityTask GTest-ext wb_sot)

ADD_EXECUTABLE(testPosturalVelocityTask src/tests/wb_sot/tasks/velocity/TestPostural.cpp)
TARGET_LINK_LIBRARIES(testPosturalVelocityTask ${VelocityBoundsLibs} ${PCL_LIBRARIES})
add_dependencies(testPosturalVelocityTask GTest-ext wb_sot)

SET(SolverLibs wb_sot qpOASES ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES}
               ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES}
               ${iDynTree_LIBRARIES} ${YARP_LIBRARIES}
               icubmod ${ICUB_LIBRARIES}
               ${GTEST_BOTH_LIBRARIES})
ADD_EXECUTABLE(testSolverQPOases src/tests/wb_sot/solvers/TestQPOases.cpp)
TARGET_LINK_LIBRARIES(testSolverQPOases ${SolverLibs})
add_dependencies(testSolverQPOases GTest-ext wb_sot qpOASES-ext)


add_test(wb_sot_bounds_Aggregated testAggregatedVelocityBounds)
add_test(wb_sot_bounds_velocity_JointLimits  testJointLimitsVelocityBounds)
add_test(wb_sot_bounds_velocity_VelocityLimit testVelocityLimitsVelocityBounds)
add_test(wb_sot_bounds_velocity_CoMVelocity testCoMVelocityVelocityBounds)
add_test(wb_sot_bounds_velocity_ConvexHull testConvexHullVelocityBounds)
add_test(wb_sot_bounds_solvers_qpOases testSolverQPOases)
add_test(wb_sot_task_Aggregated testAggregatedTask)
add_test(wb_sot_task_velocity_Cartesian testCartesianTask)
add_test(wb_sot_task_velocity_CoM TestComVelocityTask)
add_test(wb_sot_task_velocity_MinimumEffort testMinimumEffortVelocityTask)
add_test(wb_sot_task_velocity_Postural testPosturalVelocityTask)

# we must wait for qpOASES-ext to be ready before building sot_VelKinCon_ctrl
add_dependencies(sot_VelKinCon qpOASES-ext)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(wb_sot ${drc_shared_LIBRARIES} ${iDynTree_LIBRARIES} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(sot_VelKinCon wb_sot ${drc_shared_LIBRARIES} ${moveit_core_LIBRARIES} ${srdfdom_LIBRARIES} ${rospack_LIBRARIES} ${rosconsole_LIBRARIES} ${roslib_LIBRARIES}
                                    ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${paramHelp_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdf_LIBRARIES} ${kdl_parser_LIBRARIES} ${PCL_LIBRARIES}
                      qpOASES ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES} )
TARGET_LINK_LIBRARIES(uTest1  wb_sot ${drc_shared_LIBRARIES} ${moveit_core_LIBRARIES} ${srdfdom_LIBRARIES} ${rospack_LIBRARIES} ${rosconsole_LIBRARIES} ${roslib_LIBRARIES}
                              ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${paramHelp_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdf_LIBRARIES} ${kdl_parser_LIBRARIES} ${PCL_LIBRARIES}
                      qpOASES ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(uTest2 ${drc_shared_LIBRARIES} ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${orocos_kdl_LIBRARIES} ${urdf_LIBRARIES} ${kdl_parser_LIBRARIES} ${PCL_LIBRARIES}
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
