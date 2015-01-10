cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

# compite GTest
find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

# add local include
include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()

if(TRUE OR YCM_EP_DEVEL_MODE_OpenSoT)
  add_custom_command( TARGET OpenSoT POST_BUILD
                      COMMAND ${CMAKE_CTEST_COMMAND}
                      MAIN_DEPENDENCY OpenSoT
                      DEPENDS
                      testAggregatedConstraint
                      testBilateralConstraint
                      testJointLimitsVelocityBounds
                      testVelocityLimitsVelocityBounds
                      testCoMVelocityVelocityConstraint
                      testConvexHullVelocityConstraint
                      testQPOasesSolver
                      testQPOases_ConvexHull
                      testQPOases_FF
                      testAggregatedTask
                      testCartesianVelocityTask
                      testCoMVelocityTask
                      testMinimumEffortVelocityTask
                      testPosturalVelocityTask
                      testYCartesian
                      testYPostural)
endif()

# gTests for Velocity constraints
SET(TestLibs OpenSoT ${idynutils_LIBRARIES} ${idynutils_LIBRARIES}
               ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES}
               ${iDynTree_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES}
               ${GTEST_BOTH_LIBRARIES})

ADD_EXECUTABLE(testAggregatedConstraint     constraints/TestAggregated.cpp)
TARGET_LINK_LIBRARIES(testAggregatedConstraint ${TestLibs})
add_dependencies(testAggregatedConstraint GTest-ext OpenSoT)

ADD_EXECUTABLE(testBilateralConstraint     constraints/TestBilateralConstraint.cpp)
TARGET_LINK_LIBRARIES(testBilateralConstraint ${TestLibs})
add_dependencies(testBilateralConstraint GTest-ext OpenSoT)

ADD_EXECUTABLE(testJointLimitsVelocityBounds    constraints/velocity/TestJointLimits.cpp)
TARGET_LINK_LIBRARIES(testJointLimitsVelocityBounds ${TestLibs})
add_dependencies(testJointLimitsVelocityBounds GTest-ext OpenSoT)

ADD_EXECUTABLE(testVelocityLimitsVelocityBounds constraints/velocity/TestVelocityLimits.cpp)
TARGET_LINK_LIBRARIES(testVelocityLimitsVelocityBounds ${TestLibs})
add_dependencies(testVelocityLimitsVelocityBounds GTest-ext OpenSoT)

ADD_EXECUTABLE(testCoMVelocityVelocityConstraint constraints/velocity/TestCoMVelocity.cpp)
TARGET_LINK_LIBRARIES(testCoMVelocityVelocityConstraint ${TestLibs})
add_dependencies(testCoMVelocityVelocityConstraint GTest-ext OpenSoT)

ADD_EXECUTABLE(testConvexHullVelocityConstraint constraints/velocity/TestConvexHull.cpp)
TARGET_LINK_LIBRARIES(testConvexHullVelocityConstraint ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testConvexHullVelocityConstraint GTest-ext OpenSoT)

ADD_EXECUTABLE(testQPOasesSolver solvers/TestQPOases.cpp)
TARGET_LINK_LIBRARIES(testQPOasesSolver ${TestLibs})
add_dependencies(testQPOasesSolver GTest-ext OpenSoT)

ADD_EXECUTABLE(testQPOases_FF solvers/TestQPOases_FF.cpp)
TARGET_LINK_LIBRARIES(testQPOases_FF ${TestLibs})
add_dependencies(testQPOases_FF GTest-ext OpenSoT)

ADD_EXECUTABLE(testQPOases_ConvexHull solvers/TestQPOases_ConvexHull.cpp)
TARGET_LINK_LIBRARIES(testQPOases_ConvexHull ${TestLibs})
add_dependencies(testQPOases_ConvexHull GTest-ext OpenSoT)

ADD_EXECUTABLE(testAggregatedTask tasks/TestAggregated.cpp)
TARGET_LINK_LIBRARIES(testAggregatedTask ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testAggregatedTask GTest-ext OpenSoT)

ADD_EXECUTABLE(testCartesianVelocityTask tasks/velocity/TestCartesian.cpp)
TARGET_LINK_LIBRARIES(testCartesianVelocityTask ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testCartesianVelocityTask GTest-ext OpenSoT)

ADD_EXECUTABLE(testCoMVelocityTask tasks/velocity/TestCoM.cpp)
TARGET_LINK_LIBRARIES(testCoMVelocityTask ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testCoMVelocityTask GTest-ext OpenSoT)

ADD_EXECUTABLE(testMinimumEffortVelocityTask tasks/velocity/TestMinimumEffort.cpp)
TARGET_LINK_LIBRARIES(testMinimumEffortVelocityTask ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testMinimumEffortVelocityTask GTest-ext OpenSoT)

ADD_EXECUTABLE(testPosturalVelocityTask tasks/velocity/TestPostural.cpp)
TARGET_LINK_LIBRARIES(testPosturalVelocityTask ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testPosturalVelocityTask GTest-ext OpenSoT)

ADD_EXECUTABLE(testAutoStack utils/TestAutoStack.cpp)
TARGET_LINK_LIBRARIES(testAutoStack ${TestLibs})
add_dependencies(testAutoStack GTest-ext OpenSoT)

ADD_EXECUTABLE(testDefaultHumanoidStack utils/TestDefaultHumanoidStack.cpp)
TARGET_LINK_LIBRARIES(testDefaultHumanoidStack ${TestLibs})
add_dependencies(testDefaultHumanoidStack GTest-ext OpenSoT)

ADD_EXECUTABLE(testYCartesian interfaces/yarp/tasks/testYCartesian.cpp)
TARGET_LINK_LIBRARIES(testYCartesian ${TestLibs})
add_dependencies(testYCartesian GTest-ext OpenSoT)

ADD_EXECUTABLE(testYPostural interfaces/yarp/tasks/testYPostural.cpp)
TARGET_LINK_LIBRARIES(testYPostural ${TestLibs})
add_dependencies(testYPostural GTest-ext OpenSoT)

add_test(NAME OpenSoT_constraints_Aggregated COMMAND testAggregatedConstraint)
add_test(NAME OpenSoT_constraints_BilateralConstraint COMMAND testBilateralConstraint)
add_test(NAME OpenSoT_constraints_velocity_JointLimits  COMMAND testJointLimitsVelocityBounds)
add_test(NAME OpenSoT_constraints_velocity_VelocityLimit COMMAND testVelocityLimitsVelocityBounds)
add_test(NAME OpenSoT_constraints_velocity_CoMVelocity COMMAND testCoMVelocityVelocityConstraint)
add_test(NAME OpenSoT_constraints_velocity_ConvexHull COMMAND testConvexHullVelocityConstraint)
add_test(NAME OpenSoT_interfaces_cartesian COMMAND testYCartesian)
add_test(NAME OpenSoT_interfaces_postural COMMAND testYPostural)
add_test(NAME OpenSoT_solvers_qpOases COMMAND testQPOasesSolver)
add_test(NAME OpenSoT_solvers_qpOases_FF COMMAND testQPOases_FF)
add_test(NAME OpenSoT_solvers_qpOases_ConvexHull COMMAND testQPOases_ConvexHull)
add_test(NAME OpenSoT_task_Aggregated COMMAND testAggregatedTask)
add_test(NAME OpenSoT_task_velocity_Cartesian COMMAND testCartesianVelocityTask)
add_test(NAME OpenSoT_task_velocity_CoM COMMAND testCoMVelocityTask)
add_test(NAME OpenSoT_task_velocity_MinimumEffort COMMAND testMinimumEffortVelocityTask)
add_test(NAME OpenSoT_task_velocity_Postural COMMAND testPosturalVelocityTask)
add_test(NAME OpenSoT_utils_AutoStack COMMAND testAutoStack)
add_test(NAME OpenSoT_utils_DefaultHumanoidStack COMMAND testDefaultHumanoidStack)