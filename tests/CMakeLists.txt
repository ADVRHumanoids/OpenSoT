cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

#TESTS THAT USES GAZEBO
set(OPENSOT_COMPILE_SIMULATION_TESTS OFF CACHE BOOL "Flag to compile tests that uses Gazebo")

if (OPENSOT_COMPILE_SIMULATION_TESTS)
    add_definitions( -DOPENSOT_COMPILE_SIMULATION_TESTS=true )
    #Gazebo
    find_package(Gazebo   REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
    link_directories(${GAZEBO_LIBRARY_DIRS})
    include_directories(${GAZEBO_INCLUDE_DIRS})

    set(OPENSOT_SIMULATION_TESTS_VISUALIZATION OFF CACHE BOOL "Flag to show simulation tests")
    if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
        add_definitions( -DOPENSOT_SIMULATION_TESTS_VISUALIZATION=true )
    else()
        add_definitions( -DOPENSOT_SIMULATION_TESTS_VISUALIZATION=false )
    endif()
else()
    add_definitions( -DOPENSOT_COMPILE_SIMULATION_TESTS=false )
endif()

option(SHOW_TEST_RESULTS "After running tests, show result plots" FALSE)

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
                      #testBilateralConstraint
                      #testCartesianPositionVelocityConstraint
                      #testCollisionAvoidanceVelocityConstraint
                      #testCoMVelocityVelocityConstraint
                      testConvexHullVelocityConstraint
                      #testDynamicsConstraint
                      testJointLimitsVelocityBounds
                      testVelocityLimitsVelocityBounds
                      testQPOasesSolver
                      #testQPOases_AutoStack
                      #testQPOases_CartesianPositionConstraint
                      #testQPOases_ConvexHull
                      #testQPOases_FF
                      #testQPOases_GlobalConstraints.cpp
                      #testQPOases_Options
                      #testQPOases_PriorityInversion
                      #testQPOases_SelfCollisionAvoidance
                      #testQPOases_Sparse
                      #testQPOases_SubTask
                      #testQPOases_VelocityAllocation
                      testAggregatedTask
                      testCartesianVelocityTask
                      #testCoMForceTask
                      #testCoMVelocityTask
                      #testGazeVelocityTask
                      #testInteractionVelocityTask
                      #TestManipulabilityTask
                      #testMinimizeAccelerationTask
                      #testMinimumEffortVelocityTask
                      #testMinimumVelocityTask
                      testPosturalVelocityTask
                      #testSubTask
                      #testAutoStack
                      #testDefaultHumanoidStack
                      #testLogger
                      #testPreviewer
                      #testVelocityAllocation
                      #testYCartesian
                      #testYPostural
)
endif()

# gTests for Velocity constraints
SET(TestLibs OpenSoT ${GTEST_BOTH_LIBRARIES} ${idynutils_LIBRARIES} ${idynutils_LIBRARIES}
               ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES}
               ${iDynTree_LIBRARIES} ${YARP_LIBRARIES})

add_definitions(-DOPENSOT_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/robots/")

ADD_EXECUTABLE(testAggregatedConstraint     constraints/TestAggregated.cpp)
TARGET_LINK_LIBRARIES(testAggregatedConstraint ${TestLibs})
add_dependencies(testAggregatedConstraint GTest-ext OpenSoT)

#ADD_EXECUTABLE(testBilateralConstraint     constraints/TestBilateralConstraint.cpp)
#TARGET_LINK_LIBRARIES(testBilateralConstraint ${TestLibs})
#add_dependencies(testBilateralConstraint GTest-ext OpenSoT)

#ADD_EXECUTABLE(testCartesianPositionVelocityConstraint constraints/velocity/TestCartesianPositionConstraint.cpp)
#TARGET_LINK_LIBRARIES(testCartesianPositionVelocityConstraint ${TestLibs})
#add_dependencies(testCartesianPositionVelocityConstraint GTest-ext OpenSoT)

#ADD_EXECUTABLE(testCollisionAvoidanceVelocityConstraint constraints/velocity/TestCollisionAvoidance.cpp)
#TARGET_LINK_LIBRARIES(testCollisionAvoidanceVelocityConstraint ${TestLibs})
#add_dependencies(testCollisionAvoidanceVelocityConstraint GTest-ext OpenSoT)

#ADD_EXECUTABLE(testCoMVelocityVelocityConstraint constraints/velocity/TestCoMVelocity.cpp)
#TARGET_LINK_LIBRARIES(testCoMVelocityVelocityConstraint ${TestLibs})
#add_dependencies(testCoMVelocityVelocityConstraint GTest-ext OpenSoT)

ADD_EXECUTABLE(testConvexHullVelocityConstraint constraints/velocity/TestConvexHull.cpp)
TARGET_LINK_LIBRARIES(testConvexHullVelocityConstraint ${TestLibs})
add_dependencies(testConvexHullVelocityConstraint GTest-ext OpenSoT)

#ADD_EXECUTABLE(testDynamicsConstraint constraints/velocity/TestDynamics.cpp)
#TARGET_LINK_LIBRARIES(testDynamicsConstraint ${TestLibs})
#add_dependencies(testDynamicsConstraint GTest-ext OpenSoT)

ADD_EXECUTABLE(testJointLimitsVelocityBounds    constraints/velocity/TestJointLimits.cpp)
TARGET_LINK_LIBRARIES(testJointLimitsVelocityBounds ${TestLibs})
add_dependencies(testJointLimitsVelocityBounds GTest-ext OpenSoT)

ADD_EXECUTABLE(testVelocityLimitsVelocityBounds constraints/velocity/TestVelocityLimits.cpp)
TARGET_LINK_LIBRARIES(testVelocityLimitsVelocityBounds ${TestLibs})
add_dependencies(testVelocityLimitsVelocityBounds GTest-ext OpenSoT)

ADD_EXECUTABLE(testQPOasesSolver solvers/TestQPOases.cpp)
TARGET_LINK_LIBRARIES(testQPOasesSolver ${TestLibs})
add_dependencies(testQPOasesSolver GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_AutoStack solvers/TestQPOases_AutoStack.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_AutoStack ${TestLibs})
#add_dependencies(testQPOases_AutoStack GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_CartesianPositionConstraint solvers/TestQPOases_CartesianPositionConstraint.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_CartesianPositionConstraint ${TestLibs})
#add_dependencies(testQPOases_CartesianPositionConstraint GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_ConvexHull solvers/TestQPOases_ConvexHull.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_ConvexHull ${TestLibs})
#add_dependencies(testQPOases_ConvexHull GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_FF solvers/TestQPOases_FF.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_FF ${TestLibs})
#add_dependencies(testQPOases_FF GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_GlobalConstraints solvers/TestQPOases_GlobalConstraints.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_GlobalConstraints ${TestLibs} ${PCL_LIBRARIES})
#add_dependencies(testQPOases_GlobalConstraints GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_Options solvers/TestQPOases_Options.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_Options ${TestLibs})
#add_dependencies(testQPOases_Options GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_PriorityInversion solvers/TestQPOases_PriorityInversion.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_PriorityInversion ${TestLibs})
#add_dependencies(testQPOases_PriorityInversion GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_SelfCollisionAvoidance solvers/TestQPOases_SelfCollisionAvoidance.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_SelfCollisionAvoidance ${TestLibs})
#add_dependencies(testQPOases_SelfCollisionAvoidance GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_Sparse solvers/TestQPOases_Sparse.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_Sparse ${TestLibs})
#add_dependencies(testQPOases_Sparse GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_SubTask solvers/TestQPOases_SubTask.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_SubTask ${TestLibs})
#add_dependencies(testQPOases_SubTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testQPOases_VelocityAllocation solvers/TestQPOases_VelocityAllocation.cpp)
#TARGET_LINK_LIBRARIES(testQPOases_VelocityAllocation ${TestLibs})
#add_dependencies(testQPOases_VelocityAllocation GTest-ext OpenSoT)

ADD_EXECUTABLE(testAggregatedTask tasks/TestAggregated.cpp)
TARGET_LINK_LIBRARIES(testAggregatedTask ${TestLibs})
add_dependencies(testAggregatedTask GTest-ext OpenSoT)

ADD_EXECUTABLE(testCartesianVelocityTask tasks/velocity/TestCartesian.cpp)
TARGET_LINK_LIBRARIES(testCartesianVelocityTask ${TestLibs})
add_dependencies(testCartesianVelocityTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testCoMForceTask tasks/force/TestCoM.cpp)
#TARGET_LINK_LIBRARIES(testCoMForceTask ${TestLibs})
#add_dependencies(testCoMForceTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testCoMVelocityTask tasks/velocity/TestCoM.cpp)
#TARGET_LINK_LIBRARIES(testCoMVelocityTask ${TestLibs})
#add_dependencies(testCoMVelocityTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testGazeVelocityTask tasks/velocity/TestGaze.cpp)
#TARGET_LINK_LIBRARIES(testGazeVelocityTask ${TestLibs})
#add_dependencies(testGazeVelocityTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testInteractionVelocityTask tasks/velocity/TestInteraction.cpp)
#TARGET_LINK_LIBRARIES(testInteractionVelocityTask ${TestLibs})
#add_dependencies(testInteractionVelocityTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testManipulabilityTask tasks/velocity/TestManipulability.cpp)
#TARGET_LINK_LIBRARIES(testManipulabilityTask ${TestLibs} ${PCL_LIBRARIES})
#add_dependencies(testManipulabilityTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testMinimizeAccelerationTask tasks/velocity/TestMinimizeAcceleration.cpp)
#TARGET_LINK_LIBRARIES(testMinimizeAccelerationTask ${TestLibs} ${PCL_LIBRARIES})
#add_dependencies(testMinimizeAccelerationTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testMinimumEffortVelocityTask tasks/velocity/TestMinimumEffort.cpp)
#TARGET_LINK_LIBRARIES(testMinimumEffortVelocityTask ${TestLibs})
#add_dependencies(testMinimumEffortVelocityTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testMinimumVelocityTask tasks/velocity/TestMinimumVelocity.cpp)
#TARGET_LINK_LIBRARIES(testMinimumVelocityTask ${TestLibs} ${PCL_LIBRARIES})
#add_dependencies(testMinimumVelocityTask GTest-ext OpenSoT)

ADD_EXECUTABLE(testPosturalVelocityTask tasks/velocity/TestPostural.cpp)
TARGET_LINK_LIBRARIES(testPosturalVelocityTask ${TestLibs})
add_dependencies(testPosturalVelocityTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testSubTask tasks/TestSubTask.cpp)
#TARGET_LINK_LIBRARIES(testSubTask ${TestLibs})
#add_dependencies(testSubTask GTest-ext OpenSoT)

#ADD_EXECUTABLE(testAutoStack utils/TestAutoStack.cpp)
#TARGET_LINK_LIBRARIES(testAutoStack ${TestLibs})
#add_dependencies(testAutoStack GTest-ext OpenSoT)

#ADD_EXECUTABLE(testDefaultHumanoidStack utils/TestDefaultHumanoidStack.cpp)
#TARGET_LINK_LIBRARIES(testDefaultHumanoidStack ${TestLibs})
#add_dependencies(testDefaultHumanoidStack GTest-ext OpenSoT)

#ADD_EXECUTABLE(testLogger utils/TestLogger.cpp)
#TARGET_LINK_LIBRARIES(testLogger ${TestLibs})
#add_dependencies(testLogger GTest-ext OpenSoT)

#ADD_EXECUTABLE(testPreviewer utils/TestPreviewer.cpp)
#TARGET_LINK_LIBRARIES(testPreviewer ${TestLibs})
#add_dependencies(testPreviewer GTest-ext OpenSoT)

#ADD_EXECUTABLE(testVelocityAllocation utils/TestVelocityAllocation.cpp)
#TARGET_LINK_LIBRARIES(testVelocityAllocation ${TestLibs})
#add_dependencies(testVelocityAllocation GTest-ext OpenSoT)

#ADD_EXECUTABLE(testYCartesian interfaces/yarp/tasks/testYCartesian.cpp)
#TARGET_LINK_LIBRARIES(testYCartesian ${TestLibs})
#add_dependencies(testYCartesian GTest-ext OpenSoT)

#ADD_EXECUTABLE(testYPostural interfaces/yarp/tasks/testYPostural.cpp)
#TARGET_LINK_LIBRARIES(testYPostural ${TestLibs})
#add_dependencies(testYPostural GTest-ext OpenSoT)

#if(OPENSOT_COMPILE_EXAMPLES)
#    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../examples/KlamptController/include")
#    ADD_EXECUTABLE(testKlamptController ../examples/KlamptController/tests/testKlamptController.cpp
#                                        ../examples/KlamptController/src/KlamptController.cpp)
#    TARGET_LINK_LIBRARIES(testKlamptController ${TestLibs})
#    add_dependencies(testKlamptController GTest-ext OpenSoT)
#    add_test(NAME OpenSoT_KlamptController COMMAND testKlamptController)
#endif()


add_test(NAME OpenSoT_constraints_Aggregated COMMAND testAggregatedConstraint)
#add_test(NAME OpenSoT_constraints_BilateralConstraint COMMAND testBilateralConstraint)
#add_test(NAME OpenSoT_constraints_velocity_CartesianPosition COMMAND testCartesianPositionVelocityConstraint)
#add_test(NAME OpenSoT_constraints_velocity_CollisionAvoidance COMMAND testCollisionAvoidanceVelocityConstraint)
#add_test(NAME OpenSoT_constraints_velocity_CoMVelocity COMMAND testCoMVelocityVelocityConstraint)
add_test(NAME OpenSoT_constraints_velocity_ConvexHull COMMAND testConvexHullVelocityConstraint)
#add_test(NAME OpenSoT_constraints_velocity_Dynamics COMMAND testDynamicsConstraint)
add_test(NAME OpenSoT_constraints_velocity_JointLimits  COMMAND testJointLimitsVelocityBounds)
add_test(NAME OpenSoT_constraints_velocity_VelocityLimit COMMAND testVelocityLimitsVelocityBounds)
add_test(NAME OpenSoT_solvers_qpOases COMMAND testQPOasesSolver)
#add_test(NAME OpenSoT_solvers_qpOases_AutoStack COMMAND testQPOases_AutoStack)
#add_test(NAME OpenSoT_solvers_qpOases_CartesianPositionConstraint COMMAND testQPOases_CartesianPositionConstraint)
#add_test(NAME OpenSoT_solvers_qpOases_ConvexHull COMMAND testQPOases_ConvexHull)
#add_test(NAME OpenSoT_solvers_qpOases_FF COMMAND testQPOases_FF)
#add_test(NAME OpenSoT_solvers_qpOases_GlobalConstraints COMMAND testQPOases_GlobalConstraints)
#add_test(NAME OpenSoT_solvers_qpOases_Options COMMAND testQPOases_Options)
##the following tests compile but don't get executed automatically. They are not proper tests ATM, rather case-studies and references for stack tuning.
##add_test(NAME OpenSoT_solvers_qpOases_PriorityInversion COMMAND testQPOases_PriorityInversion)
#add_test(NAME OpenSoT_solvers_qpOases_SelfCollisionAvoidance COMMAND testQPOases_SelfCollisionAvoidance)
#add_test(NAME OpenSoT_solvers_qpOases_Sparse COMMAND testQPOases_Sparse)
#add_test(NAME OpenSoT_solvers_qpOases_SubTask COMMAND testQPOases_SubTask)
#add_test(NAME OpenSoT_solvers_qpOases_VelocityAllocation COMMAND testQPOases_VelocityAllocation)
add_test(NAME OpenSoT_task_Aggregated COMMAND testAggregatedTask)
#add_test(NAME OpenSoT_task_SubTask COMMAND testSubTask)
add_test(NAME OpenSoT_task_velocity_Cartesian COMMAND testCartesianVelocityTask)
#add_test(NAME OpenSoT_task_force_CoM COMMAND testCoMForceTask)
#add_test(NAME OpenSoT_task_velocity_CoM COMMAND testCoMVelocityTask)
#add_test(NAME OpenSoT_task_velocity_Gaze COMMAND testGazeVelocityTask)
#add_test(NAME OpenSoT_task_velocity_Interaction COMMAND testInteractionVelocityTask)
#add_test(NAME OpenSoT_task_velocity_Manipulability COMMAND testManipulabilityTask)
#add_test(NAME OpenSoT_task_velocity_MinimizeAcceleration COMMAND testMinimizeAccelerationTask)
#add_test(NAME OpenSoT_task_velocity_MinimumEffort COMMAND testMinimumEffortVelocityTask)
#add_test(NAME OpenSoT_task_velocity_MinimumVelocity COMMAND testMinimumVelocityTask)
add_test(NAME OpenSoT_task_velocity_Postural COMMAND testPosturalVelocityTask)
#add_test(NAME OpenSoT_utils_AutoStack COMMAND testAutoStack)
#add_test(NAME OpenSoT_utils_DefaultHumanoidStack COMMAND testDefaultHumanoidStack)
#add_test(NAME OpenSoT_utils_Logger COMMAND testLogger)
#add_test(NAME OpenSoT_utils_Previewer COMMAND testPreviewer)
#add_test(NAME OpenSoT_utils_VelocityAllocation COMMAND testVelocityAllocation)
#add_test(NAME OpenSoT_interfaces_cartesian COMMAND testYCartesian)
#add_test(NAME OpenSoT_interfaces_postural COMMAND testYPostural)


find_package(MATLAB QUIET)
find_package(PythonInterp)
if(MATLAB_FOUND AND PYTHONINTERP_FOUND AND SHOW_TEST_RESULTS)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CTestCustom.cmake ${CMAKE_CURRENT_BINARY_DIR}/../CTestCustom.cmake)
endif()

add_custom_target(copy_test_plotter_m ALL
                  ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plot_test_results.m" "${CMAKE_CURRENT_BINARY_DIR}/")

add_custom_target(copy_test_plotter_py ALL
                  ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plot_test_results.py" "${CMAKE_CURRENT_BINARY_DIR}/")
