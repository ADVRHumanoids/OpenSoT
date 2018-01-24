cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})


FIND_PACKAGE(YARP QUIET)
FIND_PACKAGE(iDynTree QUIET)
FIND_PACKAGE(conversion_utils_YARP QUIET)
FIND_PACKAGE(idynutils2 QUIET)
find_package(test_utils QUIET)
find_package(trajectory_utils QUIET)
find_package(rviz_visual_tools QUIET)
find_package(ModelInterfaceIDYNUTILS QUIET)
find_package(catkin REQUIRED tf roscpp)


if(${YARP_FOUND})
    set(TEST_INCLUDES ${TEST_INCLUDES} ${YARP_INCLUDE_DIRS} )
endif()
if(${iDynTree_FOUND} AND ${idynutils2_FOUND} AND ${ModelInterfaceIDYNUTILS_FOUND})
    set(TEST_INCLUDES ${TEST_INCLUDES}
                      ${iDynTree_INCLUDE_DIRS}
                      ${idynutils2_INCLUDE_DIRS}
                      ${ModelInterfaceIDYNUTILS_INCLUDE_DIRS})
endif()
if(${conversion_utils_YARP_FOUND} AND ${test_utils_FOUND})
    set(TEST_INCLUDES ${TEST_INCLUDES}
                      ${conversion_utils_YARP_INCLUDE_DIRS}
                      ${test_utils_INCLUDE_DIRS})
endif()
if(${trajectory_utils_FOUND} AND ${rviz_visual_tools_FOUND})
    set(TEST_INCLUDES ${TEST_INCLUDES}
                      ${trajectory_utils_INCLUDE_DIRS}
                      ${rviz_visual_tools_INCLUDE_DIRS})
endif()



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

#ALL THE FOLLOWING TESTS ARE YARP FREE
set(OPENSOT_TESTS testBilateralConstraint
                  testJointLimitsVelocityBounds
                  testVelocityLimitsVelocityBounds 
                  testQPOasesSolver  
                  testQPOases_SetActiveStack 
                  testQPOases_Options  
                  testQPOases_SubTask
                  testFrictionConeForceConstraint 
                  testCoMVelocityTask
                  testManipulabilityTask
                  testMinimizeAccelerationTask
                  testMinimumVelocityTask
                  testMinimumEffortVelocityTask
                  testPosturalVelocityTask
                  testSubTask
                  testAutoStack 
                  testCartesianUtils 
                  testBasicAlgebra 
                  testPosturalAccelerationTask
                  testCartesianAccelerationTask
                  testCoMAccelerationTask
                  testContactFloatingBaseTask
)

if(${PCL_FOUND} AND ${moveit_core_FOUND})
    set(OPENSOT_TESTS ${OPENSOT_TESTS} testAggregatedConstraint)
endif()

#THIS TEST DEPEND ON fcl
if(${fcl_FOUND})
    set(OPENSOT_TESTS ${OPENSOT_TESTS} testCollisionUtils)
endif()

#THESE TESTS DEPEND ON trajectory_utils
if(${trajectory_utils_FOUND})
    set(OPENSOT_TESTS ${OPENSOT_TESTS}
                  testQPOases_StaticWalk_FloatingBase
                  testEigenSVD_StaticWalk_FloatingBase)
endif()

#THESE TESTS DEPEND ON YARP
if(${YARP_FOUND})
    set(OPENSOT_TESTS ${OPENSOT_TESTS}
                    testCollisionAvoidanceVelocityConstraint
                    testCoMVelocityVelocityConstraint
                    testConvexHullVelocityConstraint
                    testQPOases_ConvexHull
                    testQPOases_FF
                    testQPOases_StaticWalk
                    testAggregatedTask
                    testCartesianVelocityTask
                    testCoMForceTask
    )
endif()


#                      testCartesianPositionVelocityConstraint



#                      testDynamicsConstraint
                      
                      
                      
#                      testQPOases_AutoStack
#                      testQPOases_CartesianPositionConstraint

                      
#                      testQPOases_GlobalConstraints
                      
#                      testQPOases_PriorityInversion
#                      testQPOases_SelfCollisionAvoidance
#                      testQPOases_Sparse

#                      testQPOases_VelocityAllocation

    
                      
#                      testGazeVelocityTask
#                      testInteractionVelocityTask
                      
                      
                      
                      
                      
#                      testDefaultHumanoidStack
#                      testLogger

#                      testVelocityAllocation
                       
if(TRUE OR YCM_EP_DEVEL_MODE_OpenSoT)
    add_custom_command(TARGET OpenSoT POST_BUILD
                      COMMAND ${CMAKE_CTEST_COMMAND}
                      MAIN_DEPENDENCY OpenSoT
                      DEPENDS
                        ${OPENSOT_TESTS}
    )
endif()

add_definitions(-DOPENSOT_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/robots/")

SET(TestLibs OpenSoT ${GTEST_BOTH_LIBRARIES} ${qpOASES_LIBRARIES}
                     ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES}
                     ${srdfdom_advr_LIBRARIES} ${XBotInterface_LIBRARIES})
if(${fcl_FOUND})
    SET(TestLibs ${TestLibs} ${fcl_LIBRARIES})
endif()
if(${YARP_FOUND} AND ${iDynTree_FOUND} AND ${idynutils2_FOUND} AND ${ModelInterfaceIDYNUTILS_FOUND})
    SET(TestLibs ${TestLibs} ${YARP_LIBRARIES} ${ModelInterfaceIDYNUTILS_LIBRARIES}
                 ${idynutils2_LIBRARIES} ${iDynTree_LIBRARIES})
endif()
if(${trajectory_utils_FOUND} AND ${rviz_visual_tools_FOUND})
    SET(TestLibs ${TestLibs} ${trajectory_utils_LIBRARIES} ${rviz_visual_tools_LIBRARIES})
endif()
if(${test_utils_FOUND})
    SET(TestLibs ${TestLibs} ${test_utils_LIBRARIES} )
endif()

    
if(${PCL_FOUND} AND ${moveit_core_FOUND})
    ADD_EXECUTABLE(testAggregatedConstraint     constraints/TestAggregated.cpp)
    TARGET_LINK_LIBRARIES(testAggregatedConstraint ${TestLibs})
    add_dependencies(testAggregatedConstraint GTest-ext OpenSoT)
    add_test(NAME OpenSoT_constraints_Aggregated COMMAND testAggregatedConstraint)
endif()

ADD_EXECUTABLE(testBilateralConstraint     constraints/TestBilateralConstraint.cpp)
TARGET_LINK_LIBRARIES(testBilateralConstraint ${TestLibs})
add_dependencies(testBilateralConstraint GTest-ext OpenSoT)
add_test(NAME OpenSoT_constraints_BilateralConstraint COMMAND testBilateralConstraint)

ADD_EXECUTABLE(testJointLimitsVelocityBounds    constraints/velocity/TestJointLimits.cpp)
TARGET_LINK_LIBRARIES(testJointLimitsVelocityBounds ${TestLibs})
add_dependencies(testJointLimitsVelocityBounds GTest-ext OpenSoT)
add_test(NAME OpenSoT_constraints_velocity_JointLimits  COMMAND testJointLimitsVelocityBounds)

ADD_EXECUTABLE(testQPOasesSolver solvers/TestQPOases.cpp)
TARGET_LINK_LIBRARIES(testQPOasesSolver ${TestLibs})
add_dependencies(testQPOasesSolver GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_qpOases COMMAND testQPOasesSolver)

ADD_EXECUTABLE(testAffineHelper utils/TestAffineHelper.cpp)
TARGET_LINK_LIBRARIES(testAffineHelper ${TestLibs})
add_dependencies(testAffineHelper GTest-ext OpenSoT)
add_test(NAME OpenSoT_utils_affineHelper COMMAND testAffineHelper)


if(${trajectory_utils_FOUND})
    ADD_EXECUTABLE(testQPOases_StaticWalk_FloatingBase solvers/TestQPOases_StaticWalk_FloatingBase.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_StaticWalk_FloatingBase ${TestLibs})
    add_dependencies(testQPOases_StaticWalk_FloatingBase GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_StaticWalk_FloatingBase COMMAND testQPOases_StaticWalk_FloatingBase)

    ADD_EXECUTABLE(testEigenSVD_StaticWalk_FloatingBase solvers/TestEigenSVD_StaticWalk_FloatingBase.cpp)
    TARGET_LINK_LIBRARIES(testEigenSVD_StaticWalk_FloatingBase ${TestLibs})
    add_dependencies(testEigenSVD_StaticWalk_FloatingBase GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_EigenSVD_StaticWalk_FloatingBase COMMAND testEigenSVD_StaticWalk_FloatingBase)  
endif()

ADD_EXECUTABLE(testFrictionConeForceConstraint constraints/force/TestFrictionCones.cpp)
TARGET_LINK_LIBRARIES(testFrictionConeForceConstraint ${TestLibs} ${catkin_LIBRARIES})
add_dependencies(testFrictionConeForceConstraint GTest-ext OpenSoT)
add_test(NAME OpenSoT_constraint_force_FrictionCones COMMAND testFrictionConeForceConstraint)

ADD_EXECUTABLE(testManipulabilityTask tasks/velocity/TestManipulability.cpp)
TARGET_LINK_LIBRARIES(testManipulabilityTask ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testManipulabilityTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_velocity_Manipulability COMMAND testManipulabilityTask)

ADD_EXECUTABLE(testMinimizeAccelerationTask tasks/velocity/TestMinimizeAcceleration.cpp)
TARGET_LINK_LIBRARIES(testMinimizeAccelerationTask ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testMinimizeAccelerationTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_velocity_MinimizeAcceleration COMMAND testMinimizeAccelerationTask)

ADD_EXECUTABLE(testMinimumVelocityTask tasks/velocity/TestMinimumVelocity.cpp)
TARGET_LINK_LIBRARIES(testMinimumVelocityTask ${TestLibs} ${PCL_LIBRARIES})
add_dependencies(testMinimumVelocityTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_velocity_MinimumVelocity COMMAND testMinimumVelocityTask)

ADD_EXECUTABLE(testPosturalVelocityTask tasks/velocity/TestPostural.cpp)
TARGET_LINK_LIBRARIES(testPosturalVelocityTask ${TestLibs})
add_dependencies(testPosturalVelocityTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_velocity_Postural COMMAND testPosturalVelocityTask)

ADD_EXECUTABLE(testPosturalAccelerationTask tasks/acceleration/TestPostural.cpp)
TARGET_LINK_LIBRARIES(testPosturalAccelerationTask ${TestLibs})
add_dependencies(testPosturalAccelerationTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_acceleration_Postural COMMAND testPosturalAccelerationTask)

ADD_EXECUTABLE(testSubTask tasks/TestSubTask.cpp)
TARGET_LINK_LIBRARIES(testSubTask ${TestLibs})
add_dependencies(testSubTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_SubTask COMMAND testSubTask)

if(${fcl_FOUND})
    ADD_EXECUTABLE(testCollisionUtils utils/collision_utils_test.cpp)
    TARGET_LINK_LIBRARIES(testCollisionUtils ${TestLibs})
    add_dependencies(testCollisionUtils GTest-ext OpenSoT)
    add_test(NAME OpenSoT_utils_collision_utils COMMAND testCollisionUtils)
endif()

ADD_EXECUTABLE(testCartesianUtils utils/cartesian_utils_test.cpp)
TARGET_LINK_LIBRARIES(testCartesianUtils ${TestLibs})
add_dependencies(testCartesianUtils GTest-ext OpenSoT)
add_test(NAME OpenSoT_utils_cartesian_utils COMMAND testCartesianUtils)

ADD_EXECUTABLE(testQPOases_SetActiveStack solvers/TestQPOases_SetActiveStack.cpp)
TARGET_LINK_LIBRARIES(testQPOases_SetActiveStack ${TestLibs})
add_dependencies(testQPOases_SetActiveStack GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_qpOases_SetActiveStack COMMAND testQPOases_SetActiveStack)

ADD_EXECUTABLE(testMinimumEffortVelocityTask tasks/velocity/TestMinimumEffort.cpp)
TARGET_LINK_LIBRARIES(testMinimumEffortVelocityTask ${TestLibs})
add_dependencies(testMinimumEffortVelocityTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_velocity_MinimumEffort COMMAND testMinimumEffortVelocityTask)

ADD_EXECUTABLE(testVelocityLimitsVelocityBounds constraints/velocity/TestVelocityLimits.cpp)
TARGET_LINK_LIBRARIES(testVelocityLimitsVelocityBounds ${TestLibs})
add_dependencies(testVelocityLimitsVelocityBounds GTest-ext OpenSoT)
add_test(NAME OpenSoT_constraints_velocity_VelocityLimit COMMAND testVelocityLimitsVelocityBounds)

if(${YARP_FOUND})
#    ADD_EXECUTABLE(testCartesianPositionVelocityConstraint constraints/velocity/TestCartesianPositionConstraint.cpp)
#    TARGET_LINK_LIBRARIES(testCartesianPositionVelocityConstraint ${TestLibs})
#    add_dependencies(testCartesianPositionVelocityConstraint GTest-ext OpenSoT)


    ADD_EXECUTABLE(testCollisionAvoidanceVelocityConstraint constraints/velocity/TestCollisionAvoidance.cpp)
    TARGET_LINK_LIBRARIES(testCollisionAvoidanceVelocityConstraint ${TestLibs})
    add_dependencies(testCollisionAvoidanceVelocityConstraint GTest-ext OpenSoT)
    add_test(NAME OpenSoT_constraints_velocity_CollisionAvoidance COMMAND testCollisionAvoidanceVelocityConstraint)

    ADD_EXECUTABLE(testCoMVelocityVelocityConstraint constraints/velocity/TestCoMVelocity.cpp)
    TARGET_LINK_LIBRARIES(testCoMVelocityVelocityConstraint ${TestLibs})
    add_dependencies(testCoMVelocityVelocityConstraint GTest-ext OpenSoT)
    add_test(NAME OpenSoT_constraints_velocity_CoMVelocity COMMAND testCoMVelocityVelocityConstraint)

    ADD_EXECUTABLE(testConvexHullVelocityConstraint constraints/velocity/TestConvexHull.cpp)
    TARGET_LINK_LIBRARIES(testConvexHullVelocityConstraint ${TestLibs})
    add_dependencies(testConvexHullVelocityConstraint GTest-ext OpenSoT)
    add_test(NAME OpenSoT_constraints_velocity_ConvexHull COMMAND testConvexHullVelocityConstraint)

#    ADD_EXECUTABLE(testDynamicsConstraint constraints/velocity/TestDynamics.cpp)
#    TARGET_LINK_LIBRARIES(testDynamicsConstraint ${TestLibs})
#    add_dependencies(testDynamicsConstraint GTest-ext OpenSoT)


#    ADD_EXECUTABLE(testQPOases_AutoStack solvers/TestQPOases_AutoStack.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_AutoStack ${TestLibs})
#    add_dependencies(testQPOases_AutoStack GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testQPOases_CartesianPositionConstraint solvers/TestQPOases_CartesianPositionConstraint.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_CartesianPositionConstraint ${TestLibs})
#    add_dependencies(testQPOases_CartesianPositionConstraint GTest-ext OpenSoT)

    ADD_EXECUTABLE(testQPOases_ConvexHull solvers/TestQPOases_ConvexHull.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_ConvexHull ${TestLibs})
    add_dependencies(testQPOases_ConvexHull GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_ConvexHull COMMAND testQPOases_ConvexHull)

    ADD_EXECUTABLE(testQPOases_FF solvers/TestQPOases_FF.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_FF ${TestLibs})
    add_dependencies(testQPOases_FF GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_FF COMMAND testQPOases_FF)

#    ADD_EXECUTABLE(testQPOases_GlobalConstraints solvers/TestQPOases_GlobalConstraints.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_GlobalConstraints ${TestLibs} ${PCL_LIBRARIES})
#    add_dependencies(testQPOases_GlobalConstraints GTest-ext OpenSoT)

    ADD_EXECUTABLE(testQPOases_Options solvers/TestQPOases_Options.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_Options ${TestLibs})
    add_dependencies(testQPOases_Options GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_Options COMMAND testQPOases_Options)

#    ADD_EXECUTABLE(testQPOases_PriorityInversion solvers/TestQPOases_PriorityInversion.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_PriorityInversion ${TestLibs})
#    add_dependencies(testQPOases_PriorityInversion GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testQPOases_SelfCollisionAvoidance solvers/TestQPOases_SelfCollisionAvoidance.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_SelfCollisionAvoidance ${TestLibs})
#    add_dependencies(testQPOases_SelfCollisionAvoidance GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testQPOases_Sparse solvers/TestQPOases_Sparse.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_Sparse ${TestLibs})
#    add_dependencies(testQPOases_Sparse GTest-ext OpenSoT)

    ADD_EXECUTABLE(testQPOases_SubTask solvers/TestQPOases_SubTask.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_SubTask ${TestLibs})
    add_dependencies(testQPOases_SubTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_SubTask COMMAND testQPOases_SubTask)

#    ADD_EXECUTABLE(testQPOases_VelocityAllocation solvers/TestQPOases_VelocityAllocation.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_VelocityAllocation ${TestLibs})
#    add_dependencies(testQPOases_VelocityAllocation GTest-ext OpenSoT)


    ADD_EXECUTABLE(testAggregatedTask tasks/TestAggregated.cpp)
    TARGET_LINK_LIBRARIES(testAggregatedTask ${TestLibs})
    add_dependencies(testAggregatedTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_task_Aggregated COMMAND testAggregatedTask)

    ADD_EXECUTABLE(testCartesianVelocityTask tasks/velocity/TestCartesian.cpp)
    TARGET_LINK_LIBRARIES(testCartesianVelocityTask ${TestLibs})
    add_dependencies(testCartesianVelocityTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_task_velocity_Cartesian COMMAND testCartesianVelocityTask)

    ADD_EXECUTABLE(testCartesianAccelerationTask tasks/acceleration/TestCartesian.cpp)
    TARGET_LINK_LIBRARIES(testCartesianAccelerationTask ${TestLibs})
    add_dependencies(testCartesianAccelerationTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_task_acceleration_Cartesian COMMAND testCartesianAccelerationTask)

    ADD_EXECUTABLE(testCoMAccelerationTask tasks/acceleration/TestCoM.cpp)
    TARGET_LINK_LIBRARIES(testCoMAccelerationTask ${TestLibs})
    add_dependencies(testCoMAccelerationTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_task_acceleration_com COMMAND testCoMAccelerationTask)

    ADD_EXECUTABLE(testCoMForceTask tasks/force/TestCoM.cpp)
    TARGET_LINK_LIBRARIES(testCoMForceTask ${TestLibs})
    add_dependencies(testCoMForceTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_task_force_CoM COMMAND testCoMForceTask)

    ADD_EXECUTABLE(testCoMVelocityTask tasks/velocity/TestCoM.cpp)
    TARGET_LINK_LIBRARIES(testCoMVelocityTask ${TestLibs})
    add_dependencies(testCoMVelocityTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_task_velocity_CoM COMMAND testCoMVelocityTask)

#    ADD_EXECUTABLE(testGazeVelocityTask tasks/velocity/TestGaze.cpp)
#    TARGET_LINK_LIBRARIES(testGazeVelocityTask ${TestLibs})
#    add_dependencies(testGazeVelocityTask GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testInteractionVelocityTask tasks/velocity/TestInteraction.cpp)
#    TARGET_LINK_LIBRARIES(testInteractionVelocityTask ${TestLibs})
#    add_dependencies(testInteractionVelocityTask GTest-ext OpenSoT)

    ADD_EXECUTABLE(testContactFloatingBaseTask tasks/floating_base/TestContact.cpp)
    TARGET_LINK_LIBRARIES(testContactFloatingBaseTask ${TestLibs})
    add_dependencies(testContactFloatingBaseTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_task_floating_base_Postural COMMAND testContactFloatingBaseTask)

    ADD_EXECUTABLE(testAutoStack utils/TestAutoStack.cpp)
    TARGET_LINK_LIBRARIES(testAutoStack ${TestLibs})
    add_dependencies(testAutoStack GTest-ext OpenSoT)
    add_test(NAME OpenSoT_utils_AutoStack COMMAND testAutoStack)

#    ADD_EXECUTABLE(testDefaultHumanoidStack utils/TestDefaultHumanoidStack.cpp)
#    TARGET_LINK_LIBRARIES(testDefaultHumanoidStack ${TestLibs})
#    add_dependencies(testDefaultHumanoidStack GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testLogger utils/TestLogger.cpp)
#    TARGET_LINK_LIBRARIES(testLogger ${TestLibs})
#    add_dependencies(testLogger GTest-ext OpenSoT)

    #ADD_EXECUTABLE(testPreviewer utils/TestPreviewer.cpp)
    #TARGET_LINK_LIBRARIES(testPreviewer ${TestLibs})
    #add_dependencies(testPreviewer GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testVelocityAllocation utils/TestVelocityAllocation.cpp)
#    TARGET_LINK_LIBRARIES(testVelocityAllocation ${TestLibs})
#    add_dependencies(testVelocityAllocation GTest-ext OpenSoT)


    ADD_EXECUTABLE(testQPOases_StaticWalk solvers/TestQPOases_StaticWalk.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_StaticWalk ${TestLibs})
    add_dependencies(testQPOases_StaticWalk GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_StaticWalk COMMAND testQPOases_StaticWalk)

    ADD_EXECUTABLE(testBasicAlgebra solvers/TestBasicAlgebra.cpp)
    TARGET_LINK_LIBRARIES(testBasicAlgebra ${TestLibs})
    add_dependencies(testBasicAlgebra GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_BasicAlgebra COMMAND testBasicAlgebra)

    ADD_EXECUTABLE(testPiler utils/TestPiler.cpp)
    TARGET_LINK_LIBRARIES(testPiler ${TestLibs})
    add_dependencies(testPiler GTest-ext OpenSoT)
    add_test(NAME OpenSoT_utils_testPiler COMMAND testPiler)


endif()

#    if(OPENSOT_COMPILE_EXAMPLES)
#        include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../examples/KlamptController/include")
#        ADD_EXECUTABLE(testKlamptController ../examples/KlamptController/tests/testKlamptController.cpp
#                                            ../examples/KlamptController/src/KlamptController.cpp)
#        TARGET_LINK_LIBRARIES(testKlamptController ${TestLibs})
#        add_dependencies(testKlamptController GTest-ext OpenSoT)
#        add_test(NAME OpenSoT_KlamptController COMMAND testKlamptController)
#    endif()




#    add_test(NAME OpenSoT_constraints_velocity_CartesianPosition COMMAND testCartesianPositionVelocityConstraint)



#    add_test(NAME OpenSoT_constraints_velocity_Dynamics COMMAND testDynamicsConstraint)



#    add_test(NAME OpenSoT_solvers_qpOases_AutoStack COMMAND testQPOases_AutoStack)
#    add_test(NAME OpenSoT_solvers_qpOases_CartesianPositionConstraint COMMAND testQPOases_CartesianPositionConstraint)



#    add_test(NAME OpenSoT_solvers_qpOases_GlobalConstraints COMMAND testQPOases_GlobalConstraints)
    
#    ##the following tests compile but don't get executed automatically. They are not proper tests ATM, rather case-studies and references for stack tuning.
#    #add_test(NAME OpenSoT_solvers_qpOases_PriorityInversion COMMAND testQPOases_PriorityInversion)
#    add_test(NAME OpenSoT_solvers_qpOases_SelfCollisionAvoidance COMMAND testQPOases_SelfCollisionAvoidance)
#    add_test(NAME OpenSoT_solvers_qpOases_Sparse COMMAND testQPOases_Sparse)

#    add_test(NAME OpenSoT_solvers_qpOases_VelocityAllocation COMMAND testQPOases_VelocityAllocation)









#    add_test(NAME OpenSoT_task_velocity_Gaze COMMAND testGazeVelocityTask)
#    add_test(NAME OpenSoT_task_velocity_Interaction COMMAND testInteractionVelocityTask)





#    add_test(NAME OpenSoT_utils_DefaultHumanoidStack COMMAND testDefaultHumanoidStack)
#    add_test(NAME OpenSoT_utils_Logger COMMAND testLogger)
#    #add_test(NAME OpenSoT_utils_Previewer COMMAND testPreviewer)
#    add_test(NAME OpenSoT_utils_VelocityAllocation COMMAND testVelocityAllocation)





find_package(MATLAB QUIET)
find_package(PythonInterp)
if(MATLAB_FOUND AND PYTHONINTERP_FOUND AND SHOW_TEST_RESULTS)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CTestCustom.cmake ${CMAKE_CURRENT_BINARY_DIR}/../CTestCustom.cmake)
endif()

add_custom_target(copy_test_plotter_m ALL
                  ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plot_test_results.m" "${CMAKE_CURRENT_BINARY_DIR}/")

add_custom_target(copy_test_plotter_py ALL
                  ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plot_test_results.py" "${CMAKE_CURRENT_BINARY_DIR}/")
