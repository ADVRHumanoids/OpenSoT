include(ExternalProject)

find_package(trajectory_utils QUIET)
find_package(rviz_visual_tools QUIET)
find_package(catkin REQUIRED tf roscpp robot_state_publisher)
find_package(osqp QUIET)



if(${trajectory_utils_FOUND} AND ${rviz_visual_tools_FOUND})
    set(TEST_INCLUDES ${TEST_INCLUDES}
                      ${trajectory_utils_INCLUDE_DIRS}
                      ${rviz_visual_tools_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

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

include_directories(${GTEST_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

# add local include
include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()

#ALL THE FOLLOWING TESTS ARE YARP FREE
set(OPENSOT_TESTS testBilateralConstraint
                  testJointLimitsVelocityBounds
                  testJointLimitsAccelerationBounds
                  testVelocityLimitsVelocityBounds
                  testQPOasesSolver  
                  testQPOases_SetActiveStack 
                  testQPOases_Options  
                  testQPOases_SubTask
                  testQPOases_FF
                  testQPOases_AutoStack
                  testiHQP
                  testFrictionConeForceConstraint
                  testCoMVelocityVelocityConstraint
                  testCoMVelocityTask
                  testManipulabilityTask
                  testMinimizeAccelerationTask
                  testMinimumVelocityTask
                  testMinimumEffortVelocityTask
                  testPosturalVelocityTask
                  testCartesianVelocityTask
                  testCartesianAdmittanceVelocityTask
                  testSubTask
                  testCartesianUtils 
                  testBasicAlgebra 
                  testPosturalAccelerationTask
                  testCartesianAccelerationTask
                  testCoMAccelerationTask
                  testContactFloatingBaseTask
                  testCoMForceTask
                  testTask
                  testPiler
                  testKinematicEstimationFloatingBase
                  testeiQuadProgSolver
                  testKinematics
                  testl1HQP
)

if(${PCL_FOUND} AND ${moveit_core_FOUND})
    set(OPENSOT_TEST testQPOases_ConvexHull testConvexHullVelocityConstraint testAutoStack)
endif()

if(TARGET OpenSotBackEndOSQP AND ${osqp_FOUND})
    set(OPENSOT_TEST ${OPENSOT_TESTS} testOSQPSolver testGenericTask)
    set(OPENSOT_TEST ${OPENSOT_TESTS} testLP)
endif()

if(${GLPK_FOUND})
    set(OPENSOT_TEST ${OPENSOT_TESTS} testGLPKSolver)
endif()

if(${PCL_FOUND} AND ${moveit_core_FOUND})
    set(OPENSOT_TESTS ${OPENSOT_TESTS} testAggregatedConstraint
                                       testAggregatedTask)
endif()

#THESE TESTS DEPEND ON fcl
if(${FCL_FOUND})
    set(OPENSOT_TESTS ${OPENSOT_TESTS} testCollisionUtils
                                       testCollisionAvoidanceVelocityConstraint
                                       testCollisionAvoidanceEnvironmentVelocityConstraint)
endif()

#THESE TESTS DEPEND ON trajectory_utils
if(${trajectory_utils_FOUND})
    set(OPENSOT_TESTS ${OPENSOT_TESTS}
                  testQPOases_StaticWalk_FloatingBase
                  testQPOases_StaticWalk
                  testEigenSVD_StaticWalk_FloatingBase)
endif()





                      
                      
                      
#
#                      testQPOases_CartesianPositionConstraint

                      
#                      testQPOases_GlobalConstraints
                      
#                      testQPOases_PriorityInversion
#                      testQPOases_SelfCollisionAvoidance


#                      testQPOases_VelocityAllocation

    
                      
#                      testGazeVelocityTask

#                      testLogger

#                      testVelocityAllocation
                       
#if(TRUE OR YCM_EP_DEVEL_MODE_OpenSoT)
#    add_custom_command(TARGET OpenSoT POST_BUILD
#                      COMMAND ${CMAKE_CTEST_COMMAND}
#                      MAIN_DEPENDENCY OpenSoT
#                      DEPENDS
#                        ${OPENSOT_TESTS}
#    )
#endif()

add_definitions(-DOPENSOT_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/robots/")

SET(TestLibs OpenSoT ${GTEST_BOTH_LIBRARIES} ${qpOASES_LIBRARIES}
                     ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES}
                     ${srdfdom_advr_LIBRARIES} ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES})
if(${FCL_FOUND})
    SET(TestLibs ${TestLibs} ${FCL_LIBRARIES})
endif()

if(${trajectory_utils_FOUND} AND ${rviz_visual_tools_FOUND})
    SET(TestLibs ${TestLibs} ${trajectory_utils_LIBRARIES} ${rviz_visual_tools_LIBRARIES})
endif()


    
if(${PCL_FOUND} AND ${moveit_core_FOUND})
    ADD_EXECUTABLE(testAggregatedConstraint     constraints/TestAggregated.cpp)
    TARGET_LINK_LIBRARIES(testAggregatedConstraint ${TestLibs})
    add_dependencies(testAggregatedConstraint GTest-ext OpenSoT)
    add_test(NAME OpenSoT_constraints_Aggregated COMMAND testAggregatedConstraint)

    ADD_EXECUTABLE(testAggregatedTask tasks/TestAggregated.cpp)
    TARGET_LINK_LIBRARIES(testAggregatedTask ${TestLibs})
    add_dependencies(testAggregatedTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_task_Aggregated COMMAND testAggregatedTask)
endif()


ADD_EXECUTABLE(testl1HQP     solvers/Testl1HQP.cpp)
TARGET_LINK_LIBRARIES(testl1HQP ${TestLibs})
add_dependencies(testl1HQP GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_l1HQP COMMAND testl1HQP)

ADD_EXECUTABLE(testKinematicEstimationFloatingBase     floating_base/testKinematicEstimation.cpp)
TARGET_LINK_LIBRARIES(testKinematicEstimationFloatingBase ${TestLibs})
add_dependencies(testKinematicEstimationFloatingBase GTest-ext OpenSoT)
add_test(NAME OpenSoT_floating_base_KinematicEstimation COMMAND testKinematicEstimationFloatingBase)

ADD_EXECUTABLE(testTask     tasks/TestTask.cpp)
TARGET_LINK_LIBRARIES(testTask ${TestLibs})
add_dependencies(testTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_tasks_Task COMMAND testTask)

ADD_EXECUTABLE(testBilateralConstraint     constraints/TestBilateralConstraint.cpp)
TARGET_LINK_LIBRARIES(testBilateralConstraint ${TestLibs})
add_dependencies(testBilateralConstraint GTest-ext OpenSoT)
add_test(NAME OpenSoT_constraints_BilateralConstraint COMMAND testBilateralConstraint)

ADD_EXECUTABLE(testJointLimitsVelocityBounds    constraints/velocity/TestJointLimits.cpp)
TARGET_LINK_LIBRARIES(testJointLimitsVelocityBounds ${TestLibs})
add_dependencies(testJointLimitsVelocityBounds GTest-ext OpenSoT)
add_test(NAME OpenSoT_constraints_velocity_JointLimits  COMMAND testJointLimitsVelocityBounds)

ADD_EXECUTABLE(testiHQP solvers/TestiHQP.cpp)
TARGET_LINK_LIBRARIES(testiHQP ${TestLibs})
add_dependencies(testiHQP GTest-ext OpenSoT)
add_test(NAME OpenSoT_front_ends_ihqp COMMAND testiHQP)

ADD_EXECUTABLE(testQPOasesSolver solvers/TestQPOases.cpp)
TARGET_LINK_LIBRARIES(testQPOasesSolver ${TestLibs})
add_dependencies(testQPOasesSolver GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_qpOases COMMAND testQPOasesSolver)

if(TARGET OpenSotBackEndOSQP AND ${osqp_FOUND})
    ADD_EXECUTABLE(testOSQPSolver solvers/TestOSQP.cpp)
    TARGET_LINK_LIBRARIES(testOSQPSolver ${TestLibs})
    add_dependencies(testOSQPSolver GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_osqp COMMAND testOSQPSolver)

    ADD_EXECUTABLE(testGenericTask     tasks/TestGenericTask.cpp)
    TARGET_LINK_LIBRARIES(testGenericTask ${TestLibs})
    add_dependencies(testGenericTask GTest-ext OpenSoT)
    add_test(NAME OpenSoT_tasks_GenericTask COMMAND testGenericTask)


    ADD_EXECUTABLE(testLP solvers/TestLP.cpp)
    TARGET_LINK_LIBRARIES(testLP ${TestLibs})
    add_dependencies(testLP GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_linear_programming COMMAND testLP)

endif()

if(${GLPK_FOUND})
    ADD_EXECUTABLE(testGLPKSolver solvers/TestGLPK.cpp)
    TARGET_LINK_LIBRARIES(testGLPKSolver ${TestLibs})
    add_dependencies(testGLPKSolver GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_glpk COMMAND testGLPKSolver)
endif()

ADD_EXECUTABLE(testeiQuadProgSolver solvers/TesteiQuadProg.cpp)
TARGET_LINK_LIBRARIES(testeiQuadProgSolver ${TestLibs})
add_dependencies(testeiQuadProgSolver GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_eiquadprog COMMAND testeiQuadProgSolver)

ADD_EXECUTABLE(testAffineHelper utils/TestAffineHelper.cpp)
TARGET_LINK_LIBRARIES(testAffineHelper ${TestLibs})
add_dependencies(testAffineHelper GTest-ext OpenSoT)
add_test(NAME OpenSoT_utils_affineHelper COMMAND testAffineHelper)


if(${trajectory_utils_FOUND})
    ADD_EXECUTABLE(testQPOases_StaticWalk_FloatingBase solvers/TestQPOases_StaticWalk_FloatingBase.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_StaticWalk_FloatingBase ${TestLibs})
    add_dependencies(testQPOases_StaticWalk_FloatingBase GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_StaticWalk_FloatingBase COMMAND testQPOases_StaticWalk_FloatingBase)

    ADD_EXECUTABLE(testQPOases_StaticWalk solvers/TestQPOases_StaticWalk.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_StaticWalk ${TestLibs})
    add_dependencies(testQPOases_StaticWalk GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_StaticWalk COMMAND testQPOases_StaticWalk)

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

if(${FCL_FOUND})
    ADD_EXECUTABLE(testCollisionUtils utils/collision_utils_test.cpp)
    TARGET_LINK_LIBRARIES(testCollisionUtils ${TestLibs})
    add_dependencies(testCollisionUtils GTest-ext OpenSoT)
    add_test(NAME OpenSoT_utils_collision_utils COMMAND testCollisionUtils)

    ADD_EXECUTABLE(testCollisionAvoidanceVelocityConstraint constraints/velocity/TestCollisionAvoidance.cpp)
    TARGET_LINK_LIBRARIES(testCollisionAvoidanceVelocityConstraint ${TestLibs})
    add_dependencies(testCollisionAvoidanceVelocityConstraint GTest-ext OpenSoT)
    add_test(NAME OpenSoT_constraints_velocity_CollisionAvoidance COMMAND testCollisionAvoidanceVelocityConstraint)

    ADD_EXECUTABLE(testCollisionAvoidanceEnvironmentVelocityConstraint constraints/velocity/TestCollisionAvoidanceEnvironment.cpp)
    TARGET_LINK_LIBRARIES(testCollisionAvoidanceEnvironmentVelocityConstraint ${TestLibs})
    add_dependencies(testCollisionAvoidanceEnvironmentVelocityConstraint GTest-ext OpenSoT)
    add_test(NAME OpenSoT_constraints_velocity_CollisionAvoidanceEnvironment COMMAND testCollisionAvoidanceEnvironmentVelocityConstraint)
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

ADD_EXECUTABLE(testJointLimitsAccelerationBounds constraints/acceleration/TestJointLimits.cpp)
TARGET_LINK_LIBRARIES(testJointLimitsAccelerationBounds ${TestLibs})
add_dependencies(testJointLimitsAccelerationBounds GTest-ext OpenSoT)
add_test(NAME OpenSoT_constraints_acceleration_JointLimits COMMAND testJointLimitsAccelerationBounds)

ADD_EXECUTABLE(testQPOases_Options solvers/TestQPOases_Options.cpp)
TARGET_LINK_LIBRARIES(testQPOases_Options ${TestLibs})
add_dependencies(testQPOases_Options GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_qpOases_Options COMMAND testQPOases_Options)

ADD_EXECUTABLE(testQPOases_SubTask solvers/TestQPOases_SubTask.cpp)
TARGET_LINK_LIBRARIES(testQPOases_SubTask ${TestLibs})
add_dependencies(testQPOases_SubTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_qpOases_SubTask COMMAND testQPOases_SubTask)

ADD_EXECUTABLE(testCoMVelocityTask tasks/velocity/TestCoM.cpp)
TARGET_LINK_LIBRARIES(testCoMVelocityTask ${TestLibs})
add_dependencies(testCoMVelocityTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_velocity_CoM COMMAND testCoMVelocityTask)

ADD_EXECUTABLE(testAutoStack utils/TestAutoStack.cpp)
TARGET_LINK_LIBRARIES(testAutoStack ${TestLibs})
add_dependencies(testAutoStack GTest-ext OpenSoT)
add_test(NAME OpenSoT_utils_AutoStack COMMAND testAutoStack)

ADD_EXECUTABLE(testBasicAlgebra solvers/TestBasicAlgebra.cpp)
TARGET_LINK_LIBRARIES(testBasicAlgebra ${TestLibs})
add_dependencies(testBasicAlgebra GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_BasicAlgebra COMMAND testBasicAlgebra)

ADD_EXECUTABLE(testCartesianAccelerationTask tasks/acceleration/TestCartesian.cpp)
TARGET_LINK_LIBRARIES(testCartesianAccelerationTask ${TestLibs})
add_dependencies(testCartesianAccelerationTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_acceleration_Cartesian COMMAND testCartesianAccelerationTask)

ADD_EXECUTABLE(testCoMAccelerationTask tasks/acceleration/TestCoM.cpp)
TARGET_LINK_LIBRARIES(testCoMAccelerationTask ${TestLibs})
add_dependencies(testCoMAccelerationTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_acceleration_com COMMAND testCoMAccelerationTask)

ADD_EXECUTABLE(testContactFloatingBaseTask tasks/floating_base/TestContact.cpp)
TARGET_LINK_LIBRARIES(testContactFloatingBaseTask ${TestLibs})
add_dependencies(testContactFloatingBaseTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_floating_base_Postural COMMAND testContactFloatingBaseTask)

ADD_EXECUTABLE(testCartesianVelocityTask tasks/velocity/TestCartesian.cpp)
TARGET_LINK_LIBRARIES(testCartesianVelocityTask ${TestLibs})
add_dependencies(testCartesianVelocityTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_velocity_Cartesian COMMAND testCartesianVelocityTask)

ADD_EXECUTABLE(testCartesianAdmittanceVelocityTask tasks/velocity/TestCartesianAdmittance.cpp)
TARGET_LINK_LIBRARIES(testCartesianAdmittanceVelocityTask ${TestLibs})
add_dependencies(testCartesianAdmittanceVelocityTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_velocity_Cartesian_admittance COMMAND testCartesianAdmittanceVelocityTask)

ADD_EXECUTABLE(testCoMVelocityVelocityConstraint constraints/velocity/TestCoMVelocity.cpp)
TARGET_LINK_LIBRARIES(testCoMVelocityVelocityConstraint ${TestLibs})
add_dependencies(testCoMVelocityVelocityConstraint GTest-ext OpenSoT)
add_test(NAME OpenSoT_constraints_velocity_CoMVelocity COMMAND testCoMVelocityVelocityConstraint)

ADD_EXECUTABLE(testPiler utils/TestPiler.cpp)
TARGET_LINK_LIBRARIES(testPiler ${TestLibs})
add_dependencies(testPiler GTest-ext OpenSoT)
add_test(NAME OpenSoT_utils_testPiler COMMAND testPiler)

ADD_EXECUTABLE(testQPOases_FF solvers/TestQPOases_FF.cpp)
TARGET_LINK_LIBRARIES(testQPOases_FF ${TestLibs})
add_dependencies(testQPOases_FF GTest-ext OpenSoT)
add_test(NAME OpenSoT_solvers_qpOases_FF COMMAND testQPOases_FF)

if(${PCL_FOUND} AND ${moveit_core_FOUND})
    ADD_EXECUTABLE(testConvexHullVelocityConstraint constraints/velocity/TestConvexHull.cpp)
    TARGET_LINK_LIBRARIES(testConvexHullVelocityConstraint ${TestLibs})
    add_dependencies(testConvexHullVelocityConstraint GTest-ext OpenSoT)
    add_test(NAME OpenSoT_constraints_velocity_ConvexHull COMMAND testConvexHullVelocityConstraint)

    ADD_EXECUTABLE(testQPOases_ConvexHull solvers/TestQPOases_ConvexHull.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_ConvexHull ${TestLibs})
    add_dependencies(testQPOases_ConvexHull GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_ConvexHull COMMAND testQPOases_ConvexHull)

    ADD_EXECUTABLE(testQPOases_AutoStack solvers/TestQPOases_AutoStack.cpp)
    TARGET_LINK_LIBRARIES(testQPOases_AutoStack ${TestLibs})
    add_dependencies(testQPOases_AutoStack GTest-ext OpenSoT)
    add_test(NAME OpenSoT_solvers_qpOases_AutoStack COMMAND testQPOases_AutoStack)
endif()

ADD_EXECUTABLE(testCoMForceTask tasks/force/TestCoM.cpp)
TARGET_LINK_LIBRARIES(testCoMForceTask ${TestLibs})
add_dependencies(testCoMForceTask GTest-ext OpenSoT)
add_test(NAME OpenSoT_task_force_CoM COMMAND testCoMForceTask)

ADD_EXECUTABLE(testKinematics utils/TestKinematics.cpp)
TARGET_LINK_LIBRARIES(testKinematics ${TestLibs})
add_dependencies(testKinematics GTest-ext OpenSoT)
add_test(NAME OpenSoT_utils_test_kinematics COMMAND testKinematics)

#ADD_EXECUTABLE(testCartesianPositionVelocityConstraint constraints/velocity/TestCartesianPositionConstraint.cpp)
#TARGET_LINK_LIBRARIES(testCartesianPositionVelocityConstraint ${TestLibs})
#add_dependencies(testCartesianPositionVelocityConstraint GTest-ext OpenSoT)
#add_test(NAME OpenSoT_constraints_velocity_CartesianPosition COMMAND testCartesianPositionVelocityConstraint)



#    ADD_EXECUTABLE(testQPOases_CartesianPositionConstraint solvers/TestQPOases_CartesianPositionConstraint.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_CartesianPositionConstraint ${TestLibs})
#    add_dependencies(testQPOases_CartesianPositionConstraint GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testQPOases_GlobalConstraints solvers/TestQPOases_GlobalConstraints.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_GlobalConstraints ${TestLibs} ${PCL_LIBRARIES})
#    add_dependencies(testQPOases_GlobalConstraints GTest-ext OpenSoT)



#    ADD_EXECUTABLE(testQPOases_PriorityInversion solvers/TestQPOases_PriorityInversion.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_PriorityInversion ${TestLibs})
#    add_dependencies(testQPOases_PriorityInversion GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testQPOases_SelfCollisionAvoidance solvers/TestQPOases_SelfCollisionAvoidance.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_SelfCollisionAvoidance ${TestLibs})
#    add_dependencies(testQPOases_SelfCollisionAvoidance GTest-ext OpenSoT)

#    ADD_EXECUTABLE(testQPOases_VelocityAllocation solvers/TestQPOases_VelocityAllocation.cpp)
#    TARGET_LINK_LIBRARIES(testQPOases_VelocityAllocation ${TestLibs})
#    add_dependencies(testQPOases_VelocityAllocation GTest-ext OpenSoT)



#    ADD_EXECUTABLE(testGazeVelocityTask tasks/velocity/TestGaze.cpp)
#    TARGET_LINK_LIBRARIES(testGazeVelocityTask ${TestLibs})
#    add_dependencies(testGazeVelocityTask GTest-ext OpenSoT)


#    ADD_EXECUTABLE(testVelocityAllocation utils/TestVelocityAllocation.cpp)
#    TARGET_LINK_LIBRARIES(testVelocityAllocation ${TestLibs})
#    add_dependencies(testVelocityAllocation GTest-ext OpenSoT)



#
#    add_test(NAME OpenSoT_solvers_qpOases_CartesianPositionConstraint COMMAND testQPOases_CartesianPositionConstraint)

#    add_test(NAME OpenSoT_solvers_qpOases_GlobalConstraints COMMAND testQPOases_GlobalConstraints)
    
#    ##the following tests compile but don't get executed automatically. They are not proper tests ATM, rather case-studies and references for stack tuning.
#    #add_test(NAME OpenSoT_solvers_qpOases_PriorityInversion COMMAND testQPOases_PriorityInversion)
#    add_test(NAME OpenSoT_solvers_qpOases_SelfCollisionAvoidance COMMAND testQPOases_SelfCollisionAvoidance)
#    add_test(NAME OpenSoT_solvers_qpOases_Sparse COMMAND testQPOases_Sparse)

#    add_test(NAME OpenSoT_solvers_qpOases_VelocityAllocation COMMAND testQPOases_VelocityAllocation)


#    add_test(NAME OpenSoT_task_velocity_Gaze COMMAND testGazeVelocityTask)


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
