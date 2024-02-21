include(ExternalProject)

set(CMAKE_CXX_STANDARD 20)

find_package(rviz_visual_tools QUIET)
find_package(tf)
find_package(eigen_conversions)
find_package(roscpp )
find_package(robot_state_publisher )
find_package(urdf )
find_package(srdfdom)
find_package(osqp QUIET)
find_package(qpSWIFT QUIET)


set(catkin_LIBRARIES
    ${tf_LIBRARIES} ${roscpp_LIBRARIES} ${robot_state_publisher_LIBRARIES} ${urdf_LIBRARIES} ${eigen_conversions_LIBRARIES})

add_definitions(-DOPENSOT_TEST_PATH="${CMAKE_CURRENT_SOURCE_DIR}/")
add_definitions(-DOPENSOT_TEST_MODEL_TYPE="pin")
message(STATUS "DOPENSOT_TEST_PATH is ${CMAKE_CURRENT_SOURCE_DIR}")

if(${rviz_visual_tools_FOUND})
    set(TEST_INCLUDES ${TEST_INCLUDES} ${GTEST_INCLUDE_DIRS}
                      ${rviz_visual_tools_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

option(SHOW_TEST_RESULTS "After running tests, show result plots" FALSE)

# compite GTest
find_package(GTest)

include_directories(${GTEST_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

# add local include
include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()


add_definitions(-DOPENSOT_TESTS_ROBOTS_DIR="${CMAKE_CURRENT_BINARY_DIR}/robots/")

SET(TestLibs OpenSoT ${GTEST_BOTH_LIBRARIES} ${qpOASES_LIBRARIES}
                     ${kdl_codyco_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES}
                     ${srdfdom_advr_LIBRARIES} ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES})
if(${FCL_FOUND})
    SET(TestLibs ${TestLibs} ${FCL_LIBRARIES})
endif()

if(${rviz_visual_tools_FOUND})
    SET(TestLibs ${TestLibs} ${rviz_visual_tools_LIBRARIES})
endif()

if(${OPENSOT_SOTH_FRONT_END})
    SET(TestLibs ${TestLibs} hcod_wrapper soth)
endif()

ADD_EXECUTABLE(testTask     tasks/TestTask.cpp)
TARGET_LINK_LIBRARIES(testTask ${TestLibs})
add_dependencies(testTask   OpenSoT)
add_test(NAME OpenSoT_tasks_Task COMMAND testTask)

    
if(${PCL_FOUND} AND ${moveit_core_FOUND})
     ADD_EXECUTABLE(testAggregatedConstraint     constraints/TestAggregated.cpp)
     TARGET_LINK_LIBRARIES(testAggregatedConstraint ${TestLibs})
     add_dependencies(testAggregatedConstraint   OpenSoT)
     add_test(NAME OpenSoT_constraints_Aggregated COMMAND testAggregatedConstraint)

     ADD_EXECUTABLE(testAggregatedTask tasks/TestAggregated.cpp)
     TARGET_LINK_LIBRARIES(testAggregatedTask ${TestLibs})
     add_dependencies(testAggregatedTask   OpenSoT)
     add_test(NAME OpenSoT_task_Aggregated COMMAND testAggregatedTask)
endif()


ADD_EXECUTABLE(testl1HQP     solvers/Testl1HQP.cpp)
TARGET_LINK_LIBRARIES(testl1HQP ${TestLibs})
add_dependencies(testl1HQP   OpenSoT)
add_test(NAME OpenSoT_solvers_l1HQP COMMAND testl1HQP)

if(${OPENSOT_SOTH_FRONT_END})
    ADD_EXECUTABLE(testSOTH     solvers/TestSOTH.cpp)
    TARGET_LINK_LIBRARIES(testSOTH ${TestLibs})
    add_dependencies(testSOTH OpenSoT)
    add_test(NAME OpenSoT_solvers_soth COMMAND testSOTH)
endif()

ADD_EXECUTABLE(testBilateralConstraint     constraints/TestBilateralConstraint.cpp)
TARGET_LINK_LIBRARIES(testBilateralConstraint ${TestLibs})
add_dependencies(testBilateralConstraint   OpenSoT)
add_test(NAME OpenSoT_constraints_BilateralConstraint COMMAND testBilateralConstraint)

ADD_EXECUTABLE(testJointLimitsVelocityBounds    constraints/velocity/TestJointLimits.cpp)
TARGET_LINK_LIBRARIES(testJointLimitsVelocityBounds ${TestLibs})
add_dependencies(testJointLimitsVelocityBounds   OpenSoT)
add_test(NAME OpenSoT_constraints_velocity_JointLimits  COMMAND testJointLimitsVelocityBounds)

#ADD_EXECUTABLE(testJointLimitsInvarianceVelocityBounds    constraints/velocity/TestJointLimitsInvariance.cpp)
#TARGET_LINK_LIBRARIES(testJointLimitsInvarianceVelocityBounds ${TestLibs})
#add_dependencies(testJointLimitsInvarianceVelocityBounds   OpenSoT)
#add_test(NAME OpenSoT_constraints_velocity_JointLimitsInvariance  COMMAND testJointLimitsInvarianceVelocityBounds)

ADD_EXECUTABLE(testiHQP solvers/TestiHQP.cpp)
TARGET_LINK_LIBRARIES(testiHQP ${TestLibs})
add_dependencies(testiHQP   OpenSoT)
add_test(NAME OpenSoT_front_ends_ihqp COMMAND testiHQP)

ADD_EXECUTABLE(testQPOasesSolver solvers/TestQPOases.cpp)
TARGET_LINK_LIBRARIES(testQPOasesSolver ${TestLibs})
add_dependencies(testQPOasesSolver   OpenSoT)
add_test(NAME OpenSoT_solvers_qpOases COMMAND testQPOasesSolver)

if(TARGET OpenSotBackEndOSQP AND ${osqp_FOUND})
    ADD_EXECUTABLE(testOSQPSolver solvers/TestOSQP.cpp)
    TARGET_LINK_LIBRARIES(testOSQPSolver ${TestLibs} osqp::osqpstatic)
    add_dependencies(testOSQPSolver   OpenSoT)
    add_test(NAME OpenSoT_solvers_osqp COMMAND testOSQPSolver)

    ADD_EXECUTABLE(testGenericTask     tasks/TestGenericTask.cpp)
    TARGET_LINK_LIBRARIES(testGenericTask ${TestLibs})
    add_dependencies(testGenericTask   OpenSoT)
    add_test(NAME OpenSoT_tasks_GenericTask COMMAND testGenericTask)


    ADD_EXECUTABLE(testLP solvers/TestLP.cpp)
    TARGET_LINK_LIBRARIES(testLP ${TestLibs} osqp::osqpstatic)
    add_dependencies(testLP   OpenSoT)
    add_test(NAME OpenSoT_solvers_linear_programming COMMAND testLP)

endif()

 if(${GLPK_FOUND})
     ADD_EXECUTABLE(testGLPKSolver solvers/TestGLPK.cpp)
     TARGET_LINK_LIBRARIES(testGLPKSolver ${TestLibs})
     add_dependencies(testGLPKSolver   OpenSoT)
     add_test(NAME OpenSoT_solvers_glpk COMMAND testGLPKSolver)
 endif()

 ADD_EXECUTABLE(testeiQuadProgSolver solvers/TesteiQuadProg.cpp)
 TARGET_LINK_LIBRARIES(testeiQuadProgSolver ${TestLibs})
 add_dependencies(testeiQuadProgSolver   OpenSoT)
 add_test(NAME OpenSoT_solvers_eiquadprog COMMAND testeiQuadProgSolver)

ADD_EXECUTABLE(testAffineUtils utils/TestAffineUtils.cpp)
TARGET_LINK_LIBRARIES(testAffineUtils ${TestLibs})
add_dependencies(testAffineUtils   OpenSoT)
add_test(NAME OpenSoT_affine_utils COMMAND testAffineUtils)

ADD_EXECUTABLE(testAffineHelper utils/TestAffineHelper.cpp)
TARGET_LINK_LIBRARIES(testAffineHelper ${TestLibs})
add_dependencies(testAffineHelper   OpenSoT)
add_test(NAME OpenSoT_utils_affineHelper COMMAND testAffineHelper)

# if(${rviz_visual_tools_FOUND})
#     ADD_EXECUTABLE(testEigenSVD_StaticWalk_FloatingBase solvers/TestEigenSVD_StaticWalk_FloatingBase.cpp trajectory_utils.cpp path_circle_fix.cpp ros_trj_publisher.cpp)
#     TARGET_LINK_LIBRARIES(testEigenSVD_StaticWalk_FloatingBase ${TestLibs})
#     add_dependencies(testEigenSVD_StaticWalk_FloatingBase   OpenSoT)
#     add_test(NAME OpenSoT_solvers_EigenSVD_StaticWalk_FloatingBase COMMAND testEigenSVD_StaticWalk_FloatingBase)
# endif()

 ADD_EXECUTABLE(testFrictionConeForceConstraint constraints/force/TestFrictionCones.cpp)
 TARGET_LINK_LIBRARIES(testFrictionConeForceConstraint ${TestLibs} ${catkin_LIBRARIES})
 add_dependencies(testFrictionConeForceConstraint   OpenSoT)
 add_test(NAME OpenSoT_constraint_force_FrictionCones COMMAND testFrictionConeForceConstraint)

# ADD_EXECUTABLE(testManipulabilityTask tasks/velocity/TestManipulability.cpp)
# TARGET_LINK_LIBRARIES(testManipulabilityTask ${TestLibs} ${QHULL_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(testManipulabilityTask   OpenSoT)
# add_test(NAME OpenSoT_task_velocity_Manipulability COMMAND testManipulabilityTask)

ADD_EXECUTABLE(testPosturalVelocityTask tasks/velocity/TestPostural.cpp)
TARGET_LINK_LIBRARIES(testPosturalVelocityTask ${TestLibs})
add_dependencies(testPosturalVelocityTask   OpenSoT)
add_test(NAME OpenSoT_task_velocity_Postural COMMAND testPosturalVelocityTask)

# ADD_EXECUTABLE(testPosturalAccelerationTask tasks/acceleration/TestPostural.cpp)
# TARGET_LINK_LIBRARIES(testPosturalAccelerationTask ${TestLibs})
# add_dependencies(testPosturalAccelerationTask   OpenSoT)
# add_test(NAME OpenSoT_task_acceleration_Postural COMMAND testPosturalAccelerationTask)

 ADD_EXECUTABLE(testSubTask tasks/TestSubTask.cpp)
 TARGET_LINK_LIBRARIES(testSubTask ${TestLibs})
 add_dependencies(testSubTask   OpenSoT)
 add_test(NAME OpenSoT_task_SubTask COMMAND testSubTask)

 ADD_EXECUTABLE(testSubConstraint constraints/TestSubConstraint.cpp)
 TARGET_LINK_LIBRARIES(testSubConstraint ${TestLibs})
 add_dependencies(testSubConstraint   OpenSoT)
 add_test(NAME OpenSoT_constraint_SubConstraint COMMAND testSubConstraint)

# if(${FCL_FOUND})
#     ADD_EXECUTABLE(testCollisionUtils utils/collision_utils_test.cpp)
#     TARGET_LINK_LIBRARIES(testCollisionUtils ${TestLibs} -lccd)
#     add_dependencies(testCollisionUtils   OpenSoT)
#     add_test(NAME OpenSoT_utils_collision_utils COMMAND testCollisionUtils)

#     ADD_EXECUTABLE(testCollisionAvoidanceVelocityConstraint constraints/velocity/TestCollisionAvoidance.cpp)
#     TARGET_LINK_LIBRARIES(testCollisionAvoidanceVelocityConstraint ${TestLibs} -lccd)
#     add_dependencies(testCollisionAvoidanceVelocityConstraint   OpenSoT)
#     add_test(NAME OpenSoT_constraints_velocity_CollisionAvoidance COMMAND testCollisionAvoidanceVelocityConstraint)

#     ADD_EXECUTABLE(testCollisionAvoidanceEnvironmentVelocityConstraint constraints/velocity/TestCollisionAvoidanceEnvironment.cpp)
#     TARGET_LINK_LIBRARIES(testCollisionAvoidanceEnvironmentVelocityConstraint ${TestLibs} -lccd)
#     add_dependencies(testCollisionAvoidanceEnvironmentVelocityConstraint   OpenSoT)
#     add_test(NAME OpenSoT_constraints_velocity_CollisionAvoidanceEnvironment COMMAND testCollisionAvoidanceEnvironmentVelocityConstraint)
# endif()

 ADD_EXECUTABLE(testCartesianUtils utils/cartesian_utils_test.cpp)
 TARGET_LINK_LIBRARIES(testCartesianUtils ${TestLibs})
 add_dependencies(testCartesianUtils   OpenSoT)
 add_test(NAME OpenSoT_utils_cartesian_utils COMMAND testCartesianUtils)

 ADD_EXECUTABLE(testQPOases_SetActiveStack solvers/TestQPOases_SetActiveStack.cpp)
 TARGET_LINK_LIBRARIES(testQPOases_SetActiveStack ${TestLibs})
 add_dependencies(testQPOases_SetActiveStack   OpenSoT)
 add_test(NAME OpenSoT_solvers_qpOases_SetActiveStack COMMAND testQPOases_SetActiveStack)

 ADD_EXECUTABLE(testMinimumEffortVelocityTask tasks/velocity/TestMinimumEffort.cpp)
 TARGET_LINK_LIBRARIES(testMinimumEffortVelocityTask ${TestLibs})
 add_dependencies(testMinimumEffortVelocityTask   OpenSoT)
 add_test(NAME OpenSoT_task_velocity_MinimumEffort COMMAND testMinimumEffortVelocityTask)

 ADD_EXECUTABLE(testVelocityLimitsVelocityBounds constraints/velocity/TestVelocityLimits.cpp)
 TARGET_LINK_LIBRARIES(testVelocityLimitsVelocityBounds ${TestLibs})
 add_dependencies(testVelocityLimitsVelocityBounds   OpenSoT)
 add_test(NAME OpenSoT_constraints_velocity_VelocityLimit COMMAND testVelocityLimitsVelocityBounds)

# ADD_EXECUTABLE(testJointLimitsAccelerationBounds constraints/acceleration/TestJointLimits.cpp)
# TARGET_LINK_LIBRARIES(testJointLimitsAccelerationBounds ${TestLibs})
# add_dependencies(testJointLimitsAccelerationBounds   OpenSoT)
# add_test(NAME OpenSoT_constraints_acceleration_JointLimits COMMAND testJointLimitsAccelerationBounds)

# ADD_EXECUTABLE(testJointLimitsAccelerationViabilityBounds constraints/acceleration/TestJointLimitsViability.cpp)
# TARGET_LINK_LIBRARIES(testJointLimitsAccelerationViabilityBounds ${TestLibs})
# add_dependencies(testJointLimitsAccelerationViabilityBounds   OpenSoT)
# add_test(NAME OpenSoT_constraints_acceleration_viability_JointLimits COMMAND testJointLimitsAccelerationViabilityBounds)

# ADD_EXECUTABLE(testJointLimitsAccelerationECBFBounds constraints/acceleration/TestJointLimitsECBF.cpp)
# TARGET_LINK_LIBRARIES(testJointLimitsAccelerationECBFBounds ${TestLibs})
# add_dependencies(testJointLimitsAccelerationECBFBounds   OpenSoT)
# add_test(NAME OpenSoT_constraints_acceleration_ecbf_JointLimits COMMAND testJointLimitsAccelerationECBFBounds)

 ADD_EXECUTABLE(testQPOases_Options solvers/TestQPOases_Options.cpp)
 TARGET_LINK_LIBRARIES(testQPOases_Options ${TestLibs})
 add_dependencies(testQPOases_Options   OpenSoT)
 add_test(NAME OpenSoT_solvers_qpOases_Options COMMAND testQPOases_Options)

 ADD_EXECUTABLE(testQPOases_SubTask solvers/TestQPOases_SubTask.cpp DefaultHumanoidStack.cpp)
 TARGET_LINK_LIBRARIES(testQPOases_SubTask ${TestLibs})
 add_dependencies(testQPOases_SubTask   OpenSoT)
 add_test(NAME OpenSoT_solvers_qpOases_SubTask COMMAND testQPOases_SubTask)

 ADD_EXECUTABLE(testCoMVelocityTask tasks/velocity/TestCoM.cpp)
 TARGET_LINK_LIBRARIES(testCoMVelocityTask ${TestLibs})
 add_dependencies(testCoMVelocityTask   OpenSoT)
 add_test(NAME OpenSoT_task_velocity_CoM COMMAND testCoMVelocityTask)

 ADD_EXECUTABLE(testAutoStack utils/TestAutoStack.cpp DefaultHumanoidStack.cpp)
 TARGET_LINK_LIBRARIES(testAutoStack ${TestLibs})
 add_dependencies(testAutoStack   OpenSoT)
 add_test(NAME OpenSoT_utils_AutoStack COMMAND testAutoStack)

 ADD_EXECUTABLE(testBasicAlgebra solvers/TestBasicAlgebra.cpp)
 TARGET_LINK_LIBRARIES(testBasicAlgebra ${TestLibs})
 add_dependencies(testBasicAlgebra   OpenSoT)
 add_test(NAME OpenSoT_solvers_BasicAlgebra COMMAND testBasicAlgebra)

 ADD_EXECUTABLE(testCartesianAccelerationTask tasks/acceleration/TestCartesian.cpp)
 TARGET_LINK_LIBRARIES(testCartesianAccelerationTask ${TestLibs})
 add_dependencies(testCartesianAccelerationTask   OpenSoT)
 add_test(NAME OpenSoT_task_acceleration_Cartesian COMMAND testCartesianAccelerationTask)

 ADD_EXECUTABLE(testCoMAccelerationTask tasks/acceleration/TestCoM.cpp)
 TARGET_LINK_LIBRARIES(testCoMAccelerationTask ${TestLibs})
 add_dependencies(testCoMAccelerationTask   OpenSoT)
 add_test(NAME OpenSoT_task_acceleration_com COMMAND testCoMAccelerationTask)

 ADD_EXECUTABLE(testContactFloatingBaseTask tasks/floating_base/TestContact.cpp)
 TARGET_LINK_LIBRARIES(testContactFloatingBaseTask ${TestLibs})
 add_dependencies(testContactFloatingBaseTask   OpenSoT)
 add_test(NAME OpenSoT_task_floating_base_Postural COMMAND testContactFloatingBaseTask)

 ADD_EXECUTABLE(testCartesianVelocityTask tasks/velocity/TestCartesian.cpp) #<-- missing testCartesianTask.testActiveJointsMask !
 TARGET_LINK_LIBRARIES(testCartesianVelocityTask ${TestLibs})
 add_dependencies(testCartesianVelocityTask   OpenSoT)
 add_test(NAME OpenSoT_task_velocity_Cartesian COMMAND testCartesianVelocityTask)

# ADD_EXECUTABLE(testCartesianAdmittanceVelocityTask tasks/velocity/TestCartesianAdmittance.cpp)
# TARGET_LINK_LIBRARIES(testCartesianAdmittanceVelocityTask ${TestLibs})
# add_dependencies(testCartesianAdmittanceVelocityTask   OpenSoT)
# add_test(NAME OpenSoT_task_velocity_Cartesian_admittance COMMAND testCartesianAdmittanceVelocityTask)

 ADD_EXECUTABLE(testCoMVelocityVelocityConstraint constraints/velocity/TestCoMVelocity.cpp)
 TARGET_LINK_LIBRARIES(testCoMVelocityVelocityConstraint ${TestLibs})
 add_dependencies(testCoMVelocityVelocityConstraint   OpenSoT)
 add_test(NAME OpenSoT_constraints_velocity_CoMVelocity COMMAND testCoMVelocityVelocityConstraint)

 ADD_EXECUTABLE(testPiler utils/TestPiler.cpp)
 TARGET_LINK_LIBRARIES(testPiler ${TestLibs})
 add_dependencies(testPiler   OpenSoT)
 add_test(NAME OpenSoT_utils_testPiler COMMAND testPiler)

# ADD_EXECUTABLE(testQPOases_FF solvers/TestQPOases_FF.cpp)
# TARGET_LINK_LIBRARIES(testQPOases_FF ${TestLibs})
# add_dependencies(testQPOases_FF   OpenSoT)
# add_test(NAME OpenSoT_solvers_qpOases_FF COMMAND testQPOases_FF)

# if(${PCL_FOUND} AND ${moveit_core_FOUND})
#     ADD_EXECUTABLE(testConvexHullVelocityConstraint constraints/velocity/TestConvexHull.cpp)
#     TARGET_LINK_LIBRARIES(testConvexHullVelocityConstraint ${TestLibs})
#     add_dependencies(testConvexHullVelocityConstraint   OpenSoT)
#     add_test(NAME OpenSoT_constraints_velocity_ConvexHull COMMAND testConvexHullVelocityConstraint)

#     ADD_EXECUTABLE(testQPOases_ConvexHull solvers/TestQPOases_ConvexHull.cpp)
#     TARGET_LINK_LIBRARIES(testQPOases_ConvexHull ${TestLibs})
#     add_dependencies(testQPOases_ConvexHull   OpenSoT)
#     add_test(NAME OpenSoT_solvers_qpOases_ConvexHull COMMAND testQPOases_ConvexHull)

#     ADD_EXECUTABLE(testQPOases_AutoStack solvers/TestQPOases_AutoStack.cpp DefaultHumanoidStack.cpp)
#     TARGET_LINK_LIBRARIES(testQPOases_AutoStack ${TestLibs})
#     add_dependencies(testQPOases_AutoStack   OpenSoT)
#     add_test(NAME OpenSoT_solvers_qpOases_AutoStack COMMAND testQPOases_AutoStack)
# endif()

 ADD_EXECUTABLE(testCoMForceTask tasks/force/TestCoM.cpp)
 TARGET_LINK_LIBRARIES(testCoMForceTask ${TestLibs})
 add_dependencies(testCoMForceTask  OpenSoT)
 add_test(NAME OpenSoT_task_force_CoM COMMAND testCoMForceTask)

 if(${qpSWIFT_FOUND})
     ADD_EXECUTABLE(testqpSWIFTSolver solvers/TestqpSWIFT.cpp)
     TARGET_LINK_LIBRARIES(testqpSWIFTSolver ${TestLibs} qpSWIFT::qpSWIFT-static)
     add_dependencies(testqpSWIFTSolver OpenSoT)
     add_test(NAME OpenSoT_solvers_qpswift COMMAND testqpSWIFTSolver)
 endif()



find_package(MATLAB QUIET)
find_package(PythonInterp)
if(MATLAB_FOUND AND PYTHONINTERP_FOUND AND SHOW_TEST_RESULTS)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CTestCustom.cmake ${CMAKE_CURRENT_BINARY_DIR}/../CTestCustom.cmake)
endif()

add_custom_target(copy_test_plotter_m ALL
                  ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plot_test_results.m" "${CMAKE_CURRENT_BINARY_DIR}/")

add_custom_target(copy_test_plotter_py ALL
                  ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plot_test_results.py" "${CMAKE_CURRENT_BINARY_DIR}/")
