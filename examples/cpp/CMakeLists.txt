add_executable(example_simple_qp simple_qp.cpp)
target_link_libraries(example_simple_qp OpenSoT)

add_definitions(-DOPENSOT_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
set(OPENSOT_EXAMPLE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
message(STATUS "OPENSOT_EXAMPLE_PATH is ${OPENSOT_EXAMPLE_PATH}")

add_executable(example_joint_limits_psap joint_limits_psap.cpp JointLimitsPSAP.cpp)
target_link_libraries(example_joint_limits_psap OpenSoT)


find_package(catkin QUIET COMPONENTS roscpp rviz_visual_tools tf tf_conversions)
if(${catkin_FOUND})
    add_executable(example_static_walk static_walk.cpp qp_estimation.cpp StaticWalkUtils.hpp
                            ${OPENSOT_EXAMPLE_PATH}trajectory_utils.cpp
                            ${OPENSOT_EXAMPLE_PATH}path_circle_fix.cpp
                            ${OPENSOT_EXAMPLE_PATH}ros_trj_publisher.cpp)
    target_link_libraries(example_static_walk OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES})
    
    add_executable(example_panda_ik panda_ik.cpp)
    target_link_libraries(example_panda_ik OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES})
endif()
