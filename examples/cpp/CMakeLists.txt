add_executable(example_simple_qp simple_qp.cpp)
target_link_libraries(example_simple_qp OpenSoT OpenSotBackEndQPOases)

add_definitions(-DOPENSOT_TEST_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
set(OPENSOT_TEST_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
message(STATUS "OPENSOT_TEST_PATH is ${OPENSOT_TEST_PATH}")
add_definitions(-DOPENSOT_TEST_MODEL_TYPE="pin")


add_executable(example_joint_limits_psap joint_limits_psap.cpp JointLimitsPSAP.cpp)
target_link_libraries(example_joint_limits_psap OpenSoT OpenSotBackEndQPOases)



    add_executable(example_static_walk static_walk.cpp qp_estimation.cpp StaticWalkUtils.hpp
                            ${OPENSOT_TEST_PATH}trajectory_utils.cpp
                            ${OPENSOT_TEST_PATH}path_circle_fix.cpp)
    target_link_libraries(example_static_walk OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} rclcpp::rclcpp tf2_eigen::tf2_eigen tf2_ros::tf2_ros sensor_msgs::sensor_msgs_library)
    
    add_executable(example_panda_ik panda_ik.cpp)
    target_link_libraries(example_panda_ik OpenSoT ${qpOASES_LIBRARIES} rclcpp::rclcpp tf2_eigen::tf2_eigen tf2_ros::tf2_ros sensor_msgs::sensor_msgs_library)

    add_executable(example_panda_ik_hcod panda_ik_hcod.cpp)
    target_link_libraries(example_panda_ik_hcod OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} rclcpp::rclcpp tf2_eigen::tf2_eigen tf2_ros::tf2_ros sensor_msgs::sensor_msgs_library)

    add_executable(example_coman_ik coman_ik.cpp)
    target_link_libraries(example_coman_ik OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} rclcpp::rclcpp tf2_eigen::tf2_eigen tf2_ros::tf2_ros sensor_msgs::sensor_msgs_library)

    add_executable(example_coman_ik_hcod coman_ik_hcod.cpp)
    target_link_libraries(example_coman_ik_hcod OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} rclcpp::rclcpp tf2_eigen::tf2_eigen tf2_ros::tf2_ros sensor_msgs::sensor_msgs_library)
