add_executable(example1 example1.cpp)
target_link_libraries(example1 OpenSoT)

add_definitions(-DOPENSOT_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
set(OPENSOT_EXAMPLE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
message(STATUS "OPENSOT_EXAMPLE_PATH is ${OPENSOT_EXAMPLE_PATH}")

add_executable(example2 example2.cpp JointLimitsPSAP.cpp)
target_link_libraries(example2 OpenSoT)


find_package(catkin QUIET COMPONENTS roscpp rviz_visual_tools tf)
if(${catkin_FOUND})
    add_executable(example3 example3.cpp
                            ${OPENSOT_EXAMPLE_PATH}trajectory_utils.cpp
                            ${OPENSOT_EXAMPLE_PATH}path_circle_fix.cpp
                            ${OPENSOT_EXAMPLE_PATH}ros_trj_publisher.cpp)
    target_link_libraries(example3 OpenSoT ${catkin_LIBRARIES} ${qpOASES_LIBRARIES})
endif()
