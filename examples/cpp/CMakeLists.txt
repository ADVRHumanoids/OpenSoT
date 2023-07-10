add_executable(example1 example1.cpp)
target_link_libraries(example1 OpenSoT)

add_definitions(-DOPENSOT_EXAMPLE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../tests/")
message(STATUS "DOPENSOT_EXAMPLE_PATH is ${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(example2 example2.cpp JointLimitsPSAP.cpp)
target_link_libraries(example2 OpenSoT)
