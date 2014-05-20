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

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(paramHelp REQUIRED)
FIND_PACKAGE(kdl_codyco REQUIRED)
FIND_PACKAGE(drc_shared REQUIRED)
FIND_PACKAGE(roslib REQUIRED)
FIND_PACKAGE(rospack REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(moveit_core REQUIRED)

ExternalProject_Add(qpOASES-ext GIT_REPOSITORY https://github.com/asherikov/qpOASES.git
                            PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                            INSTALL_COMMAND "")
                            #CMAKE_ARGS -DCMAKE_CXX_FLAGS:STRING="-fPIC")

set(qpOASES_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/external/src/qpOASES-ext/")

SET(ROS_HYDRO_LIB_FOLDER /opt/ros/hydro/lib/)

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${paramHelp_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} "${qpOASES_FOLDER}/include/")
LINK_DIRECTORIES(${ROS_HYDRO_LIB_FOLDER} "${qpOASES_FOLDER}/lib/")

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

# for every file in sot_INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE sot_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)


ADD_LIBRARY(cartesian_utils STATIC src/cartesian_utils.cpp)
ADD_EXECUTABLE(sot_VelKinCon src/main.cpp src/sot_VelKinCon_ctrl.cpp src/yarp_interface.cpp src/task_solver.cpp ${sot_INCLUDES})
ADD_EXECUTABLE(createMonitorScope src/tests/createMonitorScope.cpp)
ADD_EXECUTABLE(createParamTuner src/tests/createParamTuner.cpp)
ADD_EXECUTABLE(uTest1 src/sot_VelKinCon_ctrl.cpp src/yarp_interface.cpp src/tests/unitTest1.cpp src/task_solver.cpp)
ADD_EXECUTABLE(uTest2 src/tests/unitTest2.cpp)

# we must wait for qpOASES-ext to be ready before building sot_VelKinCon_ctrl
add_dependencies(sot_VelKinCon qpOASES-ext)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(cartesian_utils ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} orocos-kdl urdf kdl_parser
                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(sot_VelKinCon ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES}  ${moveit_core_LIBRARIES} ${srdfdom_LIBRARIES} ${rospack_LIBRARIES} ${rosconsole_LIBRARIES} ${roslib_LIBRARIES}
                      cartesian_utils  ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${paramHelp_LIBRARIES} orocos-kdl urdf kdl_parser
                      qpOASES ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES} )
TARGET_LINK_LIBRARIES(uTest1  ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES}  ${moveit_core_LIBRARIES} ${srdfdom_LIBRARIES} ${rospack_LIBRARIES} ${rosconsole_LIBRARIES} ${roslib_LIBRARIES}
                      cartesian_utils  ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} ${paramHelp_LIBRARIES} orocos-kdl urdf kdl_parser
                      qpOASES ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(uTest2 ${drc_shared_LIBRARIES} ${idynutils_LIBRARIES} cartesian_utils ${kdl_codyco_LIBRARIES} ${iDynTree_LIBRARIES} orocos-kdl urdf kdl_parser
                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(createMonitorScope ${YARP_LIBRARIES})

TARGET_LINK_LIBRARIES(createParamTuner ${YARP_LIBRARIES})


install(TARGETS sot_VelKinCon DESTINATION bin)
#install(TARGETS cartesian_utils sot_VelKinCon_ctrl DESTINATION lib)

add_subdirectory(app)

add_custom_target(  copy_conf_files_to_build ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/app/conf"
        "${CMAKE_BINARY_DIR}/")
