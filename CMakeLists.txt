cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(sot_VelKinCon)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

ExternalProject_Add(qpOASES-ext SOURCE_DIR $ENV{YARP_WORKSPACE}/coman_yarp_apps/extern/qpOASES/ INSTALL_COMMAND "")


SET(ROS_HYDRO_LIB_FOLDER /opt/ros/hydro/lib/)
SET(KDL_CODYCO_LIB_FOLDER $ENV{YARP_WORKSPACE}/kdl_codyco/build/ CACHE PATH "Path where kdl_codyco libs are installed (can be its build folder)")
SET(CODYCO_LIB_FOLDER $ENV{YARP_WORKSPACE}/codyco/build/lib/ CACHE PATH "Path where codyco libs are installed (can be its build/lib folder)")
SET(WALKMAN_LIB_FOLDER $ENV{WALKMAN_ROOT}/build/install/lib/ CACHE PATH "Path where WALKMAN libs are installed (should be its build/install/lib folder)")

# add include directories
INCLUDE_DIRECTORIES(include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${IDYN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} $ENV{YARP_WORKSPACE}/coman_yarp_apps/extern/qpOASES/include/)
LINK_DIRECTORIES(${ROS_HYDRO_LIB_FOLDER} ${WALKMAN_LIB_FOLDER} ${KDL_CODYCO_LIB_FOLDER} ${CODYCO_LIB_FOLDER} $ENV{YARP_WORKSPACE}/coman_yarp_apps/extern/qpOASES/lib/)

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              

ADD_LIBRARY(sot_VelKinCon_ctrl src/sot_VelKinCon_ctrl.cpp src/yarp_interface.cpp src/task_solver.cpp)
ADD_LIBRARY(cartesian_utils src/cartesian_utils.cpp)
ADD_EXECUTABLE(sot_VelKinCon src/main.cpp)
ADD_EXECUTABLE(uTest1 src/tests/unitTest1.cpp)
ADD_EXECUTABLE(uTest2 src/tests/unitTest2.cpp)

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(cartesian_utils kdl_codyco iDynTree orocos-kdl urdf kdl_parser
                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(sot_VelKinCon_ctrl cartesian_utils kdl_codyco iDynTree orocos-kdl urdf kdl_parser
                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES} qpOASES)
TARGET_LINK_LIBRARIES(sot_VelKinCon sot_VelKinCon_ctrl cartesian_utils kdl_codyco iDynTree orocos-kdl urdf kdl_parser
                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(uTest1 sot_VelKinCon_ctrl kdl_codyco iDynTree orocos-kdl urdf kdl_parser
                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(uTest2 cartesian_utils kdl_codyco iDynTree orocos-kdl urdf kdl_parser
                      ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES})
