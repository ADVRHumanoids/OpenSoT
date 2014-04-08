# Copyright: (C) 2013 Walkman Consortium
# Authors: ...
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(YARP REQUIRED)

# pick up yarp's cmake scripts
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)	

file(GLOB conf conf/*.ini)

yarp_install(FILES ${conf} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/sot_velkincon)   # directory name matches what is provided as "default context" in code
