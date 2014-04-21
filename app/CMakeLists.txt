# Copyright: (C) 2014 Walkman Consortium
# Authors: Enrico Mingo, Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(YARP REQUIRED)

# pick up yarp's cmake scripts
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)	

set(appname sot_velkincon)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scopes ${CMAKE_CURRENT_SOURCE_DIR}/scopes/*.xml)
file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/templates/*.template)
file(GLOB applications ${CMAKE_CURRENT_SOURCE_DIR}/applications/*.xml)

yarp_install(FILES ${conf} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${scopes} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${templates} DESTINATION ${YARP_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${applications} DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES sot_velkincon.xml DESTINATION ${YARP_MODULES_INSTALL_DIR})

# face target to include resource files
ADD_CUSTOM_TARGET(sot_app SOURCES ${conf} ${scopes} ${templates} ${applications})
