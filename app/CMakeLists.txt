# # Copyright: (C) 2014 Walkman Consortium
# # Authors: Enrico Mingo, Alessio Rocchi
# # CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# find_package(YARP REQUIRED)
#
# # pick up yarp's cmake scripts
# list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
# include(YarpInstallationHelpers)
#
# set(appname ${PROJECT_NAME})
#
# file(GLOB app ${CMAKE_CURRENT_SOURCE_DIR}/*.xml)
# file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
# file(GLOB conf_templates ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini.template)
# file(GLOB scopes ${CMAKE_CURRENT_SOURCE_DIR}/scopes/*.xml)
# file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/templates/*.template)
# file(GLOB applications ${CMAKE_CURRENT_SOURCE_DIR}/applications/*.xml)
# file(GLOB resources ${CMAKE_CURRENT_SOURCE_DIR}/resources/*)
#
# yarp_install(FILES ${app} DESTINATION ${YARP_MODULES_INSTALL_DIR})
# yarp_install(FILES ${conf} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})
# yarp_install(FILES ${conf_templates} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})
# yarp_install(FILES ${scopes} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})
# yarp_install(FILES ${templates} DESTINATION ${YARP_APPLICATIONS_TEMPLATES_INSTALL_DIR})
# yarp_install(FILES ${applications} DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})
# yarp_install(FILES ${resources} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})
#
# # face target to include resource files
# ADD_CUSTOM_TARGET(app_in_IDE SOURCES ${conf} ${scopes} ${templates} ${applications} ${resources})
