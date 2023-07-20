message("**** Creating target release_deb to generate .deb for ${PROJECT_NAME}, to use it type: make release_deb ****")
include (InstallRequiredSystemLibraries)

set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/xbot" CACHE PATH "Deb package install prefix")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})

set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
set(CPACK_PACKAGE_ARCHITECTURE "amd64")

set(CPACK_DEBIAN_PACKAGE_DEPENDS "xbotinterface, matlogger2, modelinterfacerbdl")

#set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON)

set(CPACK_DEBIAN_PACKAGE_MAINTAINER "enrico.mingo@iit.it")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "A Task Solving library with Constraints")

find_program(GIT_SCM git DOC "Git version control")
mark_as_advanced(GIT_SCM)
find_file(GITDIR NAMES .git PATHS ${CMAKE_SOURCE_DIR} NO_DEFAULT_PATH)
if (GIT_SCM AND GITDIR)
    execute_process(
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMAND ${GIT_SCM} log -1 "--pretty=format:%h"
        OUTPUT_VARIABLE GIT_SHA1_SHORT
    )
else()
    # No version control
    # e.g. when the software is built from a source tarball
    # and gitrevision.hh is packaged with it but no Git is available
    message(STATUS "Will not remake ${SRCDIR}/gitrevision.hh")
endif()

set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}-${GIT_SHA1_SHORT}")

execute_process(
    COMMAND lsb_release -cs
    OUTPUT_VARIABLE LINUX_DISTRO_NAME
)

string(REPLACE "\n" "" LINUX_DISTRO_NAME ${LINUX_DISTRO_NAME})

set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_ARCHITECTURE}-${LINUX_DISTRO_NAME})

message(STATUS "Will generate package '${CPACK_PACKAGE_FILE_NAME}.deb'")

include(CPack)

add_custom_target(release_deb
                  COMMAND "${CMAKE_CPACK_COMMAND}"
                  COMMENT "Generating .deb using Cpack"
                  DEPENDS ${PROJECT_NAME}
                  )
