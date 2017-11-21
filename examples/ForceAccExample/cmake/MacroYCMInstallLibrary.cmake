#A macro to allow clean, readable YCM library install
macro(ycm_library_install name major_ver minor_ver patch_ver)

set(VARS_PREFIX "${name}")

set(${name}_MAJOR_VERSION ${major_ver})
set(${name}_MINOR_VERSION ${minor_ver})
set(${name}_PATCH_VERSION ${patch_ver})
set(${name}_VERSION ${${name}_MAJOR_VERSION}.${${name}_MINOR_VERSION}.${${name}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(${name} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(${name} PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS ${name}
        EXPORT ${name}
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all ${name} dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${name})


include(InstallBasicPackageFiles)
install_basic_package_files(${name} VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)


endmacro(ycm_library_install name)
