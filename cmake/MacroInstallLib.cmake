include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CMakeParseArguments)


function(INSTALL_BASIC_PACKAGE _Name)

    # TODO check that _Name does not contain "-" characters

    set(_options NO_SET_AND_CHECK_MACRO
                 NO_CHECK_REQUIRED_COMPONENTS_MACRO
                 UPPERCASE_FILENAMES
                 LOWERCASE_FILENAMES)
    set(_oneValueArgs VERSION
                      COMPATIBILITY
                      TARGETS_PROPERTY
                      VARS_PREFIX
                      DESTINATION
                      NAMESPACE)
    set(_multiValueArgs EXTRA_PATH_VARS_SUFFIX)
    cmake_parse_arguments(_IBPF "${_options}" "${_oneValueArgs}" "${_multiValueArgs}" "${ARGN}")

    if(NOT DEFINED _IBPF_VARS_PREFIX)
        set(_IBPF_VARS_PREFIX ${_Name})
    endif()

    if(NOT DEFINED _IBPF_VERSION)
        message(FATAL_ERROR "VERSION argument is required")
    endif()

    if(NOT DEFINED _IBPF_COMPATIBILITY)
        message(FATAL_ERROR "COMPATIBILITY argument is required")
    endif()

    if(NOT DEFINED _IBPF_TARGETS_PROPERTY)
        message(FATAL_ERROR "TARGETS_PROPERTY argument is required")
    endif()

    if(_IBPF_UPPERCASE_FILENAMES AND _IBPF_LOWERCASE_FILENAMES)
        message(FATAL_ERROR "UPPERCASE_FILENAMES and LOWERCASE_FILENAMES arguments cannot be used together")
    endif()

    # Path for installed cmake files
    if(NOT DEFINED _IBPF_DESTINATION)
        if(WIN32 AND NOT CYGWIN)
            set(_IBPF_DESTINATION CMake)
        else()
            set(_IBPF_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${_Name})
        endif()
    endif()

    if(NOT DEFINED _IBPF_NAMESPACE)
        set(_IBPF_NAMESPACE "${_Name}::")
    endif()

    if(_IBPF_NO_SET_AND_CHECK_MACRO)
        list(APPEND configure_package_config_file_extra_args NO_SET_AND_CHECK_MACRO)
    endif()

    if(_IBPF_NO_CHECK_REQUIRED_COMPONENTS_MACRO)
        list(APPEND configure_package_config_file_extra_args NO_CHECK_REQUIRED_COMPONENTS_MACRO)
    endif()

    string(TOLOWER "${_Name}" _name)
    if(NOT _IBPF_UPPERCASE_FILENAMES AND NOT _IBPF_LOWERCASE_FILENAMES)
        if(EXISTS ${CMAKE_SOURCE_DIR}/${_Name}Config.cmake.in)
            set(_IBPF_UPPERCASE_FILENAMES 1)
        elseif(EXISTS${CMAKE_SOURCE_DIR}/${_name}-config.cmake.in)
            set(_IBPF_LOWERCASE_FILENAMES 1)
        else()
            set(_IBPF_UPPERCASE_FILENAMES 1)
        endif()
    endif()

    if(_IBPF_UPPERCASE_FILENAMES)
        set(_config_filename ${_Name}Config.cmake)
        set(_version_filename ${_Name}ConfigVersion.cmake)
        set(_targets_filename ${_Name}Targets.cmake)
    elseif(_IBPF_LOWERCASE_FILENAMES)
        set(_config_filename ${_name}-config.cmake)
        set(_version_filename ${_name}-config-version.cmake)
        set(_targets_filename ${_name}-targets.cmake)
    endif()

    # Make relative paths absolute (needed later on) and append the
    # defined variables to _(build|install)_path_vars_suffix
    foreach(p BINDIR          BIN_DIR
              SBINDIR         SBIN_DIR
              LIBEXECDIR      LIBEXEC_DIR
              SYSCONFDIR      SYSCONF_DIR
              SHAREDSTATEDIR  SHAREDSTATE_DIR
              LOCALSTATEDIR   LOCALSTATE_DIR
              LIBDIR          LIB_DIR
              INCLUDEDIR      INCLUDE_DIR
              OLDINCLUDEDIR   OLDINCLUDE_DIR
              DATAROOTDIR     DATAROOT_DIR
              DATADIR         DATA_DIR
              INFODIR         INFO_DIR
              LOCALEDIR       LOCALE_DIR
              MANDIR          MAN_DIR
              DOCDIR          DOC_DIR
              ${_IBPF_EXTRA_PATH_VARS_SUFFIX})
        if(DEFINED ${_IBPF_VARS_PREFIX}_BUILD_${p})
            list(APPEND _build_path_vars_suffix ${p})
            list(APPEND _build_path_vars "${_IBPF_VARS_PREFIX}_${p}")
        endif()
        if(DEFINED BUILD_${_IBPF_VARS_PREFIX}_${p})
            list(APPEND _build_path_vars_suffix ${p})
            list(APPEND _build_path_vars "${_IBPF_VARS_PREFIX}_${p}")
        endif()
        if(DEFINED ${_IBPF_VARS_PREFIX}_INSTALL_${p})
            list(APPEND _install_path_vars_suffix ${p})
            list(APPEND _install_path_vars "${_IBPF_VARS_PREFIX}_${p}")
        endif()
        if(DEFINED INSTALL_${_IBPF_VARS_PREFIX}_${p})
            list(APPEND _install_path_vars_suffix ${p})
            list(APPEND _install_path_vars "${_IBPF_VARS_PREFIX}_${p}")
        endif()
    endforeach()



    # Get targets from GLOBAL PROPERTY
    get_property(_targets GLOBAL PROPERTY ${_IBPF_TARGETS_PROPERTY})
    foreach(_target ${_targets})
        list(APPEND ${_IBPF_VARS_PREFIX}_TARGETS ${_Name}::${_target})
    endforeach()
    list(GET ${_IBPF_VARS_PREFIX}_TARGETS 0 _target)



    # <name>ConfigVersion.cmake file (same for build tree and intall)
    write_basic_package_version_file(${CMAKE_BINARY_DIR}/${_version_filename}
                                    VERSION ${_IBPF_VERSION}
                                    COMPATIBILITY ${_IBPF_COMPATIBILITY})
    install(FILES ${CMAKE_BINARY_DIR}/${_version_filename}
            DESTINATION ${_IBPF_DESTINATION})



    # If there is no Config.cmake.in file, write a basic one
    set(_config_cmake_in ${CMAKE_SOURCE_DIR}/${_config_filename}.in)
    if(NOT EXISTS ${_config_cmake_in})
        set(_config_cmake_in ${CMAKE_BINARY_DIR}/${_config_filename}.in)
        file(WRITE ${_config_cmake_in}
"set(${_IBPF_VARS_PREFIX}_VERSION \@${_IBPF_VARS_PREFIX}_VERSION\@)
@PACKAGE_INIT@
set(${_IBPF_VARS_PREFIX}_INCLUDEDIR \"@PACKAGE_${_IBPF_VARS_PREFIX}_INCLUDEDIR@\")
if(NOT TARGET ${_target})
  include(\"\${CMAKE_CURRENT_LIST_DIR}/${_targets_filename}\")
endif()
# Compatibility
set(${_Name}_LIBRARIES ${${_IBPF_VARS_PREFIX}_TARGETS})
set(${_Name}_INCLUDE_DIRS \${${_IBPF_VARS_PREFIX}_INCLUDEDIR})
")
    endif()

    # <name>Config.cmake (build tree)
    foreach(p ${_build_path_vars_suffix})
        if(DEFINED ${_IBPF_VARS_PREFIX}_BUILD_${p})
            set(${_IBPF_VARS_PREFIX}_${p} "${${_IBPF_VARS_PREFIX}_BUILD_${p}}")
        elseif(DEFINED BUILD_${_IBPF_VARS_PREFIX}_${p})
            set(${_IBPF_VARS_PREFIX}_${p} "${BUILD_${_IBPF_VARS_PREFIX}_${p}}")
        endif()
    endforeach()
    configure_package_config_file(${_config_cmake_in}
                                  ${CMAKE_BINARY_DIR}/${_config_filename}
                                  INSTALL_DESTINATION ${CMAKE_BINARY_DIR}
                                  PATH_VARS ${_build_path_vars}
                                  ${configure_package_config_file_extra_args}
                                  INSTALL_PREFIX ${CMAKE_BINARY_DIR})

    # <name>Config.cmake (installed)
    foreach(p ${_install_path_vars_suffix})
        if(DEFINED ${_IBPF_VARS_PREFIX}_INSTALL_${p})
            set(${_IBPF_VARS_PREFIX}_${p} "${${_IBPF_VARS_PREFIX}_INSTALL_${p}}")
        elseif(DEFINED INSTALL_${_IBPF_VARS_PREFIX}_${p})
            set(${_IBPF_VARS_PREFIX}_${p} "${INSTALL_${_IBPF_VARS_PREFIX}_${p}}")
        endif()
    endforeach()
    configure_package_config_file(${_config_cmake_in}
                                  ${CMAKE_BINARY_DIR}/${_config_filename}.install
                                  INSTALL_DESTINATION ${_IBPF_DESTINATION}
                                  PATH_VARS ${_install_path_vars}
                                  ${configure_package_config_file_extra_args})
    install(FILES ${CMAKE_BINARY_DIR}/${_config_filename}.install
            DESTINATION ${_IBPF_DESTINATION}
            RENAME ${_config_filename})



    # <name>Targets.cmake (build tree)
    export(TARGETS ${_targets}
           NAMESPACE ${_IBPF_NAMESPACE}
           FILE ${CMAKE_BINARY_DIR}/${_targets_filename})

    # <name>Targets.cmake (installed)
    install(EXPORT ${_Name}
            NAMESPACE ${_IBPF_NAMESPACE}
            DESTINATION ${_IBPF_DESTINATION}
            FILE ${_targets_filename})

endfunction()



macro(DEFAULT_DIRS _prefix)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

    set(${_prefix}_BUILD_LIBDIR ${CMAKE_INSTALL_LIBDIR})
    set(${_prefix}_BUILD_BINDIR ${CMAKE_INSTALL_BINDIR})
    set(${_prefix}_BUILD_INCLUDEDIR ${CMAKE_SOURCE_DIR}/src)

    set(${_prefix}_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR})
    set(${_prefix}_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
    set(${_prefix}_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})
endmacro()


#A macro to allow clean, readable YCM library install
macro(library_install name major_ver minor_ver patch_ver)

set(VARS_PREFIX "${name}")

set(${name}_MAJOR_VERSION ${major_ver})
set(${name}_MINOR_VERSION ${minor_ver})
set(${name}_PATCH_VERSION ${patch_ver})
set(${name}_VERSION ${${name}_MAJOR_VERSION}.${${name}_MINOR_VERSION}.${${name}_PATCH_VERSION})

default_dirs(${VARS_PREFIX})

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


# include(InstallBasicPackageFiles)
install_basic_package(${name} VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)


endmacro(library_install name)