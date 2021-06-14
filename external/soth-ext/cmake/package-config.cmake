# Copyright (C) 2016  LAAS-CNRS, JRL AIST-CNRS and others.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.




# _SETUP_PROJECT_PACKAGE_INIT
# -------------
#
# Initialize the PackageConfig installation configuration.
# https://github.com/forexample/package-example
#
# This function does not take any argument but check that some
# variables are defined (see documentation at the beginning of this
# file).
#
MACRO(_SETUP_PROJECT_PACKAGE_INIT)
####
# Installation (https://github.com/forexample/package-example)

# Layout. This works for all platforms:
#   * <prefix>/lib/cmake/<PROJECT-NAME>
#   * <prefix>/lib/
#   * <prefix>/include/
set(CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
set(INCLUDE_INSTALL_DIR "include")
set(INCLUDE_INSTALL_DESTINATION "${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}")

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Configuration
set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

set(_PACKAGE_CONFIG_DEPENDENCIES "" CACHE INTERNAL "")
ENDMACRO(_SETUP_PROJECT_PACKAGE_INIT)

#.rst:
# .. command:: ADD_PROJECT_DEPENDENCY(ARGS)
#
#   This is a wrapper around find_package to add correct find_dependency calls in
#   the generated config script. All arguments are passed to find_package
#
MACRO(ADD_PROJECT_DEPENDENCY)
  string(REPLACE ";" " " PACKAGE_ARGS "${ARGN}")
  if(${CMAKE_VERSION} VERSION_LESS "3.15.0")
    list(APPEND _PACKAGE_CONFIG_DEPENDENCIES "find_package(${PACKAGE_ARGS})")
  else()
    list(APPEND _PACKAGE_CONFIG_DEPENDENCIES "find_dependency(${PACKAGE_ARGS})")
  endif()
  find_package(${ARGN})
ENDMACRO()


# SETUP_PROJECT_PACKAGE_FINALIZE
# -------------
#
# Generates CMake PackageConfig.cmake, Targets, and Version 
# files so users can call:
#
# find_package(MyPackage)
#
# Initialize the PackageConfig installation configuration.
# https://github.com/forexample/package-example
#
# This function does not take any argument but check that some
# variables are defined (see documentation at the beginning of this
# file).
#
# assumes SETUP_PROJECT() was called
# internally the real requirement is that _SETUP_PROJECT_PACKAGE_INIT() was called
MACRO(SETUP_PROJECT_PACKAGE_FINALIZE)

####
# Installation (https://github.com/forexample/package-example)

# Layout. This works for all platforms:
#   * <prefix>/lib/cmake/<PROJECT-NAME>
#   * <prefix>/lib/
#   * <prefix>/include/
set(CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
set(INCLUDE_INSTALL_DIR "include")
set(INCLUDE_INSTALL_DESTINATION "${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}")

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Configuration
set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

# Include module with fuction 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

string(REPLACE ";" "\n" PACKAGE_DEPENDENCIES "${_PACKAGE_CONFIG_DEPENDENCIES}")

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Note: PROJECT_VERSION is used as a VERSION
write_basic_package_version_file(
    "${VERSION_CONFIG}" VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>Config.cmake'
# Use variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
configure_package_config_file(
    "cmake/Config.cmake.in"
    "${PROJECT_CONFIG}"
    INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Config
#   * <prefix>/lib/cmake/Foo/FooConfig.cmake
#   * <prefix>/lib/cmake/Foo/FooConfigVersion.cmake
install(
    FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
    DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Config
#   * <prefix>/lib/cmake/Foo/FooTargets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${CONFIG_INSTALL_DIR}"
)
ENDMACRO(SETUP_PROJECT_PACKAGE_FINALIZE)
