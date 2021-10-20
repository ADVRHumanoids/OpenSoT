###############################################################################
# Find Qhull
#
# This sets the following variables:
# QHULL_FOUND - True if Qhull was found.
# QHULL_INCLUDE_DIRS - Directories containing the Qhull include files.
# QHULL_LIBRARIES - Libraries needed to use Qhull.
# QHULL_LIBRARY_DIR - Directory containing Qhull libraries.

# SEARCH_FOR_QHULL
# -----------------
#
# Search for Qhull in a portable way.
#
# This macro adds detected flags to the pkg-config file automatically.
#
MACRO(SEARCH_FOR_QHULL)

  set(QHULL_MAJOR_VERSION 6)

  find_file(QHULL_HEADER
            NAMES libqhull/libqhull.h qhull.h
            HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_INCLUDE_DIR}"
            PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
                  "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373"
            PATH_SUFFIXES qhull src/libqhull libqhull include)

  set(QHULL_HEADER "${QHULL_HEADER}" CACHE INTERNAL "QHull header" FORCE )

  if(QHULL_HEADER)
      get_filename_component(qhull_header ${QHULL_HEADER} NAME_WE)
      if("${qhull_header}" STREQUAL "qhull")
          set(HAVE_QHULL_2011 OFF)
          get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
      elseif("${qhull_header}" STREQUAL "libqhull")
          set(HAVE_QHULL_2011 ON)
          get_filename_component(QHULL_INCLUDE_DIR ${QHULL_HEADER} PATH)
          get_filename_component(QHULL_INCLUDE_DIR ${QHULL_INCLUDE_DIR} PATH)
      endif()
  else(QHULL_HEADER)
      set(QHULL_INCLUDE_DIR "QHULL_INCLUDE_DIR-NOTFOUND")
  endif(QHULL_HEADER)

  set(QHULL_INCLUDE_DIR "${QHULL_INCLUDE_DIR}" CACHE PATH "QHull include dir." FORCE)

  # Prefer static libraries in Windows over shared ones
  if(WIN32)
    find_library(QHULL_LIBRARY
                 NAMES qhullstatic qhull qhull${QHULL_MAJOR_VERSION}
                 HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
                 PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
                       "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373"
                 PATH_SUFFIXES project build bin lib)

    find_library(QHULL_LIBRARY_DEBUG
                 NAMES qhullstatic_d qhull_d qhull_d${QHULL_MAJOR_VERSION} qhull qhull${QHULL_MAJOR_VERSION}
                 HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
                 PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
                       "$ENV{PROGRAMFILES}/qhull 6.2.0.1373" "$ENV{PROGRAMW6432}/qhull 6.2.0.1373"
                 PATH_SUFFIXES project build bin lib)
  else(WIN32)
    find_library(QHULL_LIBRARY
                 NAMES qhull qhull${QHULL_MAJOR_VERSION}
                 HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
                 PATH_SUFFIXES project build bin lib)

    find_library(QHULL_LIBRARY_DEBUG
                 NAMES qhull_d qhull_d${QHULL_MAJOR_VERSION} qhull qhull${QHULL_MAJOR_VERSION}
                 HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
                 PATH_SUFFIXES project build bin lib)

    find_library(QHULL_CPP_LIBRARY
                 NAMES qhullcpp
                 HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}"
                 PATH_SUFFIXES project build bin lib)
  endif(WIN32)

  if(NOT QHULL_LIBRARY_DEBUG)
    set(QHULL_LIBRARY_DEBUG ${QHULL_LIBRARY})
  endif(NOT QHULL_LIBRARY_DEBUG)

  set(QHULL_INCLUDE_DIRS ${QHULL_INCLUDE_DIR})
  set(QHULL_LIBRARIES ${QHULL_LIBRARY} ${QHULL_LIBRARY_DEBUG})

  # C++ library may not be available on old Linux distributions
  if(QHULL_CPP_LIBRARY)
      set(QHULL_LIBRARIES ${QHULL_LIBRARIES} ${QHULL_CPP_LIBRARY})
  endif(QHULL_CPP_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Qhull DEFAULT_MSG QHULL_LIBRARY
      QHULL_INCLUDE_DIR)

  mark_as_advanced(QHULL_LIBRARY QHULL_LIBRARY_DEBUG QHULL_CPP_LIBRARY QHULL_INCLUDE_DIR)

  IF(QHULL_FOUND)
    SET(HAVE_QHULL ON)
    ADD_DEFINITIONS("-DHAVE_QHULL")
    PKG_CONFIG_APPEND_CFLAGS(-DHAVE_QHULL)
    MESSAGE(STATUS "Qhull found")
    GET_FILENAME_COMPONENT(QHULL_LIBRARY_DIR ${QHULL_LIBRARY} PATH)
  ENDIF(QHULL_FOUND)

  IF(HAVE_QHULL_2011)
    ADD_DEFINITIONS("-DHAVE_QHULL_2011")
    PKG_CONFIG_APPEND_CFLAGS(-DHAVE_QHULL_2011)
  ENDIF(HAVE_QHULL_2011)

  IF(QHULL_FOUND)
    INCLUDE_DIRECTORIES(${QHULL_INCLUDE_DIRS})
    LINK_DIRECTORIES(${QHULL_LIBRARY_DIR})
    PKG_CONFIG_APPEND_CFLAGS("-I ${QHULL_INCLUDE_DIRS}")
    PKG_CONFIG_APPEND_LIBRARY_DIR("${QHULL_LIBRARY_DIR}")
  ENDIF(QHULL_FOUND)
ENDMACRO(SEARCH_FOR_QHULL)
