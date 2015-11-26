##############################################################################
# @file  FindMATLAB.cmake
# @brief Find MATLAB installation.
#
# @par Input variables:
# <table border="0">
#   <tr>
#     @tp @b MATLAB_DIR @endtp
#     <td>The installation directory of MATLAB.
#         Can also be set as environment variable.</td>
#   </tr>
#   <tr>
#     @tp @b MATLABDIR @endtp
#     <td>Alternative environment variable for @p MATLAB_DIR.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_FIND_COMPONENTS @endtp
#     <td>The @c COMPONENTS argument(s) of the find_package() command can
#         be used to only look for specific MATLAB executables and libraries.
#         Valid component values are "matlab", "mcc", "mexext", "mex",
#         "libmex", "mx" or "libmx", and "eng" or "libeng".</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_FIND_OPTIONAL_COMPONENTS @endtp
#     <td>The @c OPTIONAL_COMPONENTS argument(s) of the find_package() command.
#         See @c MATLAB_FIND_COMPONENTS.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_PATH_SUFFIXES @endtp
#     <td>Path suffixes which are used to find the proper MATLAB libraries.
#         By default, this find module tries to determine the path suffix
#         from the CMake variables which describe the system. For example,
#         on 64-bit Unix-based systems, the libraries are searched in
#         @p MATLAB_DIR/bin/glnxa64. Set this variable before the
#         find_package() command if this find module fails to
#         determine the correct location of the MATLAB libraries within
#         the root directory.</td>
#   </tr>
# </table>
#
# @par Output variables:
# <table border="0">
#   <tr>
#     @tp @b MATLAB_FOUND @endtp
#     <td>Whether the package was found and the following CMake
#         variables are valid.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_EXECUTABLE @endtp
#     <td>The absolute path of the found matlab executable.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_MCC_EXECUTABLE @endtp
#     <td>The absolute path of the found MATLAB Compiler (mcc) executable.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_MEX_EXECUTABLE @endtp
#     <td>The absolute path of the found MEX script (mex) executable.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_MEXEXT_EXECUTABLE @endtp
#     <td>The absolute path of the found mexext script executable.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_INCLUDE_DIR @endtp
#     <td>Package include directories.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_INCLUDES @endtp
#     <td>Include directories including prerequisite libraries.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_mex_LIBRARY @endtp
#     <td>The MEX library of MATLAB.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_mx_LIBRARY @endtp
#     <td>The @c mx library of MATLAB.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_eng_LIBRARY @endtp
#     <td>The MATLAB engine library.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_LIBRARY @endtp
#     <td>All MATLAB libraries.</td>
#   </tr>
#   <tr>
#     @tp @b MATLAB_LIBRARIES @endtp
#     <td>Package libraries and prerequisite libraries.</td>
#   </tr>
# </table>
#
# Copyright (c) 2011, 2012 University of Pennsylvania. All rights reserved.<br />
# See http://www.rad.upenn.edu/sbia/software/license.html or COPYING file.
#
# Contact: SBIA Group <sbia-software at uphs.upenn.edu>
#
# @ingroup CMakeFindModules
##############################################################################

# ----------------------------------------------------------------------------
# initialize search
if (NOT MATLAB_DIR)
  if (NOT $ENV{MATLABDIR} STREQUAL "")
    set (MATLAB_DIR "$ENV{MATLABDIR}"  CACHE PATH "Installation prefix for MATLAB." FORCE)
  else ()
    set (MATLAB_DIR "$ENV{MATLAB_DIR}" CACHE PATH "Installation prefix for MATLAB." FORCE)
  endif ()
endif ()

if (NOT MATLAB_DIR_HINTS)
    if(WIN32)
        set(MATLAB_DIR_HINTS "C:/Program Files/MATLAB")
    else(WIN32)
        FILE(GLOB usr_local_MATLAB /usr/local/MATLAB/*)
        FILE(GLOB usr_local_atlab /usr/local/*atlab/*)
        FILE(GLOB opt_MATLAB /opt/MATLAB/*)
        FILE(GLOB opt_atlab /opt/*atlab/*)
        set(MATLAB_DIR_HINTS ${usr_local_MATLAB} ${usr_local_atlab} ${opt_MATLAB} ${opt_atlab})
    endif()
endif()

if (NOT MATLAB_PATH_SUFFIXES)
  if (WIN32)
    if (CMAKE_GENERATOR MATCHES "Visual Studio 6")
      set (MATLAB_PATH_SUFFIXES "extern/lib/win32/microsoft/msvc60")
    elseif (CMAKE_GENERATOR MATCHES "Visual Studio 7")
      # assume people are generally using 7.1,
      # if using 7.0 need to link to: extern/lib/win32/microsoft/msvc70
      set (MATLAB_PATH_SUFFIXES "extern/lib/win32/microsoft/msvc71")
    elseif (CMAKE_GENERATOR MATCHES "Visual Studio 8")
      set (MATLAB_PATH_SUFFIXES "extern/lib/win32/microsoft/msvc80")
    elseif (CMAKE_GENERATOR MATCHES "Visual Studio 9")
      set (MATLAB_PATH_SUFFIXES "extern/lib/win32/microsoft/msvc90")
    elseif (CMAKE_GENERATOR MATCHES "Borland")
      # assume people are generally using 5.4
      # if using 5.0 need to link to: ../extern/lib/win32/microsoft/bcc50
      # if using 5.1 need to link to: ../extern/lib/win32/microsoft/bcc51
      set (MATLAB_PATH_SUFFIXES "extern/lib/win32/microsoft/bcc54")
    endif ()
  else ()
    if (CMAKE_SIZE_OF_VOID_P EQUAL 4)
      set (MATLAB_PATH_SUFFIXES "bin/glnx86")
    else ()
      set (MATLAB_PATH_SUFFIXES "bin/glnxa64")
    endif ()
  endif ()
endif ()

set (_MATLAB_EXECUTABLE_NAMES)
set (_MATLAB_LIBRARY_NAMES)
set (_MATLAB_OPTIONAL_EXECUTABLE_NAMES)
set (_MATLAB_OPTIONAL_LIBRARY_NAMES)

if (MATLAB_FIND_COMPONENTS OR MATLAB_FIND_OPTIONAL_COMPONENTS)
  foreach (_MATLAB_COMPONENT IN LISTS MATLAB_FIND_COMPONENTS)
    string (TOLOWER "${_MATLAB_COMPONENT}" _MATLAB_COMPONENT)
    if (_MATLAB_COMPONENT MATCHES "^(matlab|mcc|mexext|mex)$")
      list (APPEND _MATLAB_EXECUTABLE_NAMES ${_MATLAB_COMPONENT})
    elseif (_MATLAB_COMPONENT MATCHES "^(lib)?(mex|mx|eng)$")
      list (APPEND _MATLAB_LIBRARY_NAMES ${CMAKE_MATCH_2})
    else ()
      message (FATAL_ERROR "Unknown MATLAB component: ${_MATLAB_COMPONENT}")
    endif ()
  endforeach ()
  foreach (_MATLAB_COMPONENT IN LISTS MATLAB_FIND_OPTIONAL_COMPONENTS)
    string (TOLOWER "${_MATLAB_COMPONENT}" _MATLAB_COMPONENT)
    if (_MATLAB_COMPONENT MATCHES "^(matlab|mcc|mexext|mex)$")
      list (APPEND _MATLAB_OPTIONAL_EXECUTABLE_NAMES ${_MATLAB_COMPONENT})
    elseif (_MATLAB_COMPONENT MATCHES "^(lib)?(mex|mx|eng)$")
      list (APPEND _MATLAB_OPTIONAL_LIBRARY_NAMES ${CMAKE_MATCH_2})
    else ()
      message (FATAL_ERROR "Unknown MATLAB component: ${_MATLAB_COMPONENT}")
    endif ()
  endforeach ()
else ()
  set (_MATLAB_EXECUTABLE_NAMES          matlab)
  set (_MATLAB_OPTIONAL_EXECUTABLE_NAMES mcc mex mexext)
  set (_MATLAB_LIBRARY_NAMES             mex mx eng)
  set (_MATLAB_OPTIONAL_LIBRARY_NAMES)
endif ()

# ----------------------------------------------------------------------------
# find MATLAB executables
if (_MATLAB_EXECUTABLE_NAMES OR _MATLAB_OPTIONAL_EXECUTABLE_NAMES)
  if (MATLAB_DIR)

    foreach (_MATLAB_EXE IN LISTS _MATLAB_EXECUTABLE_NAMES _MATLAB_OPTIONAL_EXECUTABLE_NAMES)
      if (_MATLAB_EXE MATCHES "matlab")
        find_program (
          MATLAB_EXECUTABLE
            NAMES matlab
            HINTS "${MATLAB_DIR}"
            PATH_SUFFIXES bin bin/glnx86 bin/glnxa64 ${MATLAB_PATH_SUFFIXES}
            DOC   "The MATLAB application (matlab)."
        )
        message(STATUS "Found matlab in ${MATLAB_EXECUTABLE}")
        mark_as_advanced (MATLAB_EXECUTABLE)
      else ()
        string (TOUPPER "${_MATLAB_EXE}" _MATLAB_EXE_U)
        if (WIN32 AND _MATLAB_EXE MATCHES "mex")
          list (APPEND _MATLAB_EXE "${_MATLAB_EXE}.bat")
        endif ()
        find_program (
          MATLAB_${_MATLAB_EXE_U}_EXECUTABLE
            NAMES ${_MATLAB_EXE}
            HINTS "${MATLAB_DIR}"
            PATH_SUFFIXES bin bin/glnx86 bin/glnxa64 ${MATLAB_PATH_SUFFIXES}
            DOC   "The MATLAB application ${_MATLAB_EXE}."
        )
        mark_as_advanced (MATLAB_${_MATLAB_EXE_U}_EXECUTABLE)
      endif ()
    endforeach ()

  else ()

    foreach (_MATLAB_EXE IN LISTS _MATLAB_EXECUTABLE_NAMES _MATLAB_OPTIONAL_EXECUTABLE_NAMES)
      if (_MATLAB_EXE MATCHES "matlab")
        find_program (
          MATLAB_EXECUTABLE
            NAMES matlab
            HINTS  ${MATLAB_DIR_HINTS}
            PATH_SUFFIXES bin bin/glnx86 bin/glnxa64 ${MATLAB_PATH_SUFFIXES}
            DOC   "The MATLAB application (matlab)."
        )
        mark_as_advanced (MATLAB_EXECUTABLE)
      else ()
        string (TOUPPER "${_MATLAB_EXE}" _MATLAB_EXE_U)
        find_program (
          MATLAB_${_MATLAB_EXE_U}_EXECUTABLE
            NAMES "${_MATLAB_EXE}"
            HINTS  ${MATLAB_DIR_HINTS}
            PATH_SUFFIXES bin bin/glnx86 bin/glnxa64 ${MATLAB_PATH_SUFFIXES}
            DOC   "The MATLAB application ${_MATLAB_EXE}."
        )
        mark_as_advanced (MATLAB_${_MATLAB_EXE_U}_EXECUTABLE)
      endif ()
    endforeach ()

  endif ()
endif ()

# ----------------------------------------------------------------------------
# set MATLAB_DIR
if (NOT MATLAB_DIR AND MATLAB_EXECUTABLE)
  string (REGEX REPLACE "/bin(/[a-z0-9]+)?/(matlab|MATLAB)(\\.exe|\\.EXE)?$" "" _MATLAB_PREFIX "${MATLAB_EXECUTABLE}")
  set (MATLAB_DIR "${_MATLAB_PREFIX}" CACHE PATH "Installation prefix for MATLAB." FORCE)
endif ()

# ----------------------------------------------------------------------------
# find paths/files
if (_MATLAB_LIBRARY_NAMES OR _MATLAB_OPTIONAL_LIBRARY_NAMES)
  if (MATLAB_DIR)

    find_path (
      MATLAB_INCLUDE_DIR
        NAMES mex.h
        HINTS "${MATLAB_DIR}/extern/include"
        DOC   "Include directory for MATLAB libraries."
        NO_DEFAULT_PATH
    )

    foreach (_MATLAB_LIB IN LISTS _MATLAB_LIBRARY_NAMES _MATLAB_OPTIONAL_LIBRARY_NAMES)
      find_library (
        MATLAB_${_MATLAB_LIB}_LIBRARY
          NAMES         "${_MATLAB_LIB}" "lib${_MATLAB_LIB}"
          HINTS         "${MATLAB_DIR}"
          PATH_SUFFIXES ${MATLAB_PATH_SUFFIXES}
          DOC           "MATLAB ${_MATLAB_LIB} link library."
          NO_DEFAULT_PATH
      )
    endforeach ()

  else ()

    find_path (
      MATLAB_INCLUDE_DIR
        NAMES mex.h
        HINTS ENV C_INCLUDE_PATH ENV CXX_INCLUDE_PATH
        DOC   "Include directory for MATLAB libraries."
    )

    foreach (_MATLAB_LIB IN LISTS _MATLAB_LIBRARY_NAMES _MATLAB_OPTIONAL_LIBRARY_NAMES)
      find_library (
        MATLAB_${_MATLAB_LIB}_LIBRARY
          NAMES "${_MATLAB_LIB}"
          HINTS ENV LD_LIBRARY_PATH
          DOC   "MATLAB ${_MATLAB_LIB} link library."
      )
    endforeach ()

  endif ()
  # mark variables as advanced
  mark_as_advanced (MATLAB_INCLUDE_DIR)
  foreach (_MATLAB_LIB IN LISTS _MATLAB_LIBRARY_NAMES _MATLAB_OPTIONAL_LIBRARY_NAMES)
    mark_as_advanced (MATLAB_${_MATLAB_LIB}_LIBRARY)
  endforeach ()
  # list of all libraries
  set (MATLAB_LIBRARY)
  foreach (_MATLAB_LIB IN LISTS _MATLAB_LIBRARY_NAMES _MATLAB_OPTIONAL_LIBRARY_NAMES)
    if (MATLAB_${_MATLAB_LIB}_LIBRARY)
      list (APPEND MATLAB_LIBRARY "${MATLAB_${_MATLAB_LIB}_LIBRARY}")
    endif ()
  endforeach ()
  # prerequisite libraries
  set (MATLAB_INCLUDES  "${MATLAB_INCLUDE_DIR}")
  set (MATLAB_LIBRARIES "${MATLAB_LIBRARY}")
  # aliases / backwards compatibility
  set (MATLAB_INCLUDE_DIRS "${MATLAB_INCLUDES}")

endif ()

# ----------------------------------------------------------------------------
# handle the QUIETLY and REQUIRED arguments and set *_FOUND to TRUE
# if all listed variables are found or TRUE
include (FindPackageHandleStandardArgs)

set (_MATLAB_REQUIRED_VARS)

foreach (_MATLAB_EXE IN LISTS _MATLAB_EXECUTABLE_NAMES)
  if (_MATLAB_EXE MATCHES "matlab")
    list (APPEND _MATLAB_REQUIRED_VARS MATLAB_EXECUTABLE)
  else ()
    string (TOUPPER "${_MATLAB_EXE}" _MATLAB_EXECUTABLE)
    list (APPEND _MATLAB_REQUIRED_VARS MATLAB_${_MATLAB_EXECUTABLE}_EXECUTABLE)
  endif ()
endforeach ()

if (_MATLAB_LIBRARY_NAMES)
  list (APPEND _MATLAB_REQUIRED_VARS MATLAB_INCLUDE_DIR)
  foreach (_MATLAB_LIB IN LISTS _MATLAB_LIBRARY_NAMES)
    list (APPEND _MATLAB_REQUIRED_VARS MATLAB_${_MATLAB_LIB}_LIBRARY)
  endforeach ()
endif ()

if (_MATLAB_REQUIRED_VARS)
  find_package_handle_standard_args (
    MATLAB
  # MESSAGE
      DEFAULT_MSG
  # VARIABLES
      ${_MATLAB_REQUIRED_VARS}
  )
else ()
  set (MATLAB_FOUND TRUE)
endif ()

# ----------------------------------------------------------------------------
# set MATLAB_DIR
if (NOT MATLAB_DIR AND MATLAB_INCLUDE_DIR)
  string (REGEX REPLACE "/extern/include/?" "" _MATLAB_PREFIX "${MATLAB_INCLUDE_DIR}")
  set (MATLAB_DIR "${_MATLAB_PREFIX}" CACHE PATH "Installation prefix for MATLAB." FORCE)
endif ()

# ----------------------------------------------------------------------------
# unset private variables
unset (_MATLAB_REQUIRED_VARS)
unset (_MATLAB_EXECUTABLE_NAMES)
unset (_MATLAB_LIBRARY_NAMES)
unset (_MATLAB_PREFIX)
unset (_MATLAB_LIB)
unset (_MATLAB_EXE)