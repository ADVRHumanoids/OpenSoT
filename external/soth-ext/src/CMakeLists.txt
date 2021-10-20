#
#  Copyright 2010, 2019 CNRS
#
#  Author: Florent Lamiraux, Guilhem Saurel
#

SET(LIBRARY_NAME ${PROJECT_NAME})
IF(WIN32)
  ADD_DEFINITIONS("-DNOMINMAX")
ENDIF(WIN32)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../cmake")
include(MacroInstallLib)


# Boost_SIGNALS_LIBRARY

SET(${PROJECT_NAME}_SRC
  # ActiveSet.cpp
  Allocator.cpp
  Algebra.cpp
  Bound.cpp
  BaseY.cpp
  HCOD.cpp
  Givens.cpp
  debug.cpp
  Stage.cpp
  BasicStage.cpp
  Random.cpp
  )

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${PROJECT_NAME}_SRC})

IF(NOT WIN32)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})
ENDIF(NOT WIN32)

target_compile_options(soth
    PUBLIC
    -std=c++14
    PRIVATE
    -fvisibility-inlines-hidden
    -fvisibility=hidden)

target_compile_definitions(soth
    PRIVATE
    -DSOTH_DLL
    -DSOTH_DLL_EXPORTS)

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION "${PROJECT_VERSION}")

#INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

library_install(${LIBRARY_NAME} 1 0 0)

ADD_LIBRARY(hcod_wrapper SHARED HCOD_wrapper.cpp)

target_compile_options(hcod_wrapper
    PUBLIC
    -std=c++14
    PRIVATE
    -fvisibility-inlines-hidden
    -fvisibility=hidden)

target_compile_definitions(hcod_wrapper
    PRIVATE
    -DSOTH_DLL
    -DSOTH_DLL_EXPORTS)


TARGET_LINK_LIBRARIES(hcod_wrapper ${LIBRARY_NAME})
library_install(hcod_wrapper 1 0 0)
