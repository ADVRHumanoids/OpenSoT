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

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION "${PROJECT_VERSION}")

#INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

library_install(${LIBRARY_NAME} 1 0 0)
