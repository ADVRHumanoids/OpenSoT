#
#  Copyright 2019, LAAS-CNRS
#

SET(${PROJECT_NAME}_HEADERS
  api.hpp
  ActiveSet.hpp
  Algebra.hpp
  Allocator.hpp
  BaseY.hpp
  BasicStage.hpp
  Bound.hpp
  debug.hpp
  DestructiveColPivQR.hpp
  Givens.hpp
  HCOD.hpp
  solvers.hpp
  Stage.hpp
  SubMatrix.hpp
  Random.hpp
  )

INSTALL(FILES ${${PROJECT_NAME}_HEADERS} DESTINATION include/${PROJECT_NAME})
