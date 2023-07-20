IF(WIN32)
  ADD_DEFINITIONS("-DNOMINMAX")
ENDIF(WIN32)

# Static library that will avoid useless recompilation.
add_library(utils STATIC gettimeofday.cpp RandomGenerator.cpp)

SET(${PROJECT_NAME}_TESTS
  tasearch
  tbasic
  #tchrono
  tcod
  #tdamping
  tdestructiveQR
  #tdump_problem
  tgivens
  thcod
  #thcod_full
  trandom
  tstage
  tsubmatrix
  #ttrack
  )

FOREACH(test ${${PROJECT_NAME}_TESTS})
  ADD_UNIT_TEST(${test} ${test})
  TARGET_LINK_LIBRARIES(${test} utils ${PROJECT_NAME})
ENDFOREACH(test ${${PROJECT_NAME}_TESTS})
