cmake_minimum_required(VERSION 3.20)

find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
find_package(pybind11 QUIET)
find_package(Eigen3 REQUIRED)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    set(OPENSOT_BINDINGS_SOURCES    pyopensot.cpp
                                    variables.hpp
                                    autostack.hpp
                                    sub.hpp
                                    aggregated.hpp
                                    generic.hpp
                                    basic.hpp
                                    solver.hpp
                                    tasks/velocity.hpp
                                    tasks/acceleration.hpp
                                    tasks/minimize_variable.hpp
                                    constraints/velocity.hpp
                                    constraints/acceleration.hpp
                                    constraints/force.hpp
                                    variables/torque.hpp
                                    oc.hpp)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${Eigen3_INCLUDE_DIR})

    if(${OPENSOT_COMPILE_COLLISION})
        set(OPENSOT_BINDINGS_SOURCES ${OPENSOT_BINDINGS_SOURCES} constraints/velocity_collision.hpp)
    endif()
    if(${OPENSOT_SOTH_FRONT_END})
        set(OPENSOT_BINDINGS_SOURCES ${OPENSOT_BINDINGS_SOURCES} solver_hcod.hpp)
    endif()
    if(${hpipm-cpp_FOUND})
        set(OPENSOT_BINDINGS_SOURCES ${OPENSOT_BINDINGS_SOURCES} solver_hpipmoc.hpp)
    endif()

    pybind11_add_module(pyopensot ${OPENSOT_BINDINGS_SOURCES})

    target_compile_definitions(pyopensot PRIVATE $<$<BOOL:${OPENSOT_COMPILE_COLLISION}>:OPENSOT_COMPILE_COLLISION>)
    target_compile_definitions(pyopensot PRIVATE $<$<BOOL:${OPENSOT_SOTH_FRONT_END}>:OPENSOT_SOTH_FRONT_END>)
    target_compile_definitions(pyopensot PRIVATE $<$<BOOL:${hpipm-cpp_FOUND}>:HPIPM_CPP_FOUND>)

    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    target_link_libraries(pyopensot PUBLIC OpenSoT)
    if(${hpipm-cpp_FOUND})
        target_link_libraries(pyopensot PUBLIC OpenSothpipmOC)
    endif()


    install(TARGETS pyopensot
        COMPONENT python
        LIBRARY DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
        ARCHIVE DESTINATION "lib"
        RUNTIME DESTINATION "bin")

else()
    message(STATUS "pybind not found")
endif()
