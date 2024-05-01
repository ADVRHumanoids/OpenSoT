cmake_minimum_required(VERSION 3.5)

if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    message("Please consider to switch to latest version to enable
    more reliable Python3 binaries detection")
else()
    find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
endif()

find_package(pybind11 QUIET)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyopensot pyopensot.cpp
                                  variables.hpp
                                  autostack.hpp
                                  sub.hpp
                                  aggregated.hpp
                                  generic.hpp
                                  basic.hpp
                                  solver.hpp
                                  tasks/velocity.hpp
                                  constraints/velocity.hpp)
    target_link_libraries(pyopensot PUBLIC OpenSoT)


    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    install(TARGETS pyopensot
        COMPONENT python
        LIBRARY DESTINATION "~/.local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
        ARCHIVE DESTINATION "lib"
        RUNTIME DESTINATION "bin")


else()
    message(STATUS "pybind not found")
endif()
