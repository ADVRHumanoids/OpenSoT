cmake_minimum_required(VERSION 3.5)

if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    message("Please consider to switch to latest version to enable
    more reliable Python3 binaries detection")
else()
    #find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
endif()

find_package(pybind11 QUIET)

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
                                    constraints/velocity.hpp
                                    constraints/acceleration.hpp
                                    constraints/force.hpp)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyopensot ${OPENSOT_BINDINGS_SOURCES})

    target_link_libraries(pyopensot PUBLIC OpenSoT)


    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    install(TARGETS pyopensot
        COMPONENT python
        LIBRARY DESTINATION "/usr/local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages"
        ARCHIVE DESTINATION "lib"
        RUNTIME DESTINATION "bin")

    if(${OPENSOT_COMPILE_COLLISION})
        set(OPENSOT_BINDINGS_COLLISION_SOURCES pyopensot_collision.cpp constraints/velocity_collision.hpp)

        include_directories(${CMAKE_CURRENT_SOURCE_DIR})
        pybind11_add_module(pyopensot_collision ${OPENSOT_BINDINGS_COLLISION_SOURCES})

        target_link_libraries(pyopensot_collision PUBLIC OpenSoT)


        find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

        install(TARGETS pyopensot_collision
            COMPONENT python
            LIBRARY DESTINATION "~/.local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
            ARCHIVE DESTINATION "lib"
            RUNTIME DESTINATION "bin")
    endif()

    if(${OPENSOT_SOTH_FRONT_END})
        set(OPENSOT_BINDINGS_HCOD_SOURCES pyopensot_hcod.cpp solver_hcod.hpp)

        include_directories(${CMAKE_CURRENT_SOURCE_DIR})
        pybind11_add_module(pyopensot_hcod ${OPENSOT_BINDINGS_HCOD_SOURCES})

        target_link_libraries(pyopensot_hcod PUBLIC OpenSoT)

        find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

        install(TARGETS pyopensot_hcod
            COMPONENT python
            LIBRARY DESTINATION "~/.local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
            ARCHIVE DESTINATION "lib"
            RUNTIME DESTINATION "bin")
    endif()



else()
    message(STATUS "pybind not found")
endif()
