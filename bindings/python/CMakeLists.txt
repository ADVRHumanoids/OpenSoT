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
                                    variables/torque.hpp)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${Eigen3_INCLUDE_DIR})
    pybind11_add_module(pyopensot ${OPENSOT_BINDINGS_SOURCES})

    target_link_libraries(pyopensot PUBLIC OpenSoT)

    if(${OPENSOT_SOTH_FRONT_END})
        target_compile_definitions(pyopensot PRIVATE -DOPENSOT_SOTH_FRONT_END)
    endif()

    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    install(TARGETS pyopensot
        COMPONENT python
        LIBRARY DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
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
            LIBRARY DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
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
            LIBRARY DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
            ARCHIVE DESTINATION "lib"
            RUNTIME DESTINATION "bin")
    endif()

    ##### HPIPM
    if(${hpipm-cpp_FOUND})
        set(OPENSOT_BINDINGS_hpipmoc_SOURCES pyopensot_hpipmoc.cpp solver_hpipmoc.hpp)

        include_directories(${CMAKE_CURRENT_SOURCE_DIR})
        pybind11_add_module(pyopensot_hpipmoc ${OPENSOT_BINDINGS_hpipmoc_SOURCES})

        target_link_libraries(pyopensot_hpipmoc PUBLIC OpenSoT OpenSothpipmOC)

        find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

        install(TARGETS pyopensot_hpipmoc
            COMPONENT python
            LIBRARY DESTINATION "~/.local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
            ARCHIVE DESTINATION "lib"
            RUNTIME DESTINATION "bin")



        set(OPENSOT_BINDINGS_oc_SOURCES pyopensot_oc.cpp oc.hpp)

        include_directories(${CMAKE_CURRENT_SOURCE_DIR})
        pybind11_add_module(pyopensot_oc ${OPENSOT_BINDINGS_oc_SOURCES})

        target_link_libraries(pyopensot_oc PUBLIC OpenSoT OpenSothpipmOC)

        find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

        install(TARGETS pyopensot_oc
            COMPONENT python
            LIBRARY DESTINATION "~/.local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
            ARCHIVE DESTINATION "lib"
            RUNTIME DESTINATION "bin")

    endif()


else()
    message(STATUS "pybind not found")
endif()
