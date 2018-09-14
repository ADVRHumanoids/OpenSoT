# Copyright (C) 2009 RobotCub Consortium
# Copyright (C) 2013-2014  iCub Facility, Istituto Italiano di Tecnologia
# Authors: Paul Fitzpatrick
#          Daniele E. Domenichelli
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(Doxygen QUIET)

message("Doxygen found: " ${DOXYGEN_FOUND})

if(${DOXYGEN_FOUND})
    message("Creating Documentation")
    set(DOX_GENERATE_HTML YES)
    set(DOX_GENERATE_XML NO)
    option(OPENSOT_DOXYGEN_XML "Generate doxygen XML output, for use by SWIG" NO)
    mark_as_advanced(OPENSOT_DOXYGEN_XML)
    if(OPENSOT_DOXYGEN_XML)
        set(DOX_GENERATE_XML YES)
    endif()
    # openSoT documentation is always verbose now.  It used to be
    # partially stripped of stuff less interesting to end-users.

    add_custom_target(dox COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    set(DOX_PATTERNS "") # Will use doxygen default patterns
    set(DOX_GENERATE_MAN NO)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    set(DOX_GENERATE_HTML YES)
    set(DOX_GENERATE_XML NO)
    add_custom_target(man COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.man)
endif()
