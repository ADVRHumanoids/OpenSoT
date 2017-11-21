# set the search paths
set( Xenomai_SEARCH_PATH /usr/local/xenomai /usr/xenomai /usr/include/xenomai $ENV{XENOMAI_ROOT_DIR})

# find xeno-config.h
find_path( Xenomai_DIR
NAMES include/xeno_config.h xeno_config.h
PATHS ${Xenomai_SEARCH_PATH} )


# did we find xeno_config.h?
if( Xenomai_DIR ) 
    MESSAGE(STATUS "xenomai found: \"${Xenomai_DIR}\"")

    execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
    
    macro(set_xeno_flags target)
	#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS}")
	#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
	#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative -lrtdm")
	set_target_properties(${target} PROPERTIES COMPILE_FLAGS ${XENO_CFLAGS})
	set_target_properties(${target} PROPERTIES LINK_FLAGS "${XENO_LDFLAGS} -lnative -lrtdm")
    endmacro(set_xeno_flags)

    set(Xenomai_FOUND True)
    
else( Xenomai_DIR )
    MESSAGE(STATUS "xenomai NOT found. (${Xenomai_SEARCH_PATH})")
endif( Xenomai_DIR )


