set(GFLAGS_FOUND TRUE) # auto 

#message(${CAFFE3RDPARTY_ROOT_DIR})
if(WIN32)
set(GFLAGS_ROOT_DIR ${CAFFE3RDPARTY_ROOT_DIR})
elseif(UNIX)
else()
endif()
#message(${GFLAGS_ROOT_DIR})

find_path(GFLAGS_INCLUDE_DIR NAMES gflags/gflags.h PATHS "${GFLAGS_ROOT_DIR}/include") 
mark_as_advanced(GFLAGS_INCLUDE_DIR) # show entry in cmake-gui


if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	message("debug mode")
	
	find_library(GFLAGS_LIBRARY NAMES gflagsd.lib PATHS "${GFLAGS_ROOT_DIR}/lib") 
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Release")
	message("release mode")
	
	find_library(GFLAGS_LIBRARY NAMES gflags.lib PATHS "${GFLAGS_ROOT_DIR}/lib") 
else()
endif()

#find_library(GFLAGS_LIBRARY NAMES gflags.lib PATHS "${GFLAGS_ROOT_DIR}/lib") 
#message("-------------------")
#message(${GFLAGS_LIBRARY})
mark_as_advanced(GFLAGS_LIBRARY) # show entry in cmake-gui

# use xxx_INCLUDE_DIRS and xxx_LIBRARIES in CMakeLists.txt
set(GFLAGS_INCLUDE_DIRS ${GFLAGS_INCLUDE_DIR} )
#set(GFLAGS_LIBRARIES ${GFLAGS_LIBRARY} )

message( "gflags-config.cmake " ${GFLAGS_ROOT_DIR})