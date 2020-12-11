# <CMake 版本>
cmake_minimum_required( VERSION 3.14 FATAL_ERROR)




option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()
set(PROJECT_NAME openposeutil)


SET(MSVC_INCREMENTAL_DEFAULT ON)


# <工程配置>
project(${PROJECT_NAME}
 
  LANGUAGES CXX
  VERSION 1.0.0
)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")
# <构建类型>

#set(CMAKE_BUILD_TYPE Debug)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
#message(${CMAKE_BUILD_TYPE})

# <C++标准>
set(CMAKE_CXX_STANDARD 14)          # c++标准
set(CMAKE_CXX_EXTENSIONS ON)       # 只启用ISO C++标准的编译器标志，而不使用特定编译器的扩展
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 语言标准是否必须
                                    # ON:如果此版本不可用，CMake将报错
                                    # OFF:如果此版本不可用,CMake将查找标准的下一个版本
                                    #     c++17->c++14->c++11->c++98


#add_definitions ( -Wall )

#add_definitions(-D LOG4CPLUS_STATIC)




set( CMAKE_INCLUDE_CURRENT_DIR ON )
##############################################################################
#set(CMAKE_AUTOMOC ON) # 设置自动生成moc文件,AUTOMOC打开可以省去QT5_WRAP_CPP命令
#set(CMAKE_AUTORCC ON) # Resource files
#set(CMAKE_AUTOUIC ON) # 设置自动生成ui.h文件,AUTOUIC打开可以省去QT5_WRAP_UI命令
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

if(WIN32)
	set(ZED_SDK_ROOT "C:/Program Files (x86)/ZED SDK")

	set(OPENPOSE_ROOT_DIR "D:/devtools/openpose")

	set(OPENCV_ROOT_DIR ${OPENPOSE_ROOT_DIR}/3rdparty/windows/opencv)
	
	set(CAFFE3RDPARTY_ROOT_DIR ${OPENPOSE_ROOT_DIR}/3rdparty/windows/caffe3rdparty)
	set(CAFFE_ROOT_DIR ${OPENPOSE_ROOT_DIR}/3rdparty/windows/caffe)

	set(OPENCV_CMAKE_DIR ${OPENCV_ROOT_DIR}/x64/vc15/lib)

	set(SPDLOG_ROOT_DIR ${OPENPOSE_ROOT_DIR}/3rdparty/windows/spdlog)
	#set(SWIG_ROOT_DIR ${OPENPOSE_ROOT_DIR}/3rdparty/windows/swig)
	set(JSONCPP_ROOT_DIR ${OPENPOSE_ROOT_DIR}/3rdparty/windows/jsoncpp)

	
elseif(UNIX)

else()
	message(SEND_ERROR "You are on an unsupported platform! (Not Win32, Mac OS X or Unix)")
endif(WIN32)








#message(${OPENCV_CMAKE_DIR})
set(OPENPOSE_CMAKE_DIR ${OPENPOSE_ROOT_DIR}/cmake)

#list(APPEND CMAKE_PREFIX_PATH ${QT5_CMAKE_DIR})
list(APPEND CMAKE_PREFIX_PATH ${OPENPOSE_ROOT_DIR})
list(APPEND CMAKE_PREFIX_PATH ${OPENPOSE_CMAKE_DIR})
list(APPEND CMAKE_PREFIX_PATH ${OPENCV_CMAKE_DIR})
list(APPEND CMAKE_PREFIX_PATH ${GLUT_ROOT_PATH})
list(APPEND CMAKE_PREFIX_PATH ${GLEW_ROOT_PATH})
list(APPEND CMAKE_PREFIX_PATH ${ZED_SDK_ROOT})
#message(${CMAKE_PREFIX_PATH})
#list(APPEND CMAKE_PREFIX_PATH ${GLFAGS_ROOT_PATH})
#message(${GLFAGS_ROOT_PATH})
#set(CMAKE_PREFIX_PATH ${QT5_CMAKE_DIR})

include(gflags-config)




find_package(OpenCV REQUIRED)
find_package(ZED 3 REQUIRED)
#find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenPose REQUIRED)


#find_package(gflags REQUIRED)

#---------------end openpose---------
#set(MOC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/moc)

file(GLOB src_file ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
#file(GLOB ui_file ${CMAKE_CURRENT_SOURCE_DIR}/src/*.ui)
#file(GLOB qrc_file ${CMAKE_CURRENT_SOURCE_DIR}/src/*.qrc)
#file(GLOB moc_file ${CMAKE_CURRENT_SOURCE_DIR}/moc/*.cpp)
file(GLOB cfg_file ${CMAKE_CURRENT_SOURCE_DIR}/cfg/*.config)

file(GLOB_RECURSE CAFFE3RDPARTYLIB "${CAFFE3RDPARTY_ROOT_DIR}/lib/*.lib")

file(GLOB_RECURSE CAFFELIB "${CAFFE_ROOT_DIR}/lib/*.lib")
file(GLOB_RECURSE OPENCV_LIBS "${OPENCV_ROOT_DIR}/x64/vc15/lib/*.lib")



#set(PROJECT_LIB ${PROJECT_NAME}lib)

add_library(${PROJECT_NAME} SHARED "")

#LIST(APPEND CMAKE_PROGRAM_PATH ${QT_DLL_DIR})
LIST(APPEND CMAKE_PROGRAM_PATH ${OPENPOSE_ROOT_DIR}/build/bin)


  set(SPDLOG_INCLUDE_DIR ${SPDLOG_ROOT_DIR}/include)
  set(SPDLOG_LIB 
  ${SPDLOG_ROOT_DIR}/lib/spdlog.lib
  ${SPDLOG_ROOT_DIR}/lib/spdlogd.lib
  )

  set(JSONCPP_INCLUDE_DIR ${JSONCPP_ROOT_DIR}/include)
  
 
  include_directories(${ZED_INCLUDE_DIRS})
  include_directories(${CUDA_INCLUDE_DIRS})
  include_directories(${OPENCV_INCLUDE_DIRS})
  include_directories(${SPDLOG_INCLUDE_DIR})

  include_directories(${JSONCPP_INCLUDE_DIR})




target_include_directories(${PROJECT_NAME} 
  PUBLIC   
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${OPENCV_ROOT_DIR}/include
	${OPENPOSE_ROOT_DIR}/include
	${CAFFE3RDPARTY_ROOT_DIR}/include
	)
	link_directories(${ZED_LIBRARY_DIR})
	link_directories(${CUDA_LIBRARY_DIRS})
	link_directories(${OpenCV_LIBRARY_DIRS})
	link_directories(${SPDLOG_LIB})

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()
	
	#message(${ZED_LIBRARY_DIR})
#QT5_WRAP_UI(UIS ${ParatroopersTraining_UIS})



IF(NOT WIN32)
     SET(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-std=c++11 -Wno-write-strings -O3)
ENDIF()

if(WIN32)
	
	if(MSVC)
		#set_target_properties(${PROJECT_NAME} PROPERTIES
		##	LINK_FLAGS "/ENTRY:mainCRTStartup"
		#)
	elseif(CMAKE_COMPILER_IS_GNUCXX)
			# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows") # Not tested
	else()
		message(SEND_ERROR "You are using an unsupported Windows compiler! (Not MSVC or GCC)")
	endif(MSVC)
elseif(APPLE)
	set_target_properties(${PROJECT_NAME} PROPERTIES
			MACOSX_BUNDLE YES
	)
elseif(UNIX)
	# Nothing special required
else()
	message(SEND_ERROR "You are on an unsupported platform! (Not Win32, Mac OS X or Unix)")
endif(WIN32)
# Use the Widgets module from Qt 5

target_sources(${PROJECT_NAME} PRIVATE
           ${src_file}
	  ${CMAKE_CURRENT_SOURCE_DIR}/include/Utils.hpp
	  ${CMAKE_CURRENT_SOURCE_DIR}/include/flags.hpp
	  ${CMAKE_CURRENT_SOURCE_DIR}/include/zedservice.h
	   ${CMAKE_CURRENT_SOURCE_DIR}/include/openposeUtil.h
	

	   ${CMAKE_CURRENT_SOURCE_DIR}/include/utils.h
	   ${CMAKE_CURRENT_SOURCE_DIR}/include/manpose.h
	   ${CMAKE_CURRENT_SOURCE_DIR}/include/cline3d.h
	   ${CMAKE_CURRENT_SOURCE_DIR}/include/cplane.h
	   ${CMAKE_CURRENT_SOURCE_DIR}/include/simlog.h
   	   ${CMAKE_CURRENT_SOURCE_DIR}/include/dirutil.h	  
 )

#--------------------





if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	message("debug mode")
	set(OPENPOSE_LIB ${OPENPOSE_ROOT_DIR}/build/src/openpose/Debug/openposed.lib)
	set(OPENPOSE_DLL_DIR ${OPENPOSE_ROOT_DIR}/build/x64/Debug/openposed.dll)

	set(PROJECT_EXE_DIR ${PROJECT_BINARY_DIR}/Debug)
	set(JSONCPP_LIB 
  ${JSONCPP_ROOT_DIR}/lib/jsoncpp-d.lib
  )
	#set(GFLAGS_LIBRARYSS ${GLFAGS_ROOT_PATH}/build/lib/Debug/gflags_nothreads_debug.lib)
elseif(${CMAKE_BUILD_TYPE} STREQUAL "Release")
	message("release mode")
	set(OPENPOSE_LIB ${OPENPOSE_ROOT_DIR}/build/src/openpose/Release/openpose.lib)
	set(OPENPOSE_DLL_DIR ${OPENPOSE_ROOT_DIR}/build/x64/Release/openpose.dll)
	#set(GFLAGS_LIBRARYSS ${GLFAGS_ROOT_PATH}/build/lib/Release/gflags_nothreads_static.lib)
	set(PROJECT_EXE_DIR ${PROJECT_BINARY_DIR}/Release)
	set(JSONCPP_LIB 
  ${JSONCPP_ROOT_DIR}/lib/jsoncpp.lib)
else()
     message("unknowen error")
endif()

link_directories(${JSONCPP_LIB})

target_link_libraries(${PROJECT_NAME} 
${OPENPOSE_LIB} 
${CAFFELIB}
${CAFFE3RDPARTYLIB}
${GLOG_LIBRARY} ${GFLAGS_LIBRARY} ${Caffe_LIBS} 
${ZED_LIBRARIES}
${SPECIAL_OS_LIBS}
${OpenCV_LIBRARIES}
${OPENCV_LIBS}
${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED}
${ZED_LIBS}
${JSONCPP_LIB}

)


add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
         #file(GLOB cfg_file ${CMAKE_CURRENT_SOURCE_DIR}/cfg/*.config)
        #COMMAND ${CMAKE_COMMAND} -E copy_if_different ${cfg_file} ${PROJECT_EXE_DIR}
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_DLL_DIR} ${PROJECT_EXE_DIR}
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/gflags.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/gflagsd.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/glog.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/glogd.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/opencv_world401.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/opencv_world401d.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/caffe.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>

	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/boost_thread-vc141-mt-x64-1_69.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/caffehdf5_hl.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/cudart64_100.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>

	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/curand64_100.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/cublas64_100.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/libopenblas.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>

	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/caffehdf5.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/boost_filesystem-vc141-mt-x64-1_69.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/libgcc_s_seh-1.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>

	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/libgfortran-3.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/caffezlib1.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/libquadmath-0.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>

	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/opencv_ffmpeg401_64.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_ROOT_DIR}/bin/VCRUNTIME140.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>

	#COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OPENPOSE_DEBUG_DEBUG_DLL_DIR} ${PROJECT_DEBUG_EXE_DIR}
	#COMMAND ${CMAKE_COMMAND} -E copy_if_different ${QT5_PLATFORMS}/qwindowsd.dll ${PROJECT_EXE_DIR}/platforms/
	
	
	#COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Qt5::PlatformCompositorSupport> $<TARGET_FILE_DIR:${PROJECT_NAME}>
	)

	#install(
    #3FILES "$<TARGET_FILE:Qt5::${PROJECT_NAME}>"
    #DESTINATION bin/platforms
