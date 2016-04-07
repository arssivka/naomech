cmake_minimum_required(VERSION 3.0.2)

set(RD_VISION_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CACHE STRING "Vision module include dir")
set(RD_RVISION_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CACHE STRING "Remoute Vision module include dir")

set(RD_VISION_LIBRARIES vision CACHE STRING "Vision module libs")
set(RD_RVISION_LIBRARIES rvision CACHE STRING "Vision module libs")

foreach (LIB
        BOOST OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC
        JPEG PNG TIFF GSTREAMER GOBJECT ALVALUE ALCOMMON XML2
        FFI TBB V4L GLIB2 GTHREAD GSTREAMER GMODULE FFMPEG BZIP2)
    find_package(${LIB} REQUIRED)
    set(LIBS ${LIBS} ${${LIB}_LIBRARIES})
    set(INCLUDE_DIRS ${INCLUDE_DIRS} ${${LIB}_INCLUDE_DIRS})
endforeach ()

include_directories(
        include
        include/utils
        include/detectors)

set(HEADERS
        include/Vision.h
        include/utils/Logger.h
        include/utils/Stopwatch.h
        include/utils/CoreFuntions.h
        include/detectors/BaseDetector.h
        include/detectors/BallDetector.h
        include/detectors/LineDetector.h)

set(SOURCES
        src/Vision.cpp
        src/utils/Logger.cpp
        src/utils/Stopwatch.cpp
        src/detectors/BaseDetector.cpp
        src/detectors/BallDetector.cpp
        src/detectors/LineDetector.cpp)

set(RV_HEADERS include/RemoteVision.h)
set(RV_SOURCES src/RemoteVision.cpp)

add_library(vision ${HEADERS} ${SOURCES})
add_library(rvision ${RV_HEADERS} ${RV_SOURCES})

target_include_directories(vision PUBLIC ${INCLUDE_DIRS})
target_include_directories(rvision PUBLIC ${INCLUDE_DIRS}
        ${RD_NETWORK_INCLUDE_DIRS} ${RD_HARDWARE_INCLUDE_DIRS})

target_link_libraries(vision ${LIBS})
target_link_libraries(rvision vision ${RD_NETWORK_LIBRARIES} ${RD_HARDWARE_LIBRARIES})

install(TARGETS rvision
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)

