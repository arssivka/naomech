cmake_minimum_required(VERSION 3.0.2)

set(RD_VISION_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CACHE STRING "Vision module include dir")

set(RD_VISION_LIBRARIES vision CACHE STRING "Vision module libs")

set(LIBS_REQ
        BOOST OPENCV2_IMGPROC OPENCV2_HIGHGUI OPENCV2_CORE
        JPEG PNG TIFF GSTREAMER GOBJECT ALVALUE
        ALCOMMON XML2 FFI TBB V4L GLIB2 GTHREAD
        GSTREAMER GMODULE FFMPEG BZIP2)

foreach (LIB ${LIBS_REQ})
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
        include/RemoteVision.h
        include/utils/Logger.h
        include/utils/Stopwatch.h
        include/utils/CoreFuntions.h
        include/detectors/BaseDetector.h
        include/detectors/BallDetector.h
        include/detectors/LineDetector.h)

set(SOURCES
        src/Vision.cpp
        src/RemoteVision.cpp
        src/utils/Logger.cpp
        src/utils/Stopwatch.cpp
        src/detectors/BaseDetector.cpp
        src/detectors/BallDetector.cpp
        src/detectors/LineDetector.cpp)

add_library(vision ${HEADERS} ${SOURCES})

target_include_directories(vision PUBLIC ${INCLUDE_DIRS}
        ${RD_NETWORK_INCLUDE_DIRS} ${RD_HARDWARE_INCLUDE_DIRS})

target_link_libraries(vision ${LIBS} ${RD_NETWORK_LIBRARIES} ${RD_HARDWARE_LIBRARIES})

install(TARGETS vision
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)

