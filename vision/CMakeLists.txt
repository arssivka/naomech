cmake_minimum_required(VERSION 2.8)

find_package(BOOST REQUIRED)
find_package(OPENCV2_CORE REQUIRED)
find_package(OPENCV2_HIGHGUI REQUIRED)

include_directories(
        include
        include/utils
        include/managers
        include/detectors)

set(HEADERS
        include/Vision.h
        include/utils/Param.h
        include/utils/Logger.h
        include/utils/Stopwatch.h
        include/utils/CoreFuntions.h
        include/managers/BaseManager.h
        include/managers/LineDetectorManager.h
        include/detectors/BaseDetector.h
        include/detectors/LineDetector.h)

set(SOURCES
        src/Vision.cpp
        src/utils/Stopwatch.cpp
        src/managers/BaseManager.cpp
        src/managers/LineDetectorManager.cpp
        src/detectors/LineDetector.cpp)

set(INCLUDE_DIRS
        ${BOOST_INCLUDE_DIRS}
        ${OPENCV2_CORE_INCLUDE_DIRS}
        ${OPENCV2_HIGHGUI_INCLUDE_DIRS})

set(LIBS
        ${BOOST_LIBRARIES}
        ${OPENCV2_CORE_LIBRARIES}
        ${OPENCV2_HIGHGUI_LIBRARIES})

add_library(vision ${LIBS} ${HEADERS} ${SOURCES})

add_executable(vision_test tests/main.cpp)

target_include_directories(vision PUBLIC ${INCLUDE_DIRS})

target_link_libraries(vision ${LIBS})
link_libraries(vision_test vision)

install(TARGETS vision
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)