cmake_minimum_required(VERSION 3.0.2)

find_package(BOOST REQUIRED)

include_directories(include)

set(HEADERS
        include/BaseManager.h
        include/LineDetectorManager.h
        include/Param.h)

set(SOURCES
        src/BaseManager.cpp
        src/LineDetectorManager.cpp)

add_executable(managers vision_manager.cpp ${HEADERS} ${SOURCES})

target_link_libraries(managers ${RD_VISION_LIBRARIES})
target_include_directories(managers PUBLIC ${RD_VISION_INCLUDE_DIRS})