cmake_minimum_required(VERSION 3.0.2)

project(managers)

set(SOURCES managers.cpp)
set(HEADERS)

add_executable(managers ${SOURCES} ${HEADERS})
target_link_libraries(managers ${RD_VISION_LIBRARIES})
target_include_directories(managers PUBLIC ${RD_VISION_INCLUDE_DIRS})
