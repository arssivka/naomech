cmake_minimum_required(VERSION 2.8)
project(naomech)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/lib/static")

add_subdirectory(naoqi)
add_subdirectory(xmlrpc-c)
add_subdirectory(hardware)
add_subdirectory(network)
add_subdirectory(controller)
add_subdirectory(vision)
