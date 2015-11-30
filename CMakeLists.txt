cmake_minimum_required(VERSION 2.8)
project(naomech)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_subdirectory(xmlrpc-c)
add_subdirectory(hardware)
add_subdirectory(network)
add_subdirectory(controller)