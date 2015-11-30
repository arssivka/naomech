cmake_minimum_required(VERSION 2.8)

option(NETWORK_AS_SHARED_LIBRARY "Build networking module as shared library" OFF)

find_package(BOOST REQUIRED)
find_package(XML2 REQUIRED)
find_package(CURL REQUIRED)
find_package(ZLIB REQUIRED)
find_package(ALPROXIES REQUIRED)

set(SRCS src/RPCServer.cpp
        src/RPCModule.cpp
        src/RPCMethod.cpp)

set(HEADERS include/rd/network/RPCModule.h
        include/rd/network/RPCServer.h
        include/rd/network/RPCMethod.h)

set(RD_NETWORK_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CACHE STRING "Networking module includes dir")

set(RD_NETWORK_LIBRARIES network CACHE STRING "Networking module include dir")

if (${NETWORK_AS_SHARED_LIBRARY})
    set(LIBRARY_TYPE SHARED)
else ()
    set(LIBRARY_TYPE STATIC)
endif ()

add_library(network ${LIBRARY_TYPE} ${SRCS} ${HEADERS})

target_include_directories(network PUBLIC
        ${ALPROXIES_INCLUDE_DIRS}
        ${RD_NETWORK_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS}
        ${xmlrpc-c_INCLUDE_DIRS})

target_link_libraries(network
        ${ALPROXIES_LIBRARIES}
        ${BOOST_LIBRARIES}
        xmlrpc++)

install(TARGETS network
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)