find_package(Boost REQUIRED COMPONENTS System)
find_package(XML2 REQUIRED)
find_package(CURL REQUIRED)
find_package(ZLIB REQUIRED)

set(SRCS src/RPCServer.cpp
        src/RPCModule.cpp
        src/RPCMethod.cpp)

set(HEADERS include/RD/network/RPCModule.h
        include/RD/network/RPCServer.h
        include/RD/network/RPCMethod.h)


set(RD_NETWORK_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CACHE STRING "Networking module includes dir")


add_library(network SHARED ${SRCS} ${HEADERS})

target_include_directories(network PUBLIC
        ${RD_NETWORK_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS}
        ${xmlrpc-c_INCLUDE_DIRS})

target_link_libraries(network
        ${BOOST_LIBRARIES}
        xmlrpc++)