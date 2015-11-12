# -*- cmake -*-

set(CURL_TRANSPORT_DIR ${xmlrpc-c_SOURCE_DIR}/lib/curl_transport)

list(APPEND transport_SOURCES
        ${CURL_TRANSPORT_DIR}/xmlrpc_curl_transport.c
        ${CURL_TRANSPORT_DIR}/curltransaction.c
        ${CURL_TRANSPORT_DIR}/curltransaction.h
        ${CURL_TRANSPORT_DIR}/curlmulti.c
        ${CURL_TRANSPORT_DIR}/curlmulti.h
        ${CURL_TRANSPORT_DIR}/lock.h
        ${CURL_TRANSPORT_DIR}/lock_pthread.c
        ${CURL_TRANSPORT_DIR}/lock_pthread.h
        ${CURL_TRANSPORT_DIR}/curlversion.h)

add_subdirectory(cpp)

### libxmlrpc.so
add_library(xmlrpc SHARED
        double.c parse_datetime.c parse_value.c resource.c trace.c version.c
        json.c xmlrpc_libxml2.c xmlrpc_data.c xmlrpc_datetime.c
        xmlrpc_string.c xmlrpc_array.c xmlrpc_struct.c xmlrpc_build.c
        xmlrpc_decompose.c xmlrpc_parse.c xmlrpc_serialize.c
        xmlrpc_authcookie.c)
target_include_directories(xmlrpc PUBLIC ${XML2_INCLUDE_DIRS})
target_link_libraries(xmlrpc ${XML2_LIBRARIES} xmlrpc_util)
list(APPEND lib_TARGETS xmlrpc)

### libxmlrpc_client.so
add_library(xmlrpc_client SHARED
        xmlrpc_client.c xmlrpc_client_global.c xmlrpc_server_info.c ${transport_SOURCES})

target_link_libraries(xmlrpc_client xmlrpc pthread ${CURL_LIBRARIES})
target_include_directories(xmlrpc_client PUBLIC ${CURL_INCLUDE_DIRS}
        ${CURL_TRANSPORT_DIR})

list(APPEND lib_TARGETS xmlrpc_client)

### libxmlrpc_server.so
add_library(xmlrpc_server SHARED
        registry.c method.c system_method.c)
target_link_libraries(xmlrpc_server xmlrpc)
list(APPEND lib_TARGETS xmlrpc_server)

### libxmlrpc_server_abyss.so
if (ENABLE_ABYSS_SERVER)
    add_library(xmlrpc_server_abyss SHARED xmlrpc_server_abyss.c abyss_handler.c)
    target_link_libraries(xmlrpc_server_abyss xmlrpc_abyss xmlrpc_server)
    list(APPEND lib_TARGETS xmlrpc_server_abyss)
endif (ENABLE_ABYSS_SERVER)

### libxmlrpc_server_cgi.so
if (ENABLE_CGI_SERVER)
    add_library(xmlrpc_server_cgi SHARED
            xmlrpc_server_cgi)
    target_link_libraries(xmlrpc_server_cgi xmlrpc_server)
    list(APPEND lib_TARGETS xmlrpc_server_cgi)
endif (ENABLE_CGI_SERVER)

install(TARGETS ${lib_TARGETS}
        RUNTIME DESTINATION ${_bin}
        LIBRARY DESTINATION ${_lib})

set_target_properties(${lib_TARGETS}
        PROPERTIES
        LINK_FLAGS ${XMLRPC_LINKER_FLAGS}
        VERSION ${XMLRPC_C_LIBVERSION}
        SOVERSION ${XMLRPC_C_SOVERSION})
