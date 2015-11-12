# -*- cmake -*-

set(libxml_INCLUDES ${LIBXML2_CFLAGS})
set(libxml_LIBS ${LIBXML2_LDFLAGS})
set(xmlrpc_pkgconfig_req ${libxml_pkgconfig})


set(client_curl_CFLAGS ${CURL_CFLAGS})
set(client_curl_LIBS ${CURL_LDFLAGS} pthread)
set(xmlrpc_client_pkgconfig_req libcurl)
list(APPEND transport_SOURCES
        ${curl_srcdir}/xmlrpc_curl_transport.c
        ${curl_srcdir}/curltransaction.c
        ${curl_srcdir}/curltransaction.h
        ${curl_srcdir}/curlmulti.c
        ${curl_srcdir}/curlmulti.h
        ${curl_srcdir}/lock.h
        ${curl_srcdir}/lock_pthread.c
        ${curl_srcdir}/lock_pthread.h
        ${curl_srcdir}/curlversion.h
        )

add_subdirectory(cpp)


### libxmlrpc.so
add_library(xmlrpc SHARED
        double.c parse_datetime.c parse_value.c resource.c trace.c version.c
        json.c xmlrpc_libxml2.c xmlrpc_data.c xmlrpc_datetime.c
        xmlrpc_string.c xmlrpc_array.c xmlrpc_struct.c xmlrpc_build.c
        xmlrpc_decompose.c xmlrpc_parse.c xmlrpc_serialize.c
        xmlrpc_authcookie.c)
target_include_directories(xmlrpc PUBLIC
        ${XML2_INCLUDE_DIRS})
target_link_libraries(xmlrpc ${XML2_LIBRARIES} xmlrpc_util)
list(APPEND lib_TARGETS xmlrpc)

### libxmlrpc_client.so
add_library(xmlrpc_client SHARED
        xmlrpc_client.c xmlrpc_client_global.c xmlrpc_server_info.c ${transport_SOURCES})

target_link_libraries(xmlrpc_client xmlrpc ${client_LIBS})
target_include_directories(xmlrpc_client PUBLIC
        ${CURL_INCLUDE_DIRS}
        ${curl_srcdir})

list(APPEND lib_TARGETS xmlrpc_client)

### libxmlrpc_server.so
add_library(xmlrpc_server SHARED
        registry.c method.c system_method.c)
target_link_libraries(xmlrpc_server xmlrpc)
list(APPEND lib_TARGETS xmlrpc_server)

### libxmlrpc_server_abyss.so
if (ENABLE_ABYSS_SERVER)
    add_library(xmlrpc_server_abyss SHARED
            xmlrpc_server_abyss.c abyss_handler.c)
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
