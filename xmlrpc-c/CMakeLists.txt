## -*- cmake -*-
project(xmlrpc-c)
include(FindPkgConfig)
include(CheckIncludeFile)
include(CheckFunctionExists)

cmake_minimum_required(VERSION 2.4)

if (COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif ()


set(XMLRPC_C_VERSION_MAJOR "1" CACHE STRING "Version (major) of xmlrpc-c")
set(XMLRPC_C_VERSION_MINOR "31" CACHE STRING "Version (minor) of xmlrpc-c")
set(XMLRPC_C_VERSION_POINT "0" CACHE STRING "Version (point) of xmlrpc-c")

set(XMLRPC_C_VERSION
        "${XMLRPC_C_VERSION_MAJOR}.${XMLRPC_C_VERSION_MINOR}.${XMLRPC_C_VERSION_POINT}"
        CACHE STRING "Version of xmlrpc-c")

set(XMLRPC_C_LIBVERSION "3.${XMLRPC_C_VERSION_MINOR}")
set(XMLRPC_C_SOVERSION "3")

set(XMLRPC_CXX_LIBVERSION "8.${XMLRPC_C_VERSION_MINOR}")
set(XMLRPC_CXX_SOVERSION "8")

string(REGEX REPLACE "^0+" "" XMLRPC_C_VERSION_MAJOR_NUM "${XMLRPC_C_VERSION_MAJOR}")
string(REGEX REPLACE "^0+" "" XMLRPC_C_VERSION_MINOR_NUM "${XMLRPC_C_VERSION_MINOR}")
string(REGEX REPLACE "^0+(.)" "\\1" XMLRPC_C_VERSION_POINT_NUM "${XMLRPC_C_VERSION_POINT}")

############

find_package(CURL REQUIRED)
find_package(XML2 REQUIRED)

############

set(ENABLE_CGI_SERVER 0 CACHE BOOL "Set iff CGI server shall be enabled")
set(ENABLE_ABYSS_SERVER 1 CACHE BOOL "Set iff Abyss server shall be enabled")
set(ENABLE_ABYSS_THREADS 1 CACHE BOOL "Use pthread")

set(LINKER_AS_NEEDED 1 CACHE BOOL "Use the --as-needed linker option")

####  <wchar.h> tests
check_include_file(wchar.h _have_wchar_h)
if (_have_wchar_h)
    set(HAVE_WCHAR_H 1)
else (_have_wchar_h)
    set(HAVE_WCHAR_H 1)
endif (_have_wchar_h)
set(XMLRPC_HAVE_WCHAR ${HAVE_WCHAR_H})
set(HAVE_WCHAR_H_DEFINE ${HAVE_WCHAR_H})


#######
if (LINKER_AS_NEEDED)
    set(XMLRPC_LINKER_FLAGS "-Wl,--as-needed")
endif (LINKER_AS_NEEDED)


try_compile(HAVE_ATTR_UNUSED
        ${CMAKE_BINARY_DIR}/
        ${xmlrpc-c_SOURCE_DIR}/cmake/try-attr.cc
        CMAKE_FLAGS -DCOMPILE_DEFINITIONS:STRING=-DCMAKE_TEST_ATTR=__unused__)

try_compile(VA_LIST_IS_NOT_ARRAY
        ${CMAKE_BINARY_DIR}/
        ${xmlrpc-c_SOURCE_DIR}/cmake/va-list-is-array.c)

if (VA_LIST_IS_NOT_ARRAY)
    set(VA_LIST_IS_ARRAY 0)
    message(STATUS "va_list is not an array")
else (VA_LIST_IS_NOT_ARRAY)
    set(VA_LIST_IS_ARRAY 1)
    message(STATUS "va_list is an array")
endif (VA_LIST_IS_NOT_ARRAY)


if (HAVE_ATTR_UNUSED)
    set(ATTR_UNUSED "__attribute__((__unused__))")
endif (HAVE_ATTR_UNUSED)

set(DIRECTORY_SEPARATOR "/")

check_include_file(sys/filio.h HAVE_SYS_FILIO_H)
check_include_file(sys/ioctl.h HAVE_SYS_IOCTL_H)
check_include_file(sys/select.h HAVE_SYS_SELECT_H)
check_function_exists(setenv HAVE_SETENV)
check_function_exists(strcasecmp HAVE_STRCASECMP)
check_function_exists(_stricmp HAVE__STRICMP)
check_function_exists(stricmp HAVE_STRICMP)
check_function_exists(strtoll HAVE_STRTOLL)
check_function_exists(__strtoll HAVE___STRTOLL)
check_function_exists(strtoull HAVE_STRTOULL)
check_function_exists(__strtoull HAVE___STRTOULL)
check_function_exists(strtoq HAVE_STRTOQ)
check_function_exists(strtouq HAVE_STRTOUQ)
check_function_exists(gettimeofday HAVE_GETTIMEOFDAY)
check_function_exists(setgroups HAVE_SETGROUPS)
check_function_exists(asprintf HAVE_ASPRINTF)
check_function_exists(pselect HAVE_PSELECT)
check_function_exists(wcsncmp HAVE_WCSNCMP)
check_function_exists(localtime_r HAVE_LOCALTIME_R)
check_function_exists(gmtime_r HAVE_GMTIME_R)

configure_file(${xmlrpc-c_SOURCE_DIR}/xmlrpc_config.h.cmake
        ${xmlrpc-c_BINARY_DIR}/xmlrpc_config.h
        ESCAPE_QUOTES @ONLY)

configure_file(${xmlrpc-c_SOURCE_DIR}/version.h.cmake
        ${xmlrpc-c_BINARY_DIR}/version.h
        ESCAPE_QUOTES @ONLY)

configure_file(${xmlrpc-c_SOURCE_DIR}/include/xmlrpc-c/config.h.cmake
        ${xmlrpc-c_BINARY_DIR}/include/xmlrpc-c/config.h
        ESCAPE_QUOTES @ONLY)


include_directories(${xmlrpc-c_SOURCE_DIR}/include)
include_directories(${xmlrpc-c_BINARY_DIR}/include)
include_directories(${xmlrpc-c_SOURCE_DIR}/lib/util/include)
include_directories(${xmlrpc-c_BINARY_DIR})

set(_lib lib CACHE STRING "Basename of the library-directory; usually 'lib' or 'lib64' (on multilib archs)")
set(_bin bin CACHE STRING "Basename of the bin-directory; usually 'bin'")
set(prefix ${CMAKE_INSTALL_PREFIX})
set(libdir "${prefix}/${_lib}")
set(bindir "${prefix}/${_bin}")
set(mandir "${prefix}/share/man")
set(pkgconfdir "${libdir}/pkgconfig")
set(includedir "${prefix}/include")

set(xmlrpc-c_INCLUDE_DIRS
        ${xmlrpc-c_SOURCE_DIR}/include
        ${xmlrpc-c_BINARY_DIR}/include
        ${xmlrpc-c_SOURCE_DIR}/lib/util/include
        ${xmlrpc-c_BINARY_DIR}
        CACHE STRING "")

#############

add_subdirectory(lib)
add_subdirectory(src)