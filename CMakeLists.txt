cmake_minimum_required(VERSION 2.8)
project(naomech C CXX)

option(USE_QIBUILD "Build project with qibuild" ON)

set(RD_MODULES
        HardwareAccess
        Kinematics
)

################################################################################
#                               Some CMake magic                               #
################################################################################

if (USE_QIBUILD)
    find_package(qibuild)
endif ()


foreach (MODULE ${RD_MODULES})
    # Check module config
    set(${MODULE}_MODULE_DIR ${CMAKE_SOURCE_DIR}/${MODULE}
            CACHE FILEPATH "Path to ${MODULE} module")
    set(${MODULE}_CONFIG_FILE ${${MODULE}_MODULE_DIR}/module.cmake)
    if (EXISTS ${${MODULE}_CONFIG_FILE})
        include(${${MODULE}_CONFIG_FILE})

        # Include directory from module
        set(${MODULE}_MODULE_INCLUDE_DIR ${${MODULE}_MODULE_DIR}/include
                CACHE FILEPATH "Path to includes for ${MODULE} module")
        if (EXISTS ${${MODULE}_MODULE_INCLUDE_DIR})
            include_directories(${${MODULE}_MODULE_INCLUDE_DIR})
        else()
            message(WARNING "${${MODULE}_MODULE_INCLUDE_DIR}")
        endif ()

        # Process targets from module
        foreach (TARGET ${RD_TARGETS})
            # Include config for target
            set(${MODULE}_${TARGET}_TARGET_DIR ${${MODULE}_MODULE_DIR}/src/${TARGET}/
                    CACHE FILEPATH "Path to ${TARGET} target directory for ${MODULE} module")
            set(${MODULE}_${TARGET}_TARGET_CONFIG ${${MODULE}_${TARGET}_TARGET_DIR}/target.cmake)
            if (EXISTS ${${MODULE}_${TARGET}_TARGET_CONFIG})
                include(${${MODULE}_${TARGET}_TARGET_CONFIG})

                # Prepare sources list
                set(${MODULE}_${TARGET}_SRCS)
                foreach (SRC ${RD_SRCS})
                    if(EXISTS ${SRC})
                        list(APPEND ${MODULE}_${TARGET}_SRCS ${SRC})
                    elseif(EXISTS ${${MODULE}_${TARGET}_TARGET_DIR}/${SRC})
                        list(APPEND ${MODULE}_${TARGET}_SRCS ${${MODULE}_${TARGET}_TARGET_DIR}/${SRC})
                    else()
                        message(WARNING "File ${SRC} don't exits")
                    endif()
                endforeach ()

                list(LENGTH ${MODULE}_${TARGET}_SRCS ${MODULE}_${TARGET}_SRCS_COUNT)
                if (${MODULE}_${TARGET}_SRCS_COUNT GREATER 0)
                    if (USE_QIBUILD)
                        # Build with qibuild
                        if (${RD_SHARED})
                            qi_create_lib(${TARGET} ${${MODULE}_${TARGET}_SRCS} NO_INSTALL)
                        else()
                            qi_create_bin(${TARGET} ${${MODULE}_${TARGET}_SRCS} NO_INSTALL)
                        endif ()
                        if(NOT "${RD_REQUIRED_PACKAGES}" STREQUAL "")
                            qi_use_lib(${TARGET} ${RD_REQUIRED_PACKAGES})
                        endif()

                        if(NOT "${RD_INSTALL_DESTINATION}" STREQUAL "")
                            # TODO use qi_build
                            install(TARGETS ${TARGET} DESTINATION ${RD_INSTALL_DESTINATION})
                        endif()
                    else()
                        # Build without qibuild
                        set(${MODULE}_${TARGET}_INCLUDE_DIRS)
                        set(${MODULE}_${TARGET}_LIBS)

                        foreach (PACKAGE ${RD_REQUIRED_PACKAGES})
                            find_package(${PACKAGE} REQUIRED)
                            list(APPEND ${MODULE}_${TARGET}_LIBS ${${PACKAGE}_LIBRARY})
                            list(APPEND ${MODULE}_${TARGET}_INCLUDE_DIRS ${${PACKAGE}_INCLUDE_DIRS})
                        endforeach ()

                        if (${RD_SHARED})
                            add_library(${TARGET} ${${MODULE}_${TARGET}_SRCS})
                        else()
                            add_executable(${TARGET} ${${MODULE}_${TARGET}_SRCS})
                        endif ()

                        if(NOT "${RD_REQUIRED_PACKAGES}" STREQUAL "")
                            target_include_directories(${TARGET} PRIVATE ${${MODULE}_${TARGET}_INCLUDE_DIRS})
                            target_link_libraries(${TARGET} ${${MODULE}_${TARGET}_LIBS})
                        endif()

                        if(NOT "${RD_INSTALL_DESTINATION}" STREQUAL "")
                            install(TARGETS ${TARGET} DESTINATION ${RD_INSTALL_DESTINATION})
                        endif()

                    endif ()
                else()
                    message(WARNING "Source list for ${TARGET} target in module ${MODULE} is empty. Target skipped.")
                endif ()
            else()
                message(WARNING "Can't find config for ${TARGET} target in module ${MODULE}. Target skipped.")
            endif ()
        endforeach ()
    else()
        message(WARNING "Can't find config for ${MODULE} module. Module skipped.")
    endif ()
endforeach ()