cmake_minimum_required(VERSION 2.8)

option(HARDWARE_AS_SHARED_LIBRARY "Build hardware module as shared library" OFF)
option(BUILD_WRAPPER "Build remote module for hardware access classes" ON)

find_package(Boost REQUIRED COMPONENTS System)
find_package(ALCOMMON REQUIRED)
find_package(ALPROXIES REQUIRED)
find_package(QI REQUIRED)

set(SRCS src/Accelerometer.cpp
        src/Clock.cpp
        src/Gyro.cpp
        src/LEDs.cpp
        src/Joints.cpp
        src/Robot.cpp)

set(HEADERS include/rd/hardware/Accelerometer.h
        include/rd/hardware/Camera.h
        include/rd/hardware/Clock.h
        include/rd/hardware/Sensor.h
        include/rd/hardware/Actuator.h
        include/rd/hardware/SensorData.h
        include/rd/hardware/Gyro.h
        include/rd/hardware/Joints.h
        include/rd/hardware/LEDs.h
        include/rd/hardware/Robot.h)

set(INCLUDE_DIRS ${RD_HARDWARE_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS}
        ${ALCOMMON_INCLUDE_DIRS}
        ${ALPROXIES_INCLUDE_DIRS}
        ${QI_INCLUDE_DIRS})

set(LIBS ${BOOST_LIBRARIES}
        ${ALCOMMON_LIBRARIES}
        ${ALPROXIES_LIBRARILES}
        ${QI_LIBRARIES})

set(RD_HARDWARE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CACHE STRING "Hardware access module includes dir")

set(RD_HARDWARE_LIBRARIES hardware CACHE STRING "Hardware access module include dir")

if (${BUILD_WRAPPER})
    list(APPEND SRCS src/RemoteJoints.cpp)
    list(APPEND HEADERS include/rd/hardware/RemoteJoints.h)
    list(APPEND INCLUDE_DIRS ${RD_NETWORK_INCLUDE_DIRS})
    list(APPEND LIBS ${RD_NETWORK_LIBRARIES})
endif ()

if (${HARDWARE_AS_SHARED_LIBRARY})
    set(LIBRARY_TYPE SHARED)
else ()
    set(LIBRARY_TYPE STATIC)
endif ()

add_library(hardware ${LIBRARY_TYPE} ${SRCS} ${HEADERS})

target_include_directories(hardware PUBLIC ${INCLUDE_DIRS})

target_link_libraries(hardware ${LIBS})