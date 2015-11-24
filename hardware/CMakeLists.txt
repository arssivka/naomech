cmake_minimum_required(VERSION 2.8)

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

file(GLOB_RECURSE HEADERS include/*.h include/*.hpp)

add_library(hardware SHARED ${SRCS} ${HEADERS})

target_include_directories(hardware PUBLIC
        include
        ${Boost_INCLUDE_DIRS}
        ${ALCOMMON_INCLUDE_DIRS}
        ${ALPROXIES_INCLUDE_DIRS}
        ${QI_INCLUDE_DIRS})

target_link_libraries(hardware
        ${Boost_LIBRARIES}
        ${ALCOMMON_LIBRARIES}
        ${ALPROXIES_LIBRARILES}
        ${QI_LIBRARIES})