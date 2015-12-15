find_package(BOOST REQUIRED COMPONENTS System)
find_package(OPENCV2_CORE REQUIRED)
find_package(OPENCV2_HIGHGUI REQUIRED)
find_package(ALMEMORYFASTACCESS)
find_package(ALVALUE REQUIRED)
find_package(ALERROR REQUIRED)
find_package(ALCOMMON REQUIRED)
find_package(ALPROXIES REQUIRED)
find_package(QI REQUIRED)

set(SRCS src/main.cpp
        src/HWController.cpp)

set(HEADERS include/rd/naoqi/hwcontroller/HWController.h)

set(INCLUDE_DIRS ${RD_NAOQI_HWCONTROLLER_INCLUDE_DIRS}
        ${ALMEMORYFASTACCESS_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS}
        ${OPENCV2_CORE_INCLUDE_DIRS}
        ${OPENCV2_HIGHGUI_INCLUDE_DIRS}
        ${ALVALUE_INCLUDE_DIRS}
        ${ALERROR_INCLUDE_DIRS}
        ${ALCOMMON_INCLUDE_DIRS}
        ${ALPROXIES_INCLUDE_DIRS}
        ${QI_INCLUDE_DIRS})

set(LIBS ${BOOST_LIBRARIES}
        ${OPENCV2_CORE_LIBRARIES}
        ${OPENCV2_HIGHGUI_LIBRARIES}
        ${ALMEMORYFASTACCESS_LIBRARIES}
        ${ALVALUE_LIBRARIES}
        ${ALERROR_LIBRARIES}
        ${ALCOMMON_LIBRARIES}
        ${ALPROXIES_LIBRARIES}
        ${QI_LIBRARIES})

set(RD_NAOQI_HWCONTROLLER_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CACHE STRING "Hardware access module includes dir")

add_library(hwcontroller SHARED ${SRCS} ${HEADERS})

target_include_directories(hwcontroller PUBLIC ${INCLUDE_DIRS})

target_link_libraries(hwcontroller ${LIBS})

install(TARGETS hwcontroller
        LIBRARY DESTINATION lib)