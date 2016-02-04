cmake_minimum_required(VERSION 2.8)

option(HARDWARE_AS_SHARED_LIBRARY "Build hardware module as shared library" OFF)
option(BUILD_WRAPPER "Build remote module for hardware access classes" ON)

find_package(BOOST REQUIRED)
find_package(ALCOMMON REQUIRED)
find_package(ALPROXIES REQUIRED)
find_package(QI REQUIRED)
find_package(OPENCV2_CORE REQUIRED)
find_package(OPENCV2_HIGHGUI REQUIRED)
find_package(TBB REQUIRED)

set(SRCS src/hardware/Accelerometer.cpp
        src/hardware/Clock.cpp
        src/hardware/Gyro.cpp
        src/hardware/LEDs.cpp
        src/hardware/Joints.cpp
        src/hardware/Robot.cpp
        src/hardware/Camera.cpp
        src/control/Kinematics.cpp
        /home/arssivka/sources/nao/naomech/hardware/src/bhuman/representations/infrastructure/CameraInfo.cpp
        /home/arssivka/sources/nao/naomech/hardware/src/bhuman/representations/perception/CameraMatrix.cpp
        /home/arssivka/sources/nao/naomech/hardware/src/bhuman/tools/Matrix.cpp
        /home/arssivka/sources/nao/naomech/hardware/src/bhuman/tools/RotationMatrix.cpp
        /home/arssivka/sources/nao/naomech/hardware/src/bhuman/tools/Transformation.cpp)

set(HEADERS include/rd/hardware/Accelerometer.h
        include/rd/hardware/Camera.h
        include/rd/hardware/Clock.h
        include/rd/representation/SensorData.h
        include/rd/hardware/Gyro.h
        include/rd/hardware/Joints.h
        include/rd/hardware/LEDs.h
        include/rd/hardware/Robot.h
        include/rd/representation/CvImage.h
        include/rd/control/Kinematics.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/representations/configuration/CameraCalibration.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/representations/configuration/JointCalibration.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/representations/configuration/MassCalibration.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/representations/configuration/RobotDimensions.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/representations/infrastructure/CameraInfo.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/representations/infrastructure/JointData.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/representations/perception/CameraMatrix.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/BHMath.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Matrix.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Matrix2x2.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Matrix3x3.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Matrix4x4.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Pose2D.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Pose3D.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/RotationMatrix.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Transformation.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Vector.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Vector2.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Vector3.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/math/Vector4.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/Boundary.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/ForwardKinematic.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/InverseKinematic.h
        /home/arssivka/sources/nao/naomech/hardware/include/bhuman/tools/Range.h)

set(INCLUDE_DIRS ${RD_HARDWARE_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS}
        ${ALCOMMON_INCLUDE_DIRS}
        ${ALPROXIES_INCLUDE_DIRS}
        ${QI_INCLUDE_DIRS}
        ${OPENCV2_CORE_INCLUDE_DIRS}
        ${OPENCV2_HIGHGUI_INCLUDE_DIRS}
        ${TBB_INCLUDE_DIRS})

set(LIBS ${BOOST_LIBRARIES}
        ${ALCOMMON_LIBRARIES}
        ${ALPROXIES_LIBRARILES}
        ${QI_LIBRARIES}
        ${OPENCV2_CORE_LIBRARIES}
        ${OPENCV2_HIGHGUI_LIBRARIES}
        ${TBB_LIBRARIES})

set(RD_HARDWARE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CACHE STRING "Hardware access module includes dir")

set(RD_HARDWARE_LIBRARIES hardware CACHE STRING "Hardware access module include dir")

if (${BUILD_WRAPPER})
    list(APPEND SRCS
            src/remote/hardware/RemoteJoints.cpp
            src/remote/hardware/RemoteCamera.cpp
            src/remote/hardware/RemoteLEDs.cpp
            src/remote/hardware/RemoteGyro.cpp
            src/remote/hardware/RemoteAccelerometer.cpp
            src/remote/control/RemoteKinematics.cpp)
    list(APPEND HEADERS
            include/rd/remote/hardware/RemoteJoints.h
            include/rd/remote/hardware/RemoteCamera.h
            include/rd/remote/hardware/RemoteLEDs.h
            include/rd/remote/hardware/RemoteGyro.h
            include/rd/remote/hardware/RemoteAccelerometer.h
            include/rd/remote/control/RemoteKinematics.h)
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

install(TARGETS hardware
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)
