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

set(SRCS src/rd/hardware/Accelerometer.cpp
        src/rd/hardware/Clock.cpp
        src/rd/hardware/Gyro.cpp
        src/rd/hardware/LEDs.cpp
        src/rd/hardware/Joints.cpp
        src/rd/hardware/Robot.cpp
        src/rd/hardware/Camera.cpp
        src/rd/hardware/Kinematics.cpp
        src/rd/hardware/Angle.cpp
        src/rd/hardware/Locomotion.cpp
        src/bhuman/CameraInfo.cpp
        src/bhuman/CameraMatrix.cpp
        src/bhuman/Matrix.cpp
        src/bhuman/RotationMatrix.cpp
        src/bhuman/Transformation.cpp
        src/nb/MetaGait.cpp
        src/nb/NBMath.cpp
        src/nb/NBMatrixMath.cpp
        src/nb/Observer.cpp
        src/nb/Step.cpp
        src/nb/StepGenerator.cpp
        src/nb/WalkingArm.cpp
        src/nb/WalkingLeg.cpp
        src/nb/ZmpAccEKF.cpp
        src/nb/ZmpEKF.cpp
        src/nb/AbstractGait.cpp
        src/nb/Gait.cpp
        src/nb/COMKinematics.cpp
        src/nb/InverseKinematics.cpp
        src/nb/CoordFrame3D.cpp
        src/nb/CoordFrame4D.cpp
        src/nb/SensorAngles.cpp
        src/nb/SpringSensor.cpp)

set(HEADERS
        include/rd/hardware/TypeDefinition.h
        include/rd/hardware/Accelerometer.h
        include/rd/hardware/Angle.h
        include/rd/hardware/Camera.h
        include/rd/hardware/Clock.h
        include/rd/hardware/SensorData.h
        include/rd/hardware/Gyro.h
        include/rd/hardware/Joints.h
        include/rd/hardware/LEDs.h
        include/rd/hardware/Robot.h
        include/rd/hardware/CvImage.h
        include/rd/hardware/Kinematics.h
        include/rd/hardware/Locomotion.h
        include/bhuman/CameraCalibration.h
        include/bhuman/JointCalibration.h
        include/bhuman/MassCalibration.h
        include/bhuman/RobotDimensions.h
        include/bhuman/CameraInfo.h
        include/bhuman/JointData.h
        include/bhuman/CameraMatrix.h
        include/bhuman/BHMath.h
        include/bhuman/Matrix.h
        include/bhuman/Matrix2x2.h
        include/bhuman/Matrix3x3.h
        include/bhuman/Matrix4x4.h
        include/bhuman/Pose2D.h
        include/bhuman/Pose3D.h
        include/bhuman/RotationMatrix.h
        include/bhuman/Transformation.h
        include/bhuman/Vector.h
        include/bhuman/Vector2.h
        include/bhuman/Vector3.h
        include/bhuman/Vector4.h
        include/bhuman/Boundary.h
        include/bhuman/ForwardKinematic.h
        include/bhuman/InverseKinematic.h
        include/bhuman/Range.h
        include/nb/WalkingArm.h
        include/nb/EKF.h
        include/nb/NBMath.h
        include/nb/MotionConstants.h
        include/nb/StepGenerator.h
        include/nb/ZmpEKF.h
        include/nb/Step.h
        include/nb/WalkController.h
        include/nb/ZmpAccEKF.h
        include/nb/MetaGait.h
        include/nb/NBMatrixMath.h
        include/nb/Observer.h
        include/nb/WalkingLeg.h
        include/nb/CoordFrame.h
        include/nb/Structs.h
        include/nb/WalkingConstants.h
        include/nb/GaitConstants.h
        include/nb/Gait.h
        include/nb/AbstractGait.h
        include/nb/COMKinematics.h
        include/nb/Kinematics.h
        include/nb/Common.h
        include/nb/InverseKinematics.h
        include/nb/BasicWorldConstants.h
        include/nb/EKFStructs.h
        include/nb/SensorAngles.h
        include/nb/SpringSensor.h)

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
            src/rd/hardware/RemoteJoints.cpp
            src/rd/hardware/RemoteCamera.cpp
            src/rd/hardware/RemoteLEDs.cpp
            src/rd/hardware/RemoteGyro.cpp
            src/rd/hardware/RemoteAccelerometer.cpp
            src/rd/hardware/RemoteKinematics.cpp
            src/rd/hardware/RemoteAngle.cpp
            src/rd/hardware/RemoteLocomotion.cpp)
    list(APPEND HEADERS
            include/rd/hardware/RemoteJoints.h
            include/rd/hardware/RemoteCamera.h
            include/rd/hardware/RemoteLEDs.h
            include/rd/hardware/RemoteGyro.h
            include/rd/hardware/RemoteAccelerometer.h
            include/rd/hardware/RemoteKinematics.h
            include/rd/hardware/RemoteAngle.h
            include/rd/hardware/RemoteLocomotion.h)
    list(APPEND INCLUDE_DIRS ${RD_NETWORK_INCLUDE_DIRS})
    list(APPEND LIBS ${RD_NETWORK_LIBRARIES})
endif ()

if (${HARDWARE_AS_SHARED_LIBRARY})
    set(LIBRARY_TYPE SHARED)
else ()
    set(LIBRARY_TYPE STATIC)
endif ()


add_library(hardware ${LIBRARY_TYPE} ${SRCS} ${HEADERS})

#add_definitions(-DUSE_COM_CONTROL)

target_include_directories(hardware PUBLIC ${INCLUDE_DIRS})

target_link_libraries(hardware ${LIBS})

install(TARGETS hardware
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)
