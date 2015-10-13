set(RD_SHARED
        TRUE
)

set(RD_TARGET
        HardwareAccessModule
)

set(RD_SRCS
	V4LRobotCamera.cpp
        HardwareAccessModule.cpp
        main.cpp
)

set(RD_REQUIRED_PACKAGES
        Boost
	OPENCV2_CORE
	OPENCV2_HIGHGUI
        ALCOMMON
        ALMEMORYFASTACCESS
        ALPROXIES
        QI
)

SET(RD_INSTALL_DESTINATION
        lib/naoqi
)
