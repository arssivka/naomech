set(RD_SHARED
        TRUE
)

set(RD_TARGET
        HardwareAccessModule
)

set(RD_SRCS
        HardwareAccessModule.cpp
        main.cpp
)

set(RD_REQUIRED_PACKAGES
        Boost
        ALCOMMON
        ALMEMORYFASTACCESS
        ALPROXIES
        QI
)

SET(RD_INSTALL_DESTINATION
        lib/naoqi
)