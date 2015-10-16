set(RD_SHARED
        TRUE
)

set(RD_TARGET
        KinematicsModule
)

set(RD_SRCS
#        ${Kinematics_MODULE_INCLUDE_DIR}/RD/KinematicsModule/KMat.h
        NAOKinematics.cpp
        NAOKinematicsJacobian.cpp
        KinematicsModule.cpp
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