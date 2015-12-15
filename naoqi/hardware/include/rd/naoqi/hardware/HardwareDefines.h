//
// Created by arssivka on 10/26/15.
//

#ifndef NAOMECH_HARDWAREDEFINES_H

namespace RD {
    enum SensorType {
        HEAD_YAW, HEAD_PITCH,
        L_SHOULDER_PITCH,
        L_SHOULDER_ROLL,
        L_ELBOW_YAW,
        L_ELBOW_ROLL,
        L_WRIST_YAW,
        L_HAND,
        L_HIP_YAW_PITCH,
        L_HIP_ROLL,
        L_HIP_PITCH,
        L_KNEE_PITCH,
        L_ANKLE_PITCH,
        L_ANKLE_ROLL,
        R_HIP_YAW_PITCH,
        R_HIP_ROLL,
        R_HIP_PITCH,
        R_KNEE_PITCH,
        R_ANKLE_PITCH,
        R_ANKLE_ROLL,
        R_SHOULDER_PITCH,
        R_SHOULDER_ROLL,
        R_ELBOW_YAW,
        R_ELBOW_ROLL,
        R_WRIST_YAW,
        R_HAND,
        ACC_X,
        ACC_Y,
        ACC_Z,
        GYR_X,
        GYR_Y,
        ANGLE_X,
        ANGLE_Y,
        L_COP_X,
        L_COP_Y,
        L_TOTAL_WEIGHT,
        R_COP_X,
        R_COP_Y,
        R_TOTAL_WEIGHT,
        NUM_OF_SENSORS
    };

    typedef int JointId;
    typedef float JointData;
    typedef std::vector <JointData> JointContainer;

    const int NUM_OF_JOINTS = R_HAND + 1;
}

#define NAOMECH_HARDWAREDEFINES_H

#endif //NAOMECH_HARDWAREDEFINES_H
