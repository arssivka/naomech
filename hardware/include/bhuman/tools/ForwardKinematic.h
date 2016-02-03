#pragma once

#include "bhuman/tools/math/Pose3D.h"
#include "bhuman/tools/math/BHMath.h"
#include "bhuman/representations/configuration/RobotDimensions.h"
#include "bhuman/representations/configuration/MassCalibration.h"
#include "rd/hardware/Joints.h"

class ForwardKinematic {
public:
    static void calculateArmChain(bool left, std::vector<double>& jointData,
                                  const RobotDimensions &robotDimensions,
                                  const MassCalibration &massCalibration,
                                  Pose3D limbs[MassCalibration::numOfLimbs]) {
        int sign = left ? -1 : 1;
        MassCalibration::Limb shoulder = left ? MassCalibration::shoulderLeft
                                              : MassCalibration::shoulderRight;
        int arm0 = left ? rd::Joints::L_SHOULDER_PITCH : rd::Joints::R_SHOULDER_PITCH;

        limbs[shoulder + 0] = Pose3D(robotDimensions.armOffset.x,
                                     robotDimensions.armOffset.y * -sign,
                                     robotDimensions.armOffset.z).rotateY(-jointData[arm0]);
        limbs[shoulder + 1] = Pose3D(limbs[shoulder])
                .rotateZ(jointData[arm0 + 1] * -sign);
        limbs[shoulder + 2] = Pose3D(limbs[shoulder + 1])
                .translate(robotDimensions.upperArmLength, robotDimensions.yElbowShoulder * -sign, 0)
                .rotateX(jointData[arm0 + 2] * -sign);
        limbs[shoulder + 3] = Pose3D(limbs[shoulder + 2])
                .rotateZ(jointData[arm0 + 3] * -sign);
    }

    static void calculateLegChain(bool left, std::vector<double>& jointData,
                                  const RobotDimensions &robotDimensions,
                                  const MassCalibration &massCalibration,
                                  Pose3D limbs[MassCalibration::numOfLimbs]) {
        int sign = 1;//left ? -1 : 1;
        MassCalibration::Limb pelvis = left ? MassCalibration::pelvisLeft
                                            : MassCalibration::pelvisRight;
        rd::Joints::Key leg0 = left ? rd::Joints::L_HIP_YAW_PITCH : rd::Joints::R_HIP_YAW_PITCH;

        limbs[pelvis + 0] = Pose3D(0, 0, 0)
                .rotateX(-pi_4 * sign)
                .rotateZ(jointData[leg0 + 0] * sign)
                .rotateX(pi_4 * sign);
        limbs[pelvis + 1] = Pose3D(limbs[pelvis + 0])
                .rotateX(jointData[leg0 + 1] * sign);
        limbs[pelvis + 2] = Pose3D(limbs[pelvis + 1])
                .rotateY(jointData[leg0 + 2]);
        limbs[pelvis + 3] = Pose3D(limbs[pelvis + 2])
                .translate(0, 0, -robotDimensions.upperLegLength)
                .rotateY(jointData[leg0 + 3]);
        limbs[pelvis + 4] = Pose3D(limbs[pelvis + 3])
                .translate(0, 0, -robotDimensions.lowerLegLength)
                .rotateY(jointData[leg0 + 4]);
        limbs[pelvis + 5] = Pose3D(limbs[pelvis + 4])
                .rotateX(jointData[leg0 + 5] * sign);
    }

    static void calculateHeadChain(std::vector<double>& jointData,
                                   const RobotDimensions &robotDimensions,
                                   const MassCalibration &massCalibration,
                                   Pose3D limbs[MassCalibration::numOfLimbs]) {
        limbs[MassCalibration::neck] = Pose3D(0, 0, robotDimensions.zLegJoint1ToHeadPan)
                .rotateZ(jointData[rd::Joints::HEAD_YAW]);
        limbs[MassCalibration::head] = Pose3D(limbs[MassCalibration::neck])
                .rotateY(-jointData[rd::Joints::HEAD_PITCH]);
    }
};
