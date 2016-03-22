#pragma once

#include <bhuman/Pose3D.h>
#include "bhuman/Vector2.h"

#include "bhuman/RobotDimensions.h"
#include "bhuman/JointCalibration.h"
#include "bhuman/CameraCalibration.h"
#include "bhuman/Range.h"
#include "rd/hardware/Kinematics.h"
#include "rd/hardware/Joints.h"
//#include "RD/Logger.hpp"

class InverseKinematic {
public:
    static bool calcLegJoints(const Pose3D &positionLeft,
                              const Pose3D &positionRight,
                              std::vector<double>& jointData,
                              const RobotDimensions &robotDimensions,
                              double ratio = 0.5f) {
        static const RotationMatrix rotPi_4 = RotationMatrix::fromRotationX(
                pi_4);
        static const RotationMatrix rotMinusPi_4 = RotationMatrix::fromRotationX(
                -pi_4);
        static const Range<> ratioClipping(0.0f, 1.0f);
        static const Range<> cosClipping(-1.0f, 1.0f);

        ratio = ratioClipping.limit(ratio);

        const Pose3D lTarget0 = Pose3D(rotMinusPi_4).translate(0.f,
                                                               0.f, 0.f).conc(
                positionLeft);
        const Pose3D rTarget0 = Pose3D(rotPi_4).translate(0.f,
                                                          0.f, 0.f).conc(
                positionRight);
        const Vector3<> lFootToHip =
                lTarget0.rotation.transpose() * (-lTarget0.translation);
        const Vector3<> rFootToHip =
                rTarget0.rotation.transpose() * (-rTarget0.translation);
        const double lMinusJoint5 = std::atan2(lFootToHip.y, lFootToHip.z);
        const double rJoint5 = std::atan2(rFootToHip.y, rFootToHip.z);
        const double lMinusBetaAndJoint4 = -std::atan2(lFootToHip.x, std::sqrt(
                sqr(lFootToHip.y) + sqr(lFootToHip.z)));
        const double rMinusBetaAndJoint4 = -std::atan2(rFootToHip.x, std::sqrt(
                sqr(rFootToHip.y) + sqr(rFootToHip.z)));
        const Vector3<> lHipRotationC1 = lTarget0.rotation *
                                         RotationMatrix::fromRotationX(
                                                 -lMinusJoint5).rotateY(
                                                 -lMinusBetaAndJoint4).c1;
        const Vector3<> rHipRotationC1 = rTarget0.rotation *
                                         RotationMatrix::fromRotationX(
                                                 -rJoint5).rotateY(
                                                 -rMinusBetaAndJoint4).c1;
        const double lMinusJoint0 = std::atan2(-lHipRotationC1.x,
                                               lHipRotationC1.y);
        const double rJoint0 = std::atan2(-rHipRotationC1.x, rHipRotationC1.y);
        const double lJoint0Combined =
                -lMinusJoint0 * ratio + rJoint0 * (1.f - ratio);

        const Pose3D lTarget1 = Pose3D(
                RotationMatrix::fromRotationZ(lJoint0Combined)).conc(lTarget0);
        const Pose3D rTarget1 = Pose3D(
                RotationMatrix::fromRotationZ(-lJoint0Combined)).conc(rTarget0);
        const Vector3<> &lHipToFoot = lTarget1.translation;
        const Vector3<> &rHipToFoot = rTarget1.translation;
        const double lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y,
                                                         -lHipToFoot.z);
        const double rPi_4AndJoint1 = -std::atan2(-rHipToFoot.y, -rHipToFoot.z);
        const double lJoint2MinusAlpha = std::atan2(-lHipToFoot.x, std::sqrt(
                sqr(lHipToFoot.y) + sqr(lHipToFoot.z)) * -sgn(lHipToFoot.z));
        const double rJoint2MinusAlpha = std::atan2(-rHipToFoot.x, std::sqrt(
                sqr(rHipToFoot.y) + sqr(rHipToFoot.z)) * -sgn(rHipToFoot.z));
        const Vector3<> lFootRotationC2 =
                RotationMatrix::fromRotationY(-lJoint2MinusAlpha).rotateX(
                        -lMinusPi_4MinusJoint1) * lTarget1.rotation.c2;
        const Vector3<> rFootRotationC2 =
                RotationMatrix::fromRotationY(-rJoint2MinusAlpha).rotateX(
                        -rPi_4AndJoint1) * rTarget1.rotation.c2;
        const double h1 = robotDimensions.upperLegLength;
        const double h2 = robotDimensions.lowerLegLength;
        const double hl = lTarget1.translation.abs();
        const double hr = rTarget1.translation.abs();
        const double h1Sqr = h1 * h1;
        const double h2Sqr = h2 * h2;
        const double hlSqr = hl * hl;
        const double hrSqr = hr * hr;
        const double lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f * h1 * hl);
        const double rCosMinusAlpha = (h1Sqr + hrSqr - h2Sqr) / (2.f * h1 * hr);
        const double lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f * h2 * hl);
        const double rCosMinusBeta = (h2Sqr + hrSqr - h1Sqr) / (2.f * h2 * hr);
        const double lAlpha = -std::acos(cosClipping.limit(lCosMinusAlpha));
        const double rAlpha = -std::acos(cosClipping.limit(rCosMinusAlpha));
        const double lBeta = -std::acos(cosClipping.limit(lCosMinusBeta));
        const double rBeta = -std::acos(cosClipping.limit(rCosMinusBeta));

        jointData[rd::Joints::L_HIP_YAW_PITCH + 0] = lJoint0Combined;
        jointData[rd::Joints::L_HIP_YAW_PITCH + 1] = -(
                lMinusPi_4MinusJoint1 + pi_4);
        jointData[rd::Joints::L_HIP_YAW_PITCH + 2] =
                lJoint2MinusAlpha + lAlpha;
        jointData[rd::Joints::L_HIP_YAW_PITCH + 3] = -lAlpha - lBeta;
        jointData[rd::Joints::L_HIP_YAW_PITCH + 4] =
                std::atan2(lFootRotationC2.x, lFootRotationC2.z) + lBeta;
        jointData[rd::Joints::L_HIP_YAW_PITCH + 5] = -std::asin(
                -lFootRotationC2.y);

        jointData[rd::Joints::R_HIP_YAW_PITCH + 0] = lJoint0Combined;
        jointData[rd::Joints::R_HIP_YAW_PITCH + 1] = rPi_4AndJoint1 - pi_4;
        jointData[rd::Joints::R_HIP_YAW_PITCH + 2] =
                rJoint2MinusAlpha + rAlpha;
        jointData[rd::Joints::R_HIP_YAW_PITCH + 3] = -rAlpha - rBeta;
        jointData[rd::Joints::R_HIP_YAW_PITCH + 4] =
                std::atan2(rFootRotationC2.x, rFootRotationC2.z) + rBeta;
        jointData[rd::Joints::R_HIP_YAW_PITCH + 5] = std::asin(
                -rFootRotationC2.y);

        const double maxLen = h1 + h2;
        return hl < maxLen && hr < maxLen;
    }

    static bool calcLegJoints(const Pose3D &position,
                              std::vector<double>& jointData,
                              bool left,
                              const RobotDimensions &robotDimensions) {
        Pose3D target(position);
        rd::Joints::Key firstJoint(
                left ? rd::Joints::L_HIP_YAW_PITCH : rd::Joints::R_HIP_YAW_PITCH);
        int sign(left ? -1 : 1);
        //target.translation.y += robotDimensions.lengthBetweenLegs / 2.f *
        //                      sign;
        static const double sqrt2_2 = std::sqrt(2.0f) * 0.5f;
        RotationMatrix rotationX_pi_4 = RotationMatrix(Vector3<>(1, 0, 0), Vector3<>(0, sqrt2_2, sqrt2_2 * sign),
                                                       Vector3<>(0, sqrt2_2 * -sign, sqrt2_2));
        target.translation = rotationX_pi_4 * target.translation;
        target.rotation = rotationX_pi_4 * target.rotation;

        target = target.invert();

        double length = target.translation.abs();
        double sqrLength = length * length;
        double upperLeg = robotDimensions.upperLegLength;
        double sqrUpperLeg = upperLeg * upperLeg;
        double lowerLeg = robotDimensions.lowerLegLength;
        double sqrLowerLeg = lowerLeg * lowerLeg;
        double cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * lowerLeg * length);
        double cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);

        const Range<> clipping(-1.0f, 1.0f);
        bool reachable = true;
        if (!clipping.isInside(cosKnee) || clipping.isInside(cosLowerLeg)) {
            cosKnee = clipping.limit(cosKnee);
            cosLowerLeg = clipping.limit(cosLowerLeg);
            reachable = false;
        }
        double joint3 = pi - std::acos(cosKnee);
        double joint4 = -std::acos(cosLowerLeg);
        joint4 -= std::atan2(target.translation.x, Vector2<>(target.translation.y, target.translation.z).abs());
        double joint5 = std::atan2(target.translation.y, target.translation.z) * sign;

        RotationMatrix hipFromFoot;
        hipFromFoot.rotateX(joint5 * -sign);
        hipFromFoot.rotateY(-joint4 - joint3);

        RotationMatrix hip = hipFromFoot.invert() * target.rotation;

        double joint1 = std::asin(-hip[2].y) * -sign;
        joint1 -= pi_4;
        double joint2 = -std::atan2(hip[2].x, hip[2].z);
        double joint0 = std::atan2(hip[0].y, hip[1].y) * -sign;

        jointData[firstJoint + 0] = joint0;
        jointData[firstJoint + 1] = joint1;
        jointData[firstJoint + 2] = joint2;
        jointData[firstJoint + 3] = joint3;
        jointData[firstJoint + 4] = joint4;
        jointData[firstJoint + 5] = joint5;

        return reachable;
    }

    static bool calcLegJoints(const Pose3D &position,
                              std::vector<double>& jointData,
                              double joint0, bool left,
                              const RobotDimensions &robotDimensions) {
        Pose3D target(position);
        rd::Joints::Key firstJoint(left ? rd::Joints::L_HIP_YAW_PITCH : rd::Joints::R_HIP_YAW_PITCH);
        const int sign(left ? -1 : 1);
        //target.translation.y += robotDimensions.lengthBetweenLegs / 2 *
        //                        sign;
        target = Pose3D().rotateZ(joint0 * -sign).rotateX(pi_4 * sign).conc(target);

        double length = target.translation.abs();
        double sqrLength = length * length;
        double upperLeg = robotDimensions.upperLegLength;
        double sqrUpperLeg = upperLeg * upperLeg;
        double lowerLeg = robotDimensions.lowerLegLength;
        double sqrLowerLeg = lowerLeg * lowerLeg;
        double cosUpperLeg = (sqrUpperLeg + sqrLength - sqrLowerLeg) / (2 * upperLeg * length);
        double cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);
        const Range<> clipping(-1.0f, 1.0f);
        bool reachable = true;
        if (!clipping.isInside(cosKnee) || clipping.isInside(upperLeg)) {
            cosKnee = clipping.limit(cosKnee);
            cosUpperLeg = clipping.limit(cosUpperLeg);
            reachable = false;
        }
        double joint1 =
                target.translation.z == 0.0f ? 0.0f : std::atan(target.translation.y / -target.translation.z) * sign;
        double joint2 = -std::acos(cosUpperLeg);
        joint2 -= std::atan2(target.translation.x,
                             Vector2<>(target.translation.y, target.translation.z).abs() * -sgn(target.translation.z));
        double joint3 = pi - std::acos(cosKnee);
        RotationMatrix beforeFoot = RotationMatrix().rotateX(joint1 * sign).rotateY(joint2 + joint3);
        joint1 -= pi_4;

        RotationMatrix foot = beforeFoot.invert() * target.rotation;
        double joint5 = std::asin(-foot[2].y) * -sign * -1;
        double joint4 = -std::atan2(foot[2].x, foot[2].z) * -1;

        jointData[firstJoint + 0] = joint0;
        jointData[firstJoint + 1] = joint1;
        jointData[firstJoint + 2] = joint2;
        jointData[firstJoint + 3] = joint3;
        jointData[firstJoint + 4] = joint4;
        jointData[firstJoint + 5] = joint5;

        return reachable;
    }

public:
    static bool calcArmJoints(const Pose3D &left, const Pose3D &right,
                              std::vector<double>& targetJointData,
                              const RobotDimensions &theRobotDimensions,
                              const JointCalibration &theJointCalibration) {
        const Vector3<> leftDir = left.rotation * Vector3<>(0, -1, 0),
                rightDir = right.rotation * Vector3<>(0, -1, 0);


        Vector3<> leftTarget =
                left.translation,// - Vector3<>(theRobotDimensions.armOffset.x, //commetns
        //          theRobotDimensions.armOffset.y,
        //        theRobotDimensions.armOffset.z),
                rightTarget =
                right.translation;// - Vector3<>(theRobotDimensions.armOffset.x,
        //          -theRobotDimensions.armOffset.y,
        //        theRobotDimensions.armOffset.z);

        static const double maxLength = (theRobotDimensions.upperArmLength +
                                         theRobotDimensions.lowerArmLength) *
                                        0.9999f;
        if (leftTarget.squareAbs() >= sqr(maxLength))
            leftTarget.normalize(maxLength);

        if (rightTarget.squareAbs() >= sqr(maxLength))
            rightTarget.normalize(maxLength);

        bool res1, res2;
        res1 = calcArmJoints(leftTarget, leftDir, 1, targetJointData,
                             theRobotDimensions, theJointCalibration);
        res2 = calcArmJoints(rightTarget, rightDir, -1, targetJointData,
                             theRobotDimensions, theJointCalibration);
        targetJointData[rd::Joints::R_SHOULDER_PITCH + 1] *= -1;
        targetJointData[rd::Joints::R_SHOULDER_PITCH + 3] *= -1;
        /*LOG << "left arm: " << targetJointData[rd::Joints::L_SHOULDER_PITCH] << " " << targetJointData[rd::Joints::L_SHOULDER_PITCH + 1] << " "
                                                        << targetJointData[rd::Joints::L_SHOULDER_PITCH + 2] << " " <<
                                                                            targetJointData[rd::Joints::L_SHOULDER_PITCH + 3] << " ";

        LOG << "right arm: " << targetJointData[rd::Joints::R_SHOULDER_PITCH] << " " << targetJointData[rd::Joints::R_SHOULDER_PITCH + 1] << " "
                                                        << targetJointData[rd::Joints::R_SHOULDER_PITCH + 2] << " " <<
                                                                            targetJointData[rd::Joints::R_SHOULDER_PITCH + 3] << " ";*/


        return res1 && res2;
    }

    static bool calcArmJoints(Vector3<> target, Vector3<> targetDir, int side,
                              std::vector<double>& targetJointData,
                              const RobotDimensions &theRobotDimensions,
                              const JointCalibration &theJointCalibration) {
        //target.y *= (double) side;
        //targetDir.y *= (double) side;

        const int offset = side == 1 ? rd::Joints::L_SHOULDER_PITCH : rd::Joints::R_SHOULDER_PITCH;
        Vector3<> elbow;
        if (!calcElbowPosition(target, targetDir, side, elbow,
                               theRobotDimensions, theJointCalibration))
            return false;
        calcJointsForElbowPos(elbow, target, targetJointData, offset,
                              theRobotDimensions);

        return true;
    }

    static bool calcElbowPosition(Vector3<> &target, const Vector3<> &targetDir,
                                  int side, Vector3<> &elbow,
                                  const RobotDimensions &theRobotDimensions,
                                  const JointCalibration &theJointCalibration) {
        const Vector3<> M1(0, 0, 0);
        const Vector3<> M2(target);
        const double r1 = theRobotDimensions.upperArmLength;
        const double r2 = theRobotDimensions.lowerArmLength;
        const Vector3<> M12 = M2 - M1;

        Vector3<> n = target;
        n.normalize();

        const Vector3<> M3 =
                M1 + M12 * ((sqr(r1) - sqr(r2)) / (2 * M12.squareAbs()) + 0.5f);

        const Vector3<> M23 = M3 - M2;
        double diff = sqr(r2) - M23.squareAbs();
        const double radius = std::sqrt(diff);

        const bool specialCase =
                n.x == 1 && n.y == 0 && n.z == 0 ? true : false;
        const Vector3<> bla(specialCase ? 0.0f : 1.0f,
                            specialCase ? 1.0f : 0.0f, 0.0f);
        const Vector3<> pointOnCircle = M3 + (n ^ bla).normalize(radius);

        double angleDiff = pi * 2.0f / 3.0f;
        Vector3<> bestMatch = pointOnCircle;
        double bestAngle = 0.0f;
        double newBestAngle = bestAngle;
        double bestQuality = -2.0f;

        Vector3<> tDir = targetDir;
        tDir.normalize();

        //const int offset = side == 1 ? 0 : rd::Joints::R_SHOULDER_PITCH - rd::Joints::L_SHOULDER_PITCH;
        int iterationCounter = 0;
        const double maxAngleEpsilon = 1.0f * pi / 180.0f;
        while (2.0f * angleDiff > maxAngleEpsilon) {
            for (int i = -1; i <= 1; i++) {
                if (i == 0 && iterationCounter != 1)
                    continue;

                iterationCounter++;

                const Pose3D elbowRotation(
                        RotationMatrix(n, bestAngle + angleDiff * i));
                const Vector3<> possibleElbow = elbowRotation * pointOnCircle;
                const Vector3<> elbowDir = (M3 - possibleElbow).normalize();
                double quality = elbowDir * tDir;
                if (quality > bestQuality) {
                    bestQuality = quality;
                    bestMatch = possibleElbow;
                    newBestAngle = bestAngle + angleDiff * i;
                }
            }
            angleDiff /= 2.0f;
            bestAngle = newBestAngle;
        }
        if (bestQuality == -2.0f)
            return false;

        /*std::vector<double> tAJR(rd::Joints::NUM_OF_JOINTS);
        calcJointsForElbowPos(bestMatch, target, tAJR, offset,
                              theRobotDimensions);

        const JointCalibration::JointInfo ji = theJointCalibration.joints[
                rd::Joints::L_SHOULDER_PITCH + offset + 1];
        if (tAJR[rd::Joints::L_SHOULDER_PITCH + offset + 1] < ji.minAngle) {
            tAJR[rd::Joints::L_SHOULDER_PITCH + offset + 1] = ji.minAngle;
            Pose3D shoulder2Elbow;
            shoulder2Elbow.translate(0, -theRobotDimensions.upperArmLength, 0);
            shoulder2Elbow.rotateX(-(tAJR[rd::Joints::L_SHOULDER_PITCH + offset + 1] - pi / 2.0f));
            shoulder2Elbow.rotateY(tAJR[rd::Joints::L_SHOULDER_PITCH + offset + 0] + pi / 2.0f);
            Vector3<> handInEllbow = shoulder2Elbow * target;

            handInEllbow.normalize(theRobotDimensions.lowerArmLength);
            target = shoulder2Elbow.invert() * handInEllbow;
            bestMatch = shoulder2Elbow.invert() * Vector3<>(0, 0, 0);
        }*/

        elbow = bestMatch;
        return true;
    }

    static void calcJointsForElbowPos(const Vector3<> &elbow,
                                      const Vector3<> &target,
                                      std::vector<double>& targetJointData,
                                      int offset,
                                      const RobotDimensions &theRobotDimensions) {
        targetJointData[offset + 0] = std::atan2(elbow.z, elbow.x);
        targetJointData[offset + 1] = std::atan2(elbow.y, std::sqrt(
                sqr(elbow.x) + sqr(elbow.z)));
        //calculate desired elbow "klapp"-angle
        const double c = target.abs(),
                a = theRobotDimensions.upperArmLength,
                b = theRobotDimensions.lowerArmLength;

        double cosAngle = (-sqr(c) + sqr(b) + sqr(a)) / (2.0f * a * b);
        if (cosAngle < -1.0f) {
            cosAngle = -1.0f;
        }
        else if (cosAngle > 1.0f) {
            cosAngle = 1.0f;
        }
        targetJointData[offset + 3] = std::acos(cosAngle) - pi;

        Pose3D shoulder2Elbow;
        shoulder2Elbow.translate(0, -theRobotDimensions.upperArmLength, 0);
        shoulder2Elbow.rotateX(-(targetJointData[offset + 1] - pi / 2.0f));
        shoulder2Elbow.rotateY(targetJointData[offset + 0] + pi / 2.0f);
        const Vector3<> handInEllbow = shoulder2Elbow * target;

        targetJointData[offset + 2] = -(
                std::atan2(handInEllbow.z, handInEllbow.x) + pi / 2.f);
        while (targetJointData[offset + 2] > pi)
            targetJointData[offset + 2] -= 2 * pi;
        while (targetJointData[offset + 2] < -pi)
            targetJointData[offset + 2] += 2 * pi;
    }

    static void calcArmJoints(const Vector3<>& position, const double elbowYaw,
                              std::vector<double>& jointData, bool left,
                              const RobotDimensions &robotDimensions) {
        rd::Joints::Key firstJoint(
                left ? rd::Joints::L_SHOULDER_PITCH : rd::Joints::R_SHOULDER_PITCH);
        const int sign(left ? -1 : 1);
        /*const Vector3<> pos(position - Vector3<>(robotDimensions.armOffset.x,
                                                 robotDimensions.armOffset.y *
                                                 -sign,
                                                 robotDimensions.armOffset.z));*/
        const Vector3<> pos(position.x, position.y * sign, position.z);
        double& joint0 = jointData[firstJoint + 0];
        double& joint1 = jointData[firstJoint + 1];
        const double& joint2 = jointData[firstJoint + 2] = elbowYaw;
        double& joint3 = jointData[firstJoint + 3];

        const double positionAbs = pos.abs();

        const double actualUpperArmLength = Vector2<>(robotDimensions.upperArmLength,
                                                      robotDimensions.yElbowShoulder).abs();
        double cosElbow = (sqr(actualUpperArmLength) + sqr(robotDimensions.lowerArmLength) - sqr(positionAbs)) /
                          (2.0f * robotDimensions.upperArmLength * robotDimensions.lowerArmLength);
        cosElbow = Range<>(-1.0f, 1.0f).limit(cosElbow);
        joint3 = -(pi - std::acos(cosElbow));
        const Pose3D tempPose = Pose3D(robotDimensions.upperArmLength, robotDimensions.yElbowShoulder * -sign,
                                       0).rotateX(joint2 * -sign).rotateZ(joint3 * -sign).translate(
                robotDimensions.lowerArmLength, 0, 0);

        joint1 = std::atan2(tempPose.translation.y * sign, tempPose.translation.x) +
                 std::asin(pos.y / Vector2<>(tempPose.translation.x, tempPose.translation.y).abs()) * -sign;

        const Pose3D tempPose2 = Pose3D().rotateZ(joint1 * -sign).conc(
                tempPose);

        joint0 = -std::atan2(tempPose2.translation.z, tempPose2.translation.x) +
                 std::atan2(pos.z, pos.x);
    }

    static void calcHeadJoints(const Vector3<>& position, const double imageTilt,
                               const RobotDimensions &robotDimensions,
                               const bool topCamera, std::vector<double>& targetJointData,
                               const CameraCalibration &cameraCalibration) {
        Vector2<> headJoint2Target(std::sqrt(sqr(position.x) + sqr(position.y)),
                                   position.z - robotDimensions.zLegJoint1ToHeadPan);
        Vector2<> headJoint2Camera(robotDimensions.getXHeadTiltToCamera(topCamera),
                                   robotDimensions.getZHeadTiltToCamera(topCamera));
        const double headJoint2CameraAngle = std::atan2(headJoint2Camera.x,
                                                       headJoint2Camera.y);
        const double cameraAngle =
                pi3_2 - imageTilt - (pi_2 - headJoint2CameraAngle) - robotDimensions.getHeadTiltToCameraTilt(topCamera);
        const double targetAngle = std::asin(headJoint2Camera.abs() * std::sin(cameraAngle) / headJoint2Target.abs());
        const double headJointAngle = pi - targetAngle - cameraAngle;
        const double tilt = std::atan2(headJoint2Target.x, headJoint2Target.y) - headJointAngle - headJoint2CameraAngle;
        targetJointData[rd::Joints::HEAD_PITCH] = tilt;
        targetJointData[rd::Joints::HEAD_YAW] = std::atan2(position.y, position.x);
    }
};
