//
// Created by arssivka on 10/5/15.
//

#include "RD/KinematicsModule/KinematicsModule.h"

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <almemoryfastaccess/almemoryfastaccess.h>
#include <alproxies/altexttospeechproxy.h>

#include "Representations/Configuration/MassCalibration.h"
#include "RD/KinematicsModule/KinematicsDefines.h"
#include "Tools/InverseKinematic.h"
#include "Tools/ForwardKinematic.h"

using namespace RD;
using namespace boost;
using namespace std;


////////////////////////////////////////////////////////////////////////////////

KinematicsModule::KinematicsModule(shared_ptr<AL::ALBroker> pBroker,
                                   const string &pName)
        : AL::ALModule(pBroker, pName), mem(new AL::ALMemoryFastAccess()),
          effectors(NUM_OF_EFFECTORS),
          effectors_mask(NUM_OF_EFFECTORS, false) {
    this->setModuleDescription("NAO Kinematics module.");


    this->functionName("update", this->getName(),
                       "update kinematics chains by sensor values");
    BIND_METHOD(KinematicsModule::update);

    this->functionName("apply", this->getName(),
                       "Sent joint data to hardware access module");
    BIND_METHOD(KinematicsModule::apply);

    this->functionName("getJoints", this->getName(),
                       "get data for all joints");
    this->setReturn("values", "return values for all joints");
    BIND_METHOD(KinematicsModule::getJoints);

    this->functionName("lookAt", this->getName(),
                       "set head position and orientation in space");
    this->addParam("pos", "position of object in the robot frame");
    this->addParam("tot_camera", "true - position for top camera, false - bot camera");
    BIND_METHOD(KinematicsModule::lookAt);

    this->functionName("setLeftArmPosition", this->getName(),
                       "set left hand position and orientation in space");
    this->addParam("pos", "position and orientation");
    BIND_METHOD(KinematicsModule::setLeftArmPosition);

    this->functionName("setRightArmPosition", this->getName(),
                       "set right hand position and orientation in space");
    this->addParam("pos", "position and orientation");
    BIND_METHOD(KinematicsModule::setRightArmPosition);

    this->functionName("setLeftLegPosition", this->getName(),
                       "set left leg position and orientation in space");
    this->addParam("pos", "position and orientation");
    BIND_METHOD(KinematicsModule::setLeftLegPosition);

    this->functionName("setLeftLegPositionWithZ", this->getName(),
                       "set left leg position and orientation in space");
    this->addParam("pos", "position and orientation");
    this->addParam("joint0", "fixed velue of joint 0");
    BIND_METHOD(KinematicsModule::setLeftLegPositionWithZ);

    this->functionName("setRightLegPositionWithZ", this->getName(),
                       "set right leg position and orientation in space");
    this->addParam("pos", "position and orientation");
    this->addParam("joint0", "fixed velue of joint 0");
    BIND_METHOD(KinematicsModule::setRightLegPositionWithZ);

    this->functionName("setRightLegPosition", this->getName(),
                       "set right leg position and orientation in space");
    this->addParam("pos", "position and orientation");
    BIND_METHOD(KinematicsModule::setRightLegPosition);

    this->functionName("getHeadPosition", this->getName(),
                       "get center of mass");
    this->addParam("top_camera", "true - position for top camera, false - bottom camera");
    this->setReturn("values", "get head position and orientation in space");
    BIND_METHOD(KinematicsModule::getHeadPosition);

    this->functionName("getLeftArmPosition", this->getName(),
                       "get center of mass");
    this->setReturn("values", "get left hand position and orientation in space");
    BIND_METHOD(KinematicsModule::getLeftArmPosition);

    this->functionName("getLeftLegPosition", this->getName(),
                       "get center of mass");
    this->setReturn("values", "get left leg position and orientation in space");
    BIND_METHOD(KinematicsModule::getLeftLegPosition);

    this->functionName("getRightArmPosition", this->getName(),
                       "get center of mass");
    this->setReturn("values", "get right hand position and orientation in space");
    BIND_METHOD(KinematicsModule::getRightArmPosition);

    this->functionName("getRightLegPosition", this->getName(),
                       "get center of mass");
    this->setReturn("values", "get right leg position and orientation in space");
    BIND_METHOD(KinematicsModule::getRightLegPosition);

    //TODO And this should be deleted! Delete this all! Waaaagh!
    jc.joints[0].offset = 0;
    jc.joints[0].sign = 1;
    jc.joints[0].maxAngle = -120;
    jc.joints[0].minAngle = 120;

    jc.joints[1].offset = 0;
    jc.joints[1].sign = -1;
    jc.joints[1].maxAngle = -30;
    jc.joints[1].minAngle = 40;

    jc.joints[2].offset = 0;
    jc.joints[2].sign = -1;
    jc.joints[2].maxAngle = -120;
    jc.joints[2].minAngle = 120;

    jc.joints[3].offset = 0;
    jc.joints[3].sign = 1;
    jc.joints[3].maxAngle = 0;
    jc.joints[3].minAngle = 90;

    jc.joints[4].offset = 0;
    jc.joints[4].sign = 1;
    jc.joints[4].maxAngle = -120;
    jc.joints[4].minAngle = 120;

    jc.joints[5].offset = 0;
    jc.joints[5].sign = 1;
    jc.joints[5].maxAngle = -90;
    jc.joints[5].minAngle = 0;

    jc.joints[6].offset = 0;
    jc.joints[6].sign = -1;
    jc.joints[6].maxAngle = -120;
    jc.joints[6].minAngle = 120;

    jc.joints[7].offset = 0;
    jc.joints[7].sign = -1;
    jc.joints[7].maxAngle = 0;
    jc.joints[7].minAngle = 90;

    jc.joints[8].offset = 0;
    jc.joints[8].sign = -1;
    jc.joints[8].maxAngle = -120;
    jc.joints[8].minAngle = 120;

    jc.joints[9].offset = 0;
    jc.joints[9].sign = -1;
    jc.joints[9].maxAngle = -90;
    jc.joints[9].minAngle = 0;

    jc.joints[10].offset = 0;
    jc.joints[10].sign = 1;
    jc.joints[10].maxAngle = -65;
    jc.joints[10].minAngle = 43;

    jc.joints[11].offset = 0;
    jc.joints[11].sign = -1;
    jc.joints[11].maxAngle = -41;
    jc.joints[11].minAngle = 23;

    jc.joints[12].offset = 0;
    jc.joints[12].sign = 1;
    jc.joints[12].maxAngle = -92;
    jc.joints[12].minAngle = 21;

    jc.joints[13].offset = 0;
    jc.joints[13].sign = 1;
    jc.joints[13].maxAngle = -5;
    jc.joints[13].minAngle = 120;

    jc.joints[14].offset = 0;
    jc.joints[14].sign = 1;
    jc.joints[14].maxAngle = -68;
    jc.joints[14].minAngle = 52;

    jc.joints[15].offset = 0;
    jc.joints[15].sign = -1;
    jc.joints[15].maxAngle = -23;
    jc.joints[15].minAngle = 27;

    jc.joints[16].offset = 0;
    jc.joints[16].sign = 1;
    jc.joints[16].maxAngle = -65;
    jc.joints[16].minAngle = 43;

    jc.joints[17].offset = 0;
    jc.joints[17].sign = 1;
    jc.joints[17].maxAngle = -41;
    jc.joints[17].minAngle = 23;

    jc.joints[18].offset = 0;
    jc.joints[18].sign = 1;
    jc.joints[18].maxAngle = -92;
    jc.joints[18].minAngle = 21;

    jc.joints[19].offset = 0;
    jc.joints[19].sign = 1;
    jc.joints[19].maxAngle = -5;
    jc.joints[19].minAngle = 120;

    jc.joints[20].offset = 0;
    jc.joints[20].sign = 1;
    jc.joints[20].maxAngle = -68;
    jc.joints[20].minAngle = 52;

    jc.joints[21].offset = 0;
    jc.joints[21].sign = 1;
    jc.joints[21].maxAngle = -23;
    jc.joints[21].minAngle = 27;

    rd.lengthBetweenLegs = 100;
    rd.upperLegLength = 100;
    rd.lowerLegLength = 102.9;
    rd.heightLeg5Joint = 45.19;
    rd.zLegJoint1ToHeadPan = 211.5;
    rd.xHeadTiltToCamera = 50.71;
    rd.zHeadTiltToCamera = 17.74;
    rd.headTiltToCameraTilt = 0.692895713; // 39.7 degree
    rd.xHeadTiltToUpperCamera = 58.71;
    rd.zHeadTiltToUpperCamera = 63.64;
    rd.headTiltToUpperCameraTilt = 0.020943951; // 1.2 degree
    rd.armOffset.x = 0;
    rd.armOffset.y = 98;
    rd.armOffset.z = 185;
    rd.yElbowShoulder = 15;
    rd.upperArmLength = 105;
    rd.lowerArmLength = 130; // estimated
}

////////////////////////////////////////////////////////////////////////////////

KinematicsModule::~KinematicsModule() { }

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::init() {
    this->initFastAccess();
    this->initHW();
    this->update();
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::update() {
    {
        lock_guard<mutex> guard(this->positions_mut);
        fill(this->effectors_mask.begin(), this->effectors_mask.end(), false);
    }
    {
        lock_guard<mutex> guard(this->joints_mut);
        this->mem->GetValues(this->joints);
    }
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::apply() {
    this->calculateJoints();
    AL::ALValue data;
    {
        lock_guard<mutex> guard(this->joints_mut);
        data = AL::ALValue(this->joints);
    }
    this->hw->callVoid("setJoints", data);
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setJoints(const AL::ALValue &values) {
    if (values.getSize() < NUM_OF_JOINTS) {
        return;
    }
    {
        lock_guard<mutex> guard(this->positions_mut);
        fill(this->effectors_mask.begin(), this->effectors_mask.end(), false);

    }
    {
        lock_guard<mutex> guard(this->joints_mut);
        for (int i = 0; i < NUM_OF_JOINTS; ++i) {
            this->joints[i] = values[i];
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getJoints() {
    AL::ALValue ret_data;
    this->calculateJoints();
    {
        lock_guard<mutex> guard(this->joints_mut);
        int size = this->joints.size();
        ret_data.arraySetSize(size);
        for (int i = 0; i < size; ++i) {
            ret_data[i] = this->joints[i];
        }
    }
    return ret_data;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::lookAt(const AL::ALValue &pos,
                              const bool top_camera) {
    lock_guard<mutex> guard(this->positions_mut);
    this->obj.x = pos[0];
    this->obj.y = pos[1];
    this->obj.z = pos[2];
    this->effectors_mask[HEAD] = true;
    this->top_camera = top_camera;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setLeftArmPosition(const AL::ALValue &pos) {
    //Pose3D pos3d = this->preparePose3D(pos);
    lock_guard<mutex> guard(this->positions_mut);
    this->effectors[LEFT_ARM] = this->preparePose3D(pos);
    this->effectors_mask[LEFT_ARM] = true;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setRightArmPosition(const AL::ALValue &pos) {
    //Pose3D pos3d = this->preparePose3D(pos);
    lock_guard<mutex> guard(this->positions_mut);
    this->effectors[RIGHT_ARM] = this->preparePose3D(pos);
    this->effectors_mask[RIGHT_ARM] = true;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setLeftLegPosition(const AL::ALValue &pos) {
    Pose3D pos3d = this->preparePose3D(pos);
    lock_guard<mutex> guard(this->positions_mut);
    this->effectors[LEFT_LEG] = this->preparePose3D(pos);
    this->effectors_mask[LEFT_LEG] = true;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setLeftLegPositionWithZ(const AL::ALValue &pos, float joint0) {
    Pose3D pos3d = this->preparePose3D(pos);
    lock_guard<mutex> guard(this->positions_mut);
    this->effectors[LEFT_LEG] = this->preparePose3D(pos);
    this->effectors_mask[LEFT_LEG] = true;
    this->leftjoint0 = joint0;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setRightLegPosition(const AL::ALValue &pos) {
    Pose3D pos3d = this->preparePose3D(pos);
    lock_guard<mutex> guard(this->positions_mut);
    this->effectors[RIGHT_LEG] = pos3d;
    this->effectors_mask[RIGHT_LEG] = true;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setRightLegPositionWithZ(const AL::ALValue &pos, float joint0) {
    Pose3D pos3d = this->preparePose3D(pos);
    lock_guard<mutex> guard(this->positions_mut);
    this->effectors[RIGHT_LEG] = this->preparePose3D(pos);
    this->effectors_mask[RIGHT_LEG] = true;
    this->leftjoint0 = joint0;
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getHeadPosition(bool top_camera) {
    this->calculateJoints();
    Pose3D cameramatrix;
    cameramatrix.translate(0.0, 0.0, this->rd.zLegJoint1ToHeadPan);
    cameramatrix.rotateZ(this->joints[RD::HEAD_YAW]);
    cameramatrix.rotateY(-this->joints[RD::HEAD_PITCH]);
    AL::ALValue head_pos;
    head_pos.arraySetSize(6);
    if (top_camera) {
        cameramatrix.translate(this->rd.xHeadTiltToUpperCamera, 0.f, this->rd.zHeadTiltToUpperCamera);
        cameramatrix.rotateY(this->rd.headTiltToUpperCameraTilt + this->cc.upperCameraTiltCorrection);
        cameramatrix.rotateX(this->cc.upperCameraRollCorrection);
        cameramatrix.rotateZ(this->cc.upperCameraPanCorrection);

    }
    else {
        cameramatrix.translate(this->rd.xHeadTiltToCamera, 0.f, this->rd.zHeadTiltToCamera);
        cameramatrix.rotateY(this->rd.headTiltToCameraTilt + this->cc.lowerCameraTiltCorrection);
        cameramatrix.rotateX(this->cc.lowerCameraRollCorrection);
        cameramatrix.rotateZ(this->cc.lowerCameraPanCorrection);
    }
    head_pos[0] = cameramatrix.translation.x;
    head_pos[1] = cameramatrix.translation.y;
    head_pos[2] = cameramatrix.translation.z;
    head_pos[3] = cameramatrix.rotation.getXAngle();
    head_pos[4] = cameramatrix.rotation.getYAngle();
    head_pos[5] = cameramatrix.rotation.getZAngle();
    return head_pos;

}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getLeftArmPosition() {
    this->calculateJoints();
    Pose3D limbs[MassCalibration::numOfLimbs];
    ForwardKinematic::calculateArmChain(true, joints, rd, mc, limbs);
    AL::ALValue arm_pos;
    arm_pos.arraySetSize(4);
    MassCalibration::Limb shoulder = MassCalibration::shoulderLeft;
    for (int i = 0; i < 4; ++i) {
        AL::ALValue limb;
        limb.arraySetSize(6);
        limb[0] = limbs[shoulder + i].translation.x;
        limb[1] = limbs[shoulder + i].translation.y;
        limb[2] = limbs[shoulder + i].translation.z;
        limb[3] = limbs[shoulder + i].rotation.getXAngle();
        limb[4] = limbs[shoulder + i].rotation.getYAngle();
        limb[5] = limbs[shoulder + i].rotation.getZAngle();
        arm_pos[i] = limb;
    }
    return arm_pos;
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getRightArmPosition() {
    this->calculateJoints();
    Pose3D limbs[MassCalibration::numOfLimbs];
    ForwardKinematic::calculateArmChain(false, joints, rd, mc, limbs);
    AL::ALValue arm_pos;
    arm_pos.arraySetSize(4);
    MassCalibration::Limb shoulder = MassCalibration::shoulderRight;
    for (int i = 0; i < 4; ++i) {
        AL::ALValue limb;
        limb.arraySetSize(6);
        limb[0] = limbs[shoulder + i].translation.x;
        limb[1] = limbs[shoulder + i].translation.y;
        limb[2] = limbs[shoulder + i].translation.z;
        limb[3] = limbs[shoulder + i].rotation.getXAngle();
        limb[4] = limbs[shoulder + i].rotation.getYAngle();
        limb[5] = limbs[shoulder + i].rotation.getZAngle();
        arm_pos[i] = limb;
    }
    return arm_pos;
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getLeftLegPosition() {
    this->calculateJoints();
    Pose3D limbs[MassCalibration::numOfLimbs];
    ForwardKinematic::calculateLegChain(true, joints, rd, mc, limbs);
    AL::ALValue leg_pos;
    leg_pos.arraySetSize(6);
    MassCalibration::Limb pelvis = MassCalibration::pelvisLeft;
    for (int i = 0; i < 6; ++i) {
        AL::ALValue limb;
        limb.arraySetSize(6);
        limb[0] = limbs[pelvis + i].translation.x;
        limb[1] = limbs[pelvis + i].translation.y;
        limb[2] = limbs[pelvis + i].translation.z;
        limb[3] = limbs[pelvis + i].rotation.getXAngle();
        limb[4] = limbs[pelvis + i].rotation.getYAngle();
        limb[5] = limbs[pelvis + i].rotation.getZAngle();
        leg_pos[i] = limb;
    }
    return leg_pos;
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getRightLegPosition() {
    this->calculateJoints();
    Pose3D limbs[MassCalibration::numOfLimbs];
    ForwardKinematic::calculateLegChain(false, joints, rd, mc, limbs);
    AL::ALValue leg_pos;
    leg_pos.arraySetSize(6);
    MassCalibration::Limb pelvis = MassCalibration::pelvisRight;
    for (int i = 0; i < 6; ++i) {
        AL::ALValue limb;
        limb.arraySetSize(6);
        limb[0] = limbs[pelvis + i].translation.x;
        limb[1] = limbs[pelvis + i].translation.y;
        limb[2] = limbs[pelvis + i].translation.z;
        limb[3] = limbs[pelvis + i].rotation.getXAngle();
        limb[4] = limbs[pelvis + i].rotation.getYAngle();
        limb[5] = limbs[pelvis + i].rotation.getZAngle();
        leg_pos[i] = limb;
    }
    return leg_pos;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::initFastAccess() {
    this->joints.clear();
    this->joints.resize(NUM_OF_JOINTS);

    this->sensor_keys.clear();
    this->sensor_keys.resize(NUM_OF_JOINTS);
    // Joints Sensor list
    this->sensor_keys[HEAD_PITCH] = string(
            "Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
    this->sensor_keys[HEAD_YAW] = string(
            "Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
    this->sensor_keys[L_ANKLE_PITCH] = string(
            "Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
    this->sensor_keys[L_ANKLE_ROLL] = string(
            "Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
    this->sensor_keys[L_ELBOW_ROLL] = string(
            "Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
    this->sensor_keys[L_ELBOW_YAW] = string(
            "Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
    this->sensor_keys[L_HAND] = string(
            "Device/SubDeviceList/LHand/Position/Sensor/Value");
    this->sensor_keys[L_HIP_PITCH] = string(
            "Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
    this->sensor_keys[L_HIP_ROLL] = string(
            "Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
    this->sensor_keys[L_HIP_YAW_PITCH] = string(
            "Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
    this->sensor_keys[L_KNEE_PITCH] = string(
            "Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
    this->sensor_keys[L_SHOULDER_PITCH] = string(
            "Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
    this->sensor_keys[L_SHOULDER_ROLL] = string(
            "Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
    this->sensor_keys[L_WRIST_YAW] = string(
            "Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
    this->sensor_keys[R_ANKLE_PITCH] = string(
            "Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
    this->sensor_keys[R_ANKLE_ROLL] = string(
            "Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
    this->sensor_keys[R_ELBOW_ROLL] = string(
            "Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
    this->sensor_keys[R_ELBOW_YAW] = string(
            "Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
    this->sensor_keys[R_HAND] = string(
            "Device/SubDeviceList/RHand/Position/Sensor/Value");
    this->sensor_keys[R_HIP_PITCH] = string(
            "Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
    this->sensor_keys[R_HIP_ROLL] = string(
            "Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
    this->sensor_keys[R_HIP_YAW_PITCH] = string(
            "Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    this->sensor_keys[R_KNEE_PITCH] = string(
            "Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    this->sensor_keys[R_SHOULDER_PITCH] = string(
            "Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
    this->sensor_keys[R_SHOULDER_ROLL] = string(
            "Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
    this->sensor_keys[R_WRIST_YAW] = string(
            "Device/SubDeviceList/RWristYaw/Position/Sensor/Value");

    this->mem->ConnectToVariables(this->getParentBroker(), this->sensor_keys,
                                  false);
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::initHW() {
    try {
        this->hw = make_shared<AL::ALProxy>(this->getParentBroker(),
                                            string("RD/HardwareAccessModule"));
    } catch (const AL::ALError &e) {
        throw ALERROR(this->getName(), "initHW()",
                      "Error when connecting to hardware access module");
    }
}

////////////////////////////////////////////////////////////////////////////////

Pose3D KinematicsModule::preparePose3D(
        const AL::ALValue &pos) {
    return Pose3D(
            RotationMatrix().rotateZ(pos[5]).rotateY(pos[4]).rotateX(pos[3]),
            Vector3<>(pos[0], pos[1], pos[2]));
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::calculateJoints() {
    JointContainer angles(NUM_OF_JOINTS);
    vector<Pose3D> positions(NUM_OF_EFFECTORS);
    vector<bool> positions_mask(NUM_OF_EFFECTORS);
    vector<float>::iterator joints_it;
    bool top_camera;

    {
        lock_guard<mutex> guard(this->positions_mut);
        copy(this->effectors.begin(), this->effectors.end(), positions.begin());
        copy(this->effectors_mask.begin(), this->effectors_mask.end(),
             positions_mask.begin());
        top_camera = this->top_camera;
        fill(this->effectors_mask.begin(), this->effectors_mask.end(),
             false);
    }
    {
        lock_guard<mutex> guard(this->joints_mut);
        copy(this->joints.begin(), this->joints.end(), angles.begin());

    }

    if (positions_mask[HEAD]) {
        InverseKinematic::calcHeadJoints(this->obj, pi / 2, this->rd, this->top_camera, angles, this->cc);

    }

    if (positions_mask[LEFT_ARM] || positions_mask[RIGHT_ARM]) {
        InverseKinematic::calcArmJoints(positions[LEFT_ARM], positions[RIGHT_ARM], angles, this->rd, this->jc);
    }
    if (positions_mask[LEFT_LEG]) {
        Pose3D limbs[MassCalibration::numOfLimbs];
        ForwardKinematic::calculateLegChain(false, joints, this->rd, this->mc, limbs);
        InverseKinematic::calcLegJoints(positions[LEFT_LEG], angles, this->leftjoint0, true, this->rd);
        MassCalibration::Limb pelvis = MassCalibration::pelvisRight;
        InverseKinematic::calcLegJoints(limbs[pelvis + 5], angles, this->leftjoint0, false, this->rd);
    }
    if (positions_mask[RIGHT_LEG]) {
        Pose3D limbs[MassCalibration::numOfLimbs];
        ForwardKinematic::calculateLegChain(true, joints, this->rd, this->mc, limbs);
        InverseKinematic::calcLegJoints(positions[RIGHT_LEG], angles, this->leftjoint0, false, this->rd);
        MassCalibration::Limb pelvis = MassCalibration::pelvisLeft;
        InverseKinematic::calcLegJoints(limbs[pelvis + 5], angles, this->leftjoint0, true, this->rd);
    }



    /*if (positions_mask[LEFT_LEG] || positions_mask[RIGHT_LEG]) {
        InverseKinematic::calcLegJoints(positions[LEFT_LEG],
                                        positions[RIGHT_LEG], angles, this->rd);
    }*/

    {
        lock_guard<mutex> guard(this->joints_mut);
        this->joints.swap(angles);

    }
}
