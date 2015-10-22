//
// Created by arssivka on 10/5/15.
//

#include "RD/KinematicsModule/KinematicsModule.h"

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <almemoryfastaccess/almemoryfastaccess.h>
#include <alproxies/altexttospeechproxy.h>

using namespace RD;
using namespace boost;
using namespace std;

enum SensorGroups {
    HEAD_GROUP = HEAD_PITCH,
    L_ARM_GROUP = L_ANKLE_PITCH,
    L_LEG_GROUP = L_HIP_PITCH,
    R_ARM_GROUP = R_ANKLE_PITCH,
    R_LEG_GROUP = R_HIP_PITCH
};

////////////////////////////////////////////////////////////////////////////////

KinematicsModule::KinematicsModule(shared_ptr<AL::ALBroker> pBroker,
                                   const string &pName)
        : AL::ALModule(pBroker, pName), mem(new AL::ALMemoryFastAccess()),
          positions(KDeviceLists::CHAINS_SIZE),
          positions_mask(KDeviceLists::CHAINS_SIZE, false) {
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

    this->functionName("getCenterOfMass", this->getName(),
                       "get center of mass");
    this->setReturn("values", "return center of mass");
    BIND_METHOD(KinematicsModule::getCenterOfMass);

    this->functionName("setHeadPosition", this->getName(),
                       "set head position and orientation in space");
    this->addParam("pos", "position and orientation");
    this->addParam("top_camera", "true - position for top camera, false - bottom camera");
    BIND_METHOD(KinematicsModule::setHeadPosition);

    this->functionName("setLeftHandPosition", this->getName(),
                       "set left hand position and orientation in space");
    this->addParam("pos", "position and orientation");
    BIND_METHOD(KinematicsModule::setLeftHandPosition);

    this->functionName("setRightHandPosition", this->getName(),
                       "set right hand position and orientation in space");
    this->addParam("pos", "position and orientation");
    BIND_METHOD(KinematicsModule::setRightHandPosition);

    this->functionName("setLeftLegPosition", this->getName(),
                       "set left leg position and orientation in space");
    this->addParam("pos", "position and orientation");
    BIND_METHOD(KinematicsModule::setLeftLegPosition);

    this->functionName("setRightLegPosition", this->getName(),
                       "set right leg position and orientation in space");
    this->addParam("pos", "position and orientation");
    BIND_METHOD(KinematicsModule::setRightLegPosition);

    this->functionName("getHeadPosition", this->getName(),
                       "get center of mass");
    this->addParam("top_camera", "true - position for top camera, false - bottom camera");
    this->setReturn("values", "get head position and orientation in space");
    BIND_METHOD(KinematicsModule::getHeadPosition);

    this->functionName("getLeftHandPosition", this->getName(),
                       "get center of mass");
    this->setReturn("values", "get left hand position and orientation in space");
    BIND_METHOD(KinematicsModule::getLeftHandPosition);

    this->functionName("getLeftLegPosition", this->getName(),
                       "get center of mass");
    this->setReturn("values", "get left leg position and orientation in space");
    BIND_METHOD(KinematicsModule::getLeftLegPosition);

    this->functionName("getRightHandPosition", this->getName(),
                       "get center of mass");
    this->setReturn("values", "get right hand position and orientation in space");
    BIND_METHOD(KinematicsModule::getRightHandPosition);

    this->functionName("getRightLegPosition", this->getName(),
                       "get center of mass");
    this->setReturn("values", "get right leg position and orientation in space");
    BIND_METHOD(KinematicsModule::getRightLegPosition);
}

////////////////////////////////////////////////////////////////////////////////

KinematicsModule::~KinematicsModule() { }

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::init() {
    this->initFastAccess();
    this->initHW();
    this->initKinematics();
    this->update();
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::update() {
    vector<float> kjoints;
    {
        lock_guard<mutex> guard(this->positions_mut);
        fill(this->positions_mask.begin(), this->positions_mask.end(), false);
    }
    {
        lock_guard<mutex> guard(this->joints_mut);
        this->mem->GetValues(this->joint_values);
        kjoints = this->joint_values;
    }
    kjoints.erase(kjoints.begin() + R_HAND);
    kjoints.erase(kjoints.begin() + L_HAND);
    {
        lock_guard<mutex> guard(this->kinematics_mut);
        this->kinematics->setJoints(kjoints);

    }
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::apply() {
    this->calculateJoints();
    AL::ALValue data;
    {
        lock_guard<mutex> guard(this->joints_mut);
        data = AL::ALValue(this->joint_values);
    }
    this->hw->callVoid("setJointValues", data);
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getJoints() {
    AL::ALValue ret_data;
    this->calculateJoints();
    {
        lock_guard<mutex> guard(this->joints_mut);
        int size = this->joint_values.size();
        ret_data.arraySetSize(size);
        for (int i = 0; i < size; ++i) {
            ret_data[i] = this->joint_values[i];
        }
    }
    return ret_data;
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getCenterOfMass() {
    KVecDouble3 com;
    {
        lock_guard<mutex> guard(this->kinematics_mut);
        com = this->kinematics->calculateCenterOfMass();
    }
    return AL::ALValue::array(com.get(0, 0), com.get(1, 0), com.get(2, 0));
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setHeadPosition(AL::ALValue pos, bool top_camera) {
    lock_guard<mutex> guard(this->positions_mut);
    this->positions[KDeviceLists::CHAIN_HEAD] = this->prepareFKvars(pos);
    this->positions_mask[KDeviceLists::CHAIN_HEAD] = true;
    this->top_camera = top_camera;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setLeftHandPosition(AL::ALValue pos) {
    lock_guard<mutex> guard(this->positions_mut);
    this->positions[KDeviceLists::CHAIN_L_ARM] = this->prepareFKvars(pos);
    this->positions_mask[KDeviceLists::CHAIN_L_ARM] = true;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setRightHandPosition(AL::ALValue pos) {
    lock_guard<mutex> guard(this->positions_mut);
    this->positions[KDeviceLists::CHAIN_R_ARM] = this->prepareFKvars(pos);
    this->positions_mask[KDeviceLists::CHAIN_R_ARM] = true;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setLeftLegPosition(AL::ALValue pos) {
    lock_guard<mutex> guard(this->positions_mut);
    this->positions[KDeviceLists::CHAIN_L_LEG] = this->prepareFKvars(pos);
    this->positions_mask[KDeviceLists::CHAIN_L_LEG] = true;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::setRightLegPosition(AL::ALValue pos) {
    lock_guard<mutex> guard(this->positions_mut);
    this->positions[KDeviceLists::CHAIN_R_LEG] = this->prepareFKvars(pos);
    this->positions_mask[KDeviceLists::CHAIN_R_LEG] = true;
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getHeadPosition(bool top_camera) {
    this->calculateJoints();
    return this->getPosition(
            (top_camera)
            ? NAOKinematics::EFF_CAMERA_TOP
            : NAOKinematics::EFF_CAMERA_BOT);
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getLeftHandPosition() {
    this->calculateJoints();
    return this->getPosition(
            (NAOKinematics::Effectors) KDeviceLists::CHAIN_L_ARM);
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getRightHandPosition() {
    this->calculateJoints();
    return this->getPosition(
            (NAOKinematics::Effectors) KDeviceLists::CHAIN_R_ARM);
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getLeftLegPosition() {
    this->calculateJoints();
    return this->getPosition(
            (NAOKinematics::Effectors) KDeviceLists::CHAIN_L_LEG);
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue KinematicsModule::getRightLegPosition() {
    this->calculateJoints();
    return this->getPosition(
            (NAOKinematics::Effectors) KDeviceLists::CHAIN_R_LEG);
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::initFastAccess() {
    this->joint_values.clear();
    this->joint_values.resize(SENSORS_COUNT);

    this->sensor_keys.clear();
    this->sensor_keys.resize(SENSORS_COUNT);
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

void KinematicsModule::initKinematics() {
    this->kinematics = make_shared<NAOKinematics>();
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

inline AL::ALValue KinematicsModule::getPosition(NAOKinematics::Effectors ef) {
    NAOKinematics::kmatTable pos;
    this->calculateJoints();
    {
        lock_guard<mutex> guard(this->joints_mut);
        pos = this->kinematics->getForwardEffector(ef);
    }
    KMath::KMat::GenMatrix<float, 3, 1> angles;
    angles = pos.getEulerAngles();
    return AL::ALValue::array(
            pos.get(0, 3),
            pos.get(1, 3),
            pos.get(2, 3),
            angles.get(0, 0),
            angles.get(1, 0),
            angles.get(2, 0));
}

////////////////////////////////////////////////////////////////////////////////

inline NAOKinematics::FKvars KinematicsModule::prepareFKvars(
        const AL::ALValue &pos) {
    NAOKinematics::FKvars fk;
    fk.p.get(0, 0) = pos[0];
    fk.p.get(1, 0) = pos[1];
    fk.p.get(2, 0) = pos[2];
    fk.a.get(0, 0) = pos[3];
    fk.a.get(1, 0) = pos[4];
    fk.a.get(2, 0) = pos[5];
    return fk;
}

////////////////////////////////////////////////////////////////////////////////

void KinematicsModule::calculateJoints() {
    NAOKinematics::AngleContainer angles;
    vector<NAOKinematics::FKvars> positions(KDeviceLists::CHAINS_SIZE);
    vector<bool> positions_mask(KDeviceLists::CHAINS_SIZE);
    vector<float>::iterator joint_values_it;
    bool top_camera;

    {
        lock_guard<mutex> guard(this->positions_mut);
        copy(this->positions.begin(), this->positions.end(), positions.begin());
        copy(this->positions_mask.begin(), this->positions_mask.end(),
             positions_mask.begin());
        top_camera = this->top_camera;
        fill(this->positions_mask.begin(), this->positions_mask.end(),
                  false);
    }

    if (positions_mask[KDeviceLists::CHAIN_HEAD]) {
        {
            lock_guard<mutex> guard(this->kinematics_mut);
            angles = this->kinematics->inverseHead(
                    positions[KDeviceLists::CHAIN_HEAD], false, top_camera);
        }
        if (!angles.empty()) {
            joint_values_it = this->joint_values.begin();
            advance(joint_values_it, HEAD_GROUP);
            lock_guard<mutex> guard(this->joints_mut);
            copy(angles[0].begin(), angles[0].end(), joint_values_it);
        }
    }

    if (positions_mask[KDeviceLists::CHAIN_L_ARM]) {
        {
            lock_guard<mutex> guard(this->kinematics_mut);
            angles = this->kinematics->inverseLeftHand(
                    positions[KDeviceLists::CHAIN_L_ARM]);
        }
        if (!angles.empty()) {
            joint_values_it = this->joint_values.begin() + L_ARM_GROUP;
            lock_guard<mutex> guard(this->joints_mut);
            copy(angles[0].begin(), angles[0].end(), joint_values_it);
        }
    }

    if (positions_mask[KDeviceLists::CHAIN_L_LEG]) {
        {
            lock_guard<mutex> guard(this->kinematics_mut);
            angles = this->kinematics->inverseLeftHand(
                    positions[KDeviceLists::CHAIN_L_LEG]);
        }
        if (!angles.empty()) {
            joint_values_it = this->joint_values.begin() + L_LEG_GROUP;
            lock_guard<mutex> guard(this->joints_mut);
            copy(angles[0].begin(), angles[0].end(), joint_values_it);
        }
    }

    if (positions_mask[KDeviceLists::CHAIN_R_ARM]) {
        {
            lock_guard<mutex> guard(this->kinematics_mut);
            angles = this->kinematics->inverseLeftHand(
                    positions[KDeviceLists::CHAIN_R_ARM]);
        }
        if (!angles.empty()) {
            joint_values_it = this->joint_values.begin() + R_ARM_GROUP;
            lock_guard<mutex> guard(this->joints_mut);
            copy(angles[0].begin(), angles[0].end(), joint_values_it);
        }
    }

    if (positions_mask[KDeviceLists::CHAIN_R_LEG]) {
        {
            lock_guard<mutex> guard(this->kinematics_mut);
            angles = this->kinematics->inverseLeftHand(
                    positions[KDeviceLists::CHAIN_R_LEG]);
        }
        if (!angles.empty()) {
            joint_values_it = this->joint_values.begin() + R_LEG_GROUP;
            lock_guard<mutex> guard(this->joints_mut);
            copy(angles[0].begin(), angles[0].end(), joint_values_it);
        }
    }
}