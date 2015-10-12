//
// Created by arssivka on 10/1/15.
//

#include "KinematicsModule.h"

using namespace AL;
using namespace RD;
using namespace std;

const string DCM_UPDATE_STRING[4] = {
    string("ClearAll"),
    string("Merge"),
    string("ClearAfter"),
    string("ClearBefore")
};

const string DCM_ALL_ACTUTATORS_ALIAS("RD/jointActuators");
const string DCM_HEAD_ACTUTATORS_ALIAS("RD/HeadActutators");
const string DCM_L_ARM_ACTUTATORS_ALIAS("RD/LArmActutators");
const string DCM_L_LEG_ACTUTATORS_ALIAS("RD/LLegActutators");
const string DCM_R_ARM_ACTUTATORS_ALIAS("RD/RArmActutators");
const string DCM_R_LEG_ACTUTATORS_ALIAS("RD/RLegActutators");

const string DCM_SENSORS[KDeviceLists::NUMOFJOINTS] = {

};

////////////////////////////////////////////////////////////////////////////////

DCMKinematicsModule::KinematicsModule(boost::shared_ptr <ALBroker> broker,
                                         const string &name,
                                         DCMUpdateType utype)
        : ALModule(broker, name), utype(utype),
          mem(new AL::ALMemoryFastAccess()) {
    this->setModuleDescription("Kinematics module with fast joints access");
    this->commands.clear();
    this->commands.arraySetSize(2);
    this->commands[0] = DCM_ALL_ACTUTATORS_ALIAS;
    this->commands[1].arraySetSize(KDeviceLists::NUMOFJOINTS);
    this->commands[1][0] = string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
    this->commands[1][1] = string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    this->commands[1][2] = string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
    this->commands[1][3] = string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
    this->commands[1][4] = string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
    this->commands[1][5] = string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
    this->commands[1][6] = string("Device/SubDeviceList/LWristYaw/Position/Actuator/Value");
    this->commands[1][7] = string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    this->commands[1][8] = string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    this->commands[1][9] = string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    this->commands[1][10] = string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
    this->commands[1][11] = string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    this->commands[1][12] = string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    this->commands[1][13] = string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    this->commands[1][14] = string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    this->commands[1][15] = string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
    this->commands[1][16] = string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
    this->commands[1][17] = string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");
    this->commands[1][18] = string("Device/SubDeviceList/RHipYawPitch/Position/Actuator/Value");
    this->commands[1][19] = string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    this->commands[1][20] = string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    this->commands[1][21] = string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    this->commands[1][22] = string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    this->commands[1][23] = string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    this->dcm->setAlias(commands);

//    commands[0] = DCM_HEAD_ACTUTATORS_ALIAS;
//    commands[1].arraySetSize(KDeviceLists::HEAD_SIZE);
//    commands[1][0] = string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
//    commands[1][1] = string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
//    this->dcm->setAlias(commands);
//
//    commands[0] = DCM_L_ARM_ACTUTATORS_ALIAS;
//    commands[1].arraySetSize(KDeviceLists::ARM_SIZE);
//    commands[1][0] = string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
//    commands[1][1] = string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
//    commands[1][2] = string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
//    commands[1][3] = string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
//    commands[1][4] = string("Device/SubDeviceList/LWristYaw/Position/Actuator/Value");
//    this->dcm->setAlias(commands);
//
//    commands[0] = DCM_L_LEG_ACTUTATORS_ALIAS;
//    commands[1].arraySetSize(KDeviceLists::LEG_SIZE);
//    commands[1][0] = string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
//    commands[1][1] = string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
//    commands[1][2] = string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
//    commands[1][3] = string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
//    commands[1][4] = string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
//    commands[1][5] = string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
//    this->dcm->setAlias(commands);
//
//    commands[0] = DCM_R_ARM_ACTUTATORS_ALIAS;
//    commands[1].arraySetSize(KDeviceLists::ARM_SIZE);
//    commands[1][0] = string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
//    commands[1][1] = string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
//    commands[1][2] = string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
//    commands[1][3] = string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
//    commands[1][4] = string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");
//    this->dcm->setAlias(commands);
//
//    commands[0] = DCM_R_LEG_ACTUTATORS_ALIAS;
//    commands[1].arraySetSize(KDeviceLists::LEG_SIZE);
//    commands[1][0] = string("Device/SubDeviceList/RHipYawPitch/Position/Actuator/Value");
//    commands[1][1] = string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
//    commands[1][2] = string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
//    commands[1][3] = string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
//    commands[1][4] = string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
//    commands[1][5] = string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
//    this->dcm->setAlias(commands);
}

////////////////////////////////////////////////////////////////////////////////

DCMKinematicsModule::~KinematicsModule() {
    this->stopLoop();
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::init() {
    this->sensor_keys.clear();
    // Head position sensors
    this->sensor_keys.push_back(string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value"));
    // Left arm position sensors
    this->sensor_keys.push_back(string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LWristYaw/Position/Sensor/Value"));
    // Left leg position sensors
    this->sensor_keys.push_back(string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value"));
    // Right arm position sensors
    this->sensor_keys.push_back(string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RWristYaw/Position/Sensor/Value"));
    // Right leg position sensors
    this->sensor_keys.push_back(string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value"));
    this->sensor_keys.push_back(string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value"));
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::setDCMUpdateType(DCMUpdateType utype) {
    this->utype;
}

////////////////////////////////////////////////////////////////////////////////

DCMUpdateType DCMKinematicsModule::getDCMUpdateType() const {
    return this->utype;
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::setHeadPosition(const AL::ALValue &pos, int time) {
    this->setChain(this->chains[KDeviceLists::CHAIN_HEAD], pos, time);
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::setLHandPosition(const AL::ALValue &pos, int time) {
    this->setChain(this->chains[KDeviceLists::CHAIN_L_ARM], pos, time);
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::setRHandPosition(const AL::ALValue &pos, int time) {
    this->setChain(this->chains[KDeviceLists::CHAIN_R_ARM], pos, time);
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::setLLegPosition(const AL::ALValue &pos, int time) {
    this->setChain(this->chains[KDeviceLists::CHAIN_L_LEG], pos, time);
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::setRLegPosition(const AL::ALValue &pos, int time) {
    this->setChain(this->chains[KDeviceLists::CHAIN_R_LEG], pos, time);
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::worker_func() {
    cout << "Now kinematics motion module is running." << endl;
    NAOKinematics k;

    while(this->run.load()) {

    }
}

////////////////////////////////////////////////////////////////////////////////

void DCMKinematicsModule::setChain(ChainContainer& chain, const AL::ALValue &pos, int time) {
    boost::lock_guard<boost::mutex> guard(chain.mut);
    chain.changed = true;
    chain.pos = pos;
    chain.time = time;
}