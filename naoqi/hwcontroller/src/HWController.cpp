//
// Created by arssivka on 10/5/15.
//

#include "rd/naoqi/hwcontroller/HWController.h"

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alproxies/dcmproxy.h>
#include <almemoryfastaccess/almemoryfastaccess.h>
#include <alproxies/altexttospeechproxy.h>


using namespace rd;
using namespace std;
using namespace boost;
using namespace AL;


HWController::HWController(shared_ptr<ALBroker> broker, const string &name)
        : ALModule(broker, name), mem(make_shared<ALMemoryFastAccess>()), sensor_keys(NUM_OF_JOINTS),
          sensor_data(NUM_OF_JOINTS), work_actuator_values(new vector<float>(NUM_OF_JOINTS)) {
    this->setModuleDescription("Fast access to hardware.");

    this->functionName("setJointData", this->getName(),
                       "change effectors of all joint");
    this->addParam("values", "new effectors of all (NUM_OF_JOINTS) joints");
    BIND_METHOD(HWController::setJointData);

    this->functionName("getJointData", this->getName(), "get data from all sensors");
    this->setReturn("values", "return values from all sensors");
    BIND_METHOD(HWController::getJointData);

    long isDCMRunning;
    try {
        this->dcm = this->getParentBroker()->getDcmProxy();
    }
    catch (ALError &e) {
        throw ALERROR(getName(), "startLoop()", "Impossible to create DCM Proxy : " + string(e.what()));
    }
    try {
        isDCMRunning = this->getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", string("DCM"));
    }
    catch (ALError &e) {
        throw ALERROR(this->getName(), "startLoop()", "Error when connecting to DCM : " + string(e.what()));
    }
    if (!isDCMRunning) throw ALERROR(this->getName(), "startLoop()", "Error no DCM running ");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HWController::~HWController() {
    this->stopLoop();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::startLoop() {
    this->connectToDCMloop();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::stopLoop() {
    this->dcm_post_process_connection.disconnect();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::setJointData(const ALValue &values) {
    if (values.getSize() < NUM_OF_JOINTS) return;
    shared_ptr<vector<float> > local_actuator_values = make_shared<vector<float> >(NUM_OF_JOINTS);
    for (int i = 0; i < NUM_OF_JOINTS; ++i) local_actuator_values->at(i) = (float) values[i];
    lock_guard<mutex> guard(this->actuator_mutex);
    this->work_actuator_values.swap(local_actuator_values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ALValue HWController::getJointData() {
    lock_guard<mutex>(this->sensor_mutex);
    this->mem->GetValues(this->sensor_data);
    for (int i = 0; i < NUM_OF_JOINTS; ++i) this->sensor_values[i] = this->sensor_data.at(i);
    return this->sensor_values;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::init() {
    this->initFastAccess();
    this->createPositionActuatorAlias();
    this->preparePositionActuatorCommand();
    this->startLoop();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::initFastAccess() {
    this->sensor_values.arraySetSize(NUM_OF_JOINTS);

    this->sensor_keys.resize(NUM_OF_JOINTS);
    this->sensor_keys[HEAD_PITCH] = string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
    this->sensor_keys[HEAD_YAW] = string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
    this->sensor_keys[L_ANKLE_PITCH] = string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
    this->sensor_keys[L_ANKLE_ROLL] = string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
    this->sensor_keys[L_ELBOW_ROLL] = string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
    this->sensor_keys[L_ELBOW_YAW] = string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
    this->sensor_keys[L_HAND] = string("Device/SubDeviceList/LHand/Position/Sensor/Value");
    this->sensor_keys[L_HIP_PITCH] = string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
    this->sensor_keys[L_HIP_ROLL] = string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
    this->sensor_keys[L_HIP_YAW_PITCH] = string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
    this->sensor_keys[L_KNEE_PITCH] = string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
    this->sensor_keys[L_SHOULDER_PITCH] = string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
    this->sensor_keys[L_SHOULDER_ROLL] = string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
    this->sensor_keys[L_WRIST_YAW] = string("Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
    this->sensor_keys[R_ANKLE_PITCH] = string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
    this->sensor_keys[R_ANKLE_ROLL] = string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
    this->sensor_keys[R_ELBOW_ROLL] = string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
    this->sensor_keys[R_ELBOW_YAW] = string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
    this->sensor_keys[R_HAND] = string("Device/SubDeviceList/RHand/Position/Sensor/Value");
    this->sensor_keys[R_HIP_PITCH] = string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
    this->sensor_keys[R_HIP_ROLL] = string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
    this->sensor_keys[R_HIP_YAW_PITCH] = string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    this->sensor_keys[R_KNEE_PITCH] = string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    this->sensor_keys[R_SHOULDER_PITCH] = string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
    this->sensor_keys[R_SHOULDER_ROLL] = string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
    this->sensor_keys[R_WRIST_YAW] = string("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");

    this->mem->ConnectToVariables(this->getParentBroker(), this->sensor_keys, false);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::createPositionActuatorAlias() {
    ALValue joint_aliases;
    joint_aliases.arraySetSize(2);
    joint_aliases[0] = string("jointActuator");
    joint_aliases[1].arraySetSize(NUM_OF_JOINTS);
    joint_aliases[1][HEAD_PITCH] = string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    joint_aliases[1][HEAD_YAW] = string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
    joint_aliases[1][L_ANKLE_PITCH] = string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    joint_aliases[1][L_ANKLE_ROLL] = string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    joint_aliases[1][L_ELBOW_ROLL] = string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
    joint_aliases[1][L_ELBOW_YAW] = string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
    joint_aliases[1][L_HAND] = string("Device/SubDeviceList/LHand/Position/Actuator/Value");
    joint_aliases[1][L_HIP_PITCH] = string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    joint_aliases[1][L_HIP_ROLL] = string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    joint_aliases[1][L_HIP_YAW_PITCH] = string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    joint_aliases[1][L_KNEE_PITCH] = string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
    joint_aliases[1][L_SHOULDER_PITCH] = string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
    joint_aliases[1][L_SHOULDER_ROLL] = string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
    joint_aliases[1][L_WRIST_YAW] = string("Device/SubDeviceList/LWristYaw/Position/Actuator/Value");
    joint_aliases[1][R_ANKLE_PITCH] = string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    joint_aliases[1][R_ANKLE_ROLL] = string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    joint_aliases[1][R_ELBOW_ROLL] = string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
    joint_aliases[1][R_ELBOW_YAW] = string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
    joint_aliases[1][R_HAND] = string("Device/SubDeviceList/RHand/Position/Actuator/Value");
    joint_aliases[1][R_HIP_PITCH] = string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    joint_aliases[1][R_HIP_ROLL] = string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    joint_aliases[1][R_HIP_YAW_PITCH] = string("Device/SubDeviceList/RHipYawPitch/Position/Actuator/Value");
    joint_aliases[1][R_KNEE_PITCH] = string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    joint_aliases[1][R_SHOULDER_PITCH] = string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    joint_aliases[1][R_SHOULDER_ROLL] = string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    joint_aliases[1][R_WRIST_YAW] = string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");

    // Create alias
    try {
        this->dcm->createAlias(joint_aliases);
    } catch (const ALError &e) {
        throw ALERROR(getName(), "createPositionActuatorAlias()", "Error when creating Alias : " + string(e.what()));
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::preparePositionActuatorCommand() {
    this->commands.arraySetSize(6);
    this->commands[0] = string("jointActuator");
    this->commands[1] = string("ClearAll");
    this->commands[2] = string("time-separate");
    this->commands[3] = 0;
    this->commands[4].arraySetSize(1);
    this->commands[5].arraySetSize(NUM_OF_JOINTS);
    for (int i = 0; i < NUM_OF_JOINTS; i++) this->commands[5][i].arraySetSize(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::connectToDCMloop() {
    this->mem->GetValues(this->sensor_data);
    for (int i = 0; i < NUM_OF_JOINTS; ++i) this->work_actuator_values->at(i) = this->sensor_data.at(i);

    try {
        dcm_post_process_connection = this->getParentBroker()
                ->getProxy("DCM")
                ->getModule()
                ->atPostProcess(bind(&HWController::synchronisedDCMcallback, this));
    }
    catch (const ALError &e) {
        throw ALERROR(this->getName(),
                      "connectToDCMloop()", "Error when connecting to DCM postProccess: " + string(e.what()));
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HWController::synchronisedDCMcallback() {
    int dcm_time;
    try {
        dcm_time = this->dcm->getTime(0);
    }
    catch (const ALError &e) {
        throw ALERROR(this->getName(), "synchronisedDCMcallback()", "Error on DCM getTime : " + string(e.what()));
    }

    this->commands[4][0] = dcm_time;
    {
        lock_guard<mutex> guard(this->actuator_mutex);
        for (int i = 0; i < NUM_OF_JOINTS; i++) this->commands[5][i][0] = this->work_actuator_values->at(i);
        try {
            this->dcm->setAlias(commands);
        }
        catch (const ALError &e) {
            throw ALERROR(this->getName(),
                          "synchronisedDCMcallback()", "Error when sending command to DCM : " + string(e.what()));
        }
    }
}