//
// Created by arssivka on 10/5/15.
//

#include "RD/HardwareAccessModule/HardwareAccessModule.h"

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alproxies/dcmproxy.h>
#include <almemoryfastaccess/almemoryfastaccess.h>
#include <alproxies/altexttospeechproxy.h>
#include "RD/HardwareAccessModule/HardwareDefines.h"


using namespace RD;
using namespace std;
using namespace boost;


HardwareAccessModule::HardwareAccessModule(
        shared_ptr<AL::ALBroker> broker,
        const string &name)
        : AL::ALModule(broker, name), mem(
        shared_ptr<AL::ALMemoryFastAccess>(
                new AL::ALMemoryFastAccess())), sensor_keys(NUM_OF_JOINTS),
          local_sensor_values(SENSORS_COUNT),
          work_actuator_values(new vector<float>(NUM_OF_JOINTS)) {

    this->setModuleDescription("Fast access to hardware.");

    this->functionName("setJoints", this->getName(),
                       "change positions of all joint");
    this->addParam("values", "new positions of all (NUM_OF_JOINTS) joints");
    BIND_METHOD(HardwareAccessModule::setJoints);

    this->functionName("setStiffness", this->getName(),
                       "change stiffness of all joint");
    this->addParam("value", "new stiffness value from 0.0 to 1.0");
    BIND_METHOD(HardwareAccessModule::setStiffness);

    this->functionName("getSensorsValues", this->getName(),
                       "get data from all sensors");
    this->setReturn("values", "return values from all sensors");
    BIND_METHOD(HardwareAccessModule::getSensorsValues);

    this->functionName("stopStream", this->getName(), "stop robot cameras");
    BIND_METHOD(HardwareAccessModule::stopStream);

    this->functionName("getImageBufferTop", this->getName(),
                       "get image buffer from top camera");
    this->setReturn("alvalue", "string with binary data");
    BIND_METHOD(HardwareAccessModule::getImageBufferTop);

    this->functionName("getImageBufferBot", this->getName(),
                       "get image buffer from bottom camera");
    this->setReturn("alvalue", "string with binary data");
    BIND_METHOD(HardwareAccessModule::getImageBufferBot);

    this->functionName("checkDevices", this->getName(),
                       "check if cameras are ok");
    this->setReturn("boolean", "true if cameras started normally");
    BIND_METHOD(HardwareAccessModule::checkDevices);

    long isDCMRunning;
    try {
        this->dcm = this->getParentBroker()->getDcmProxy();
    }
    catch (AL::ALError &e) {
        throw ALERROR(getName(), "startLoop()",
                      "Impossible to create DCM Proxy : " +
                              string(e.what()));
    }
    try {
        isDCMRunning = this->getParentBroker()->getProxy(
                "ALLauncher")->call<bool>("isModulePresent",
                                          string("DCM"));
    }
    catch (AL::ALError &e) {
        throw ALERROR(this->getName(), "startLoop()",
                      "Error when connecting to DCM : " +
                              string(e.what()));
    }
    if (!isDCMRunning) {
        throw ALERROR(this->getName(), "startLoop()", "Error no DCM running ");
    }

    this->init();
}

////////////////////////////////////////////////////////////////////////////////

HardwareAccessModule::~HardwareAccessModule() {
    this->stopLoop();
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::startLoop() {
    this->connectToDCMloop();
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::stopLoop() {
    this->setStiffness(0.0f);
    this->dcm_post_process_connection.disconnect();
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::setJoints(const AL::ALValue &values) {
    if (values.getSize() < NUM_OF_JOINTS) {
        return;
    }
    shared_ptr<vector<float> > local_actuator_values(
            new vector<float>(NUM_OF_JOINTS));
    for (int i = 0; i < NUM_OF_JOINTS; ++i) {
        local_actuator_values->at(i) = (float) values[i];
    }
    lock_guard<mutex> guard(this->actuator_mutex);
    this->work_actuator_values.swap(local_actuator_values);
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::setStiffness(const float &stiffnessValue) {
    AL::ALValue stiffnessCommands;
    int DCMtime;
    // increase stiffness with the "jointStiffness" Alias created at initialisation
    try {
        // Get time : return the time in 1 seconde
        DCMtime = this->dcm->getTime(100);
    } catch (const AL::ALError &e) {
        throw ALERROR(getName(), "setStiffness()",
                      "Error on DCM getTime : " + string(e.what()));
    }

    stiffnessCommands.arraySetSize(3);
    stiffnessCommands[0] = string("jointStiffness");
    stiffnessCommands[1] = string("Merge");
    stiffnessCommands[2].arraySetSize(1);
    stiffnessCommands[2][0].arraySetSize(2);
    stiffnessCommands[2][0][0] = stiffnessValue;
    stiffnessCommands[2][0][1] = DCMtime;
    try {
        this->dcm->set(stiffnessCommands);
    } catch (const AL::ALError &e) {
        throw ALERROR(this->getName(), "setStiffness()",
                      "Error when sending stiffness to DCM : " +
                              string(e.what()));
    }
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue HardwareAccessModule::getSensorsValues() {
    this->mem->GetValues(this->local_sensor_values);
    for (int i = 0; i < SENSORS_COUNT; ++i) {
        sensor_values[i] = this->local_sensor_values.at(i);
    }
    return sensor_values;
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::init() {

    this->top_camera = shared_ptr<V4LRobotCamera>(
            new V4LRobotCamera("/dev/video0", 320, 240, true));
    this->bottom_camera = shared_ptr<V4LRobotCamera>(
            new V4LRobotCamera("/dev/video1", 320, 240, true));
    this->top_camera->setFPS(25);
    this->bottom_camera->setFPS(25);
    this->top_camera->startCapturing();
    this->bottom_camera->startCapturing();

    this->initFastAccess();
    this->createPositionActuatorAlias();
    this->createHardnessActuatorAlias();
    this->preparePositionActuatorCommand();
    this->startLoop();
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::initFastAccess() {
    //  Here as an example inertial + joints + FSR are read
    this->sensor_values.clear();
    this->sensor_values.arraySetSize(SENSORS_COUNT);

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
    // Inertial sensors
    this->sensor_keys[ACC_X] = string(
            "Device/SubDeviceList/InertialSensor/AccX/Sensor/Value");
    this->sensor_keys[ACC_Y] = string(
            "Device/SubDeviceList/InertialSensor/AccY/Sensor/Value");
    this->sensor_keys[ACC_Z] = string(
            "Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value");
    this->sensor_keys[GYR_X] = string(
            "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value");
    this->sensor_keys[GYR_Y] = string(
            "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value");
    this->sensor_keys[ANGLE_X] = string(
            "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
    this->sensor_keys[ANGLE_Y] = string(
            "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
    // Some FSR sensors
    this->sensor_keys[L_COP_X] = string(
            "Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value");
    this->sensor_keys[L_COP_Y] = string(
            "Device/SubDeviceList/LFoot/FSR/CenterOfPressure/Y/Sensor/Value");
    this->sensor_keys[L_TOTAL_WEIGHT] = string(
            "Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value");
    this->sensor_keys[R_COP_X] = string(
            "Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value");
    this->sensor_keys[R_COP_Y] = string(
            "Device/SubDeviceList/RFoot/FSR/CenterOfPressure/Y/Sensor/Value");
    this->sensor_keys[R_TOTAL_WEIGHT] = string(
            "Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value");

    this->mem->ConnectToVariables(this->getParentBroker(), this->sensor_keys,
                                  false);
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::createPositionActuatorAlias() {
    AL::ALValue joint_aliases;
    joint_aliases.arraySetSize(2);
    joint_aliases[0] = string("jointActuator");
    joint_aliases[1].arraySetSize(NUM_OF_JOINTS);
    joint_aliases[1][HEAD_PITCH] = string(
            "Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    joint_aliases[1][HEAD_YAW] = string(
            "Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
    joint_aliases[1][L_ANKLE_PITCH] = string(
            "Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    joint_aliases[1][L_ANKLE_ROLL] = string(
            "Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    joint_aliases[1][L_ELBOW_ROLL] = string(
            "Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
    joint_aliases[1][L_ELBOW_YAW] = string(
            "Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
    joint_aliases[1][L_HAND] = string(
            "Device/SubDeviceList/LHand/Position/Actuator/Value");
    joint_aliases[1][L_HIP_PITCH] = string(
            "Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    joint_aliases[1][L_HIP_ROLL] = string(
            "Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    joint_aliases[1][L_HIP_YAW_PITCH] = string(
            "Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    joint_aliases[1][L_KNEE_PITCH] = string(
            "Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
    joint_aliases[1][L_SHOULDER_PITCH] = string(
            "Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
    joint_aliases[1][L_SHOULDER_ROLL] = string(
            "Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
    joint_aliases[1][L_WRIST_YAW] = string(
            "Device/SubDeviceList/LWristYaw/Position/Actuator/Value");
    joint_aliases[1][R_ANKLE_PITCH] = string(
            "Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    joint_aliases[1][R_ANKLE_ROLL] = string(
            "Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    joint_aliases[1][R_ELBOW_ROLL] = string(
            "Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
    joint_aliases[1][R_ELBOW_YAW] = string(
            "Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
    joint_aliases[1][R_HAND] = string(
            "Device/SubDeviceList/RHand/Position/Actuator/Value");
    joint_aliases[1][R_HIP_PITCH] = string(
            "Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    joint_aliases[1][R_HIP_ROLL] = string(
            "Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    joint_aliases[1][R_HIP_YAW_PITCH] = string(
            "Device/SubDeviceList/RHipYawPitch/Position/Actuator/Value");
    joint_aliases[1][R_KNEE_PITCH] = string(
            "Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    joint_aliases[1][R_SHOULDER_PITCH] = string(
            "Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    joint_aliases[1][R_SHOULDER_ROLL] = string(
            "Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    joint_aliases[1][R_WRIST_YAW] = string(
            "Device/SubDeviceList/RWristYaw/Position/Actuator/Value");

    // Create alias
    try {
        this->dcm->createAlias(joint_aliases);
    } catch (const AL::ALError &e) {
        throw ALERROR(getName(), "createPositionActuatorAlias()",
                      "Error when creating Alias : " + string(e.what()));
    }
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::createHardnessActuatorAlias() {
    AL::ALValue joint_aliases;
    // Alias for all joint stiffness
    joint_aliases.clear();
    joint_aliases.arraySetSize(2);
    joint_aliases[0] = string(
            "jointStiffness"); // Alias for all NUM_OF_JOINTS actuators
    joint_aliases[1].arraySetSize(NUM_OF_JOINTS);
    // stiffness list
    joint_aliases[1][HEAD_PITCH] = string(
            "Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value");
    joint_aliases[1][HEAD_YAW] = string(
            "Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value");
    joint_aliases[1][L_ANKLE_PITCH] = string(
            "Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value");
    joint_aliases[1][L_ANKLE_ROLL] = string(
            "Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value");
    joint_aliases[1][L_ELBOW_ROLL] = string(
            "Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value");
    joint_aliases[1][L_ELBOW_YAW] = string(
            "Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value");
    joint_aliases[1][L_HAND] = string(
            "Device/SubDeviceList/LHand/Hardness/Actuator/Value");
    joint_aliases[1][L_HIP_PITCH] = string(
            "Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value");
    joint_aliases[1][L_HIP_ROLL] = string(
            "Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value");
    joint_aliases[1][L_HIP_YAW_PITCH] = string(
            "Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value");
    joint_aliases[1][L_KNEE_PITCH] = string(
            "Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value");
    joint_aliases[1][L_SHOULDER_PITCH] = string(
            "Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value");
    joint_aliases[1][L_SHOULDER_ROLL] = string(
            "Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value");
    joint_aliases[1][L_WRIST_YAW] = string(
            "Device/SubDeviceList/LWristYaw/Hardness/Actuator/Value");
    joint_aliases[1][R_ANKLE_PITCH] = string(
            "Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value");
    joint_aliases[1][R_ANKLE_ROLL] = string(
            "Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value");
    joint_aliases[1][R_ELBOW_ROLL] = string(
            "Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value");
    joint_aliases[1][R_ELBOW_YAW] = string(
            "Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value");
    joint_aliases[1][R_HAND] = string(
            "Device/SubDeviceList/RHand/Hardness/Actuator/Value");
    joint_aliases[1][R_HIP_PITCH] = string(
            "Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value");
    joint_aliases[1][R_HIP_ROLL] = string(
            "Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value");
    joint_aliases[1][R_HIP_YAW_PITCH] = string(
            "Device/SubDeviceList/RHipYawPitch/Hardness/Actuator/Value");
    joint_aliases[1][R_KNEE_PITCH] = string(
            "Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value");
    joint_aliases[1][R_SHOULDER_PITCH] = string(
            "Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value");
    joint_aliases[1][R_SHOULDER_ROLL] = string(
            "Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value");
    joint_aliases[1][R_WRIST_YAW] = string(
            "Device/SubDeviceList/RWristYaw/Hardness/Actuator/Value");

    try {
        this->dcm->createAlias(joint_aliases);
    } catch (const AL::ALError &e) {
        throw ALERROR(getName(), "createHardnessActuatorAlias()",
                      "Error when creating Alias : " + string(e.what()));
    }
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::preparePositionActuatorCommand() {
    this->commands.arraySetSize(6);
    this->commands[0] = string("jointActuator");
    this->commands[1] = string("ClearAll");
    this->commands[2] = string("time-separate");
    this->commands[3] = 0;
    this->commands[4].arraySetSize(1);
    this->commands[5].arraySetSize(NUM_OF_JOINTS);

    for (int i = 0; i < NUM_OF_JOINTS; i++) {
        this->commands[5][i].arraySetSize(1);
    }
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::connectToDCMloop() {
    this->mem->GetValues(this->local_sensor_values);
    for (int i = 0; i < NUM_OF_JOINTS; ++i) {
        this->work_actuator_values->at(i) = this->local_sensor_values.at(i);
    }

    try {
        dcm_post_process_connection =
                this->getParentBroker()->getProxy(
                        "DCM")->getModule()->atPostProcess(
                        bind(
                                &HardwareAccessModule::synchronisedDCMcallback,
                                this));
    }
    catch (const AL::ALError &e) {
        throw ALERROR(this->getName(), "connectToDCMloop()",
                      "Error when connecting to DCM postProccess: " +
                              string(e.what()));
    }
}

////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::synchronisedDCMcallback() {
    int dcm_time;

    try {
        dcm_time = this->dcm->getTime(0);
    }
    catch (const AL::ALError &e) {
        throw ALERROR(this->getName(), "synchronisedDCMcallback()",
                      "Error on DCM getTime : " + string(e.what()));
    }

    this->commands[4][0] = dcm_time;
    {
        lock_guard<mutex> guard(this->actuator_mutex);
        for (int i = 0; i < NUM_OF_JOINTS; i++) {
            this->commands[5][i][0] = this->work_actuator_values->at(i);
        }
    }

    try {
        this->dcm->setAlias(commands);
    }
    catch (const AL::ALError &e) {
        throw ALERROR(this->getName(), "synchronisedDCMcallback()",
                      "Error when sending command to DCM : " +
                              string(e.what()));
    }
}

////////////////////////////////////////////////////////////////////////////////

bool HardwareAccessModule::checkDevices() {
    return this->top_camera->isStartOk() && this->bottom_camera->isStartOk();
}

/////////////////////////////////////////////////////////////////////////////////

AL::ALValue HardwareAccessModule::getImageBufferTop() {
    unsigned char *image = this->top_camera->captureImage();
    AL::ALValue alimage;
    alimage.arraySetSize(1);
    alimage[0] = string(reinterpret_cast<char *>(image));
    return alimage;
}

////////////////////////////////////////////////////////////////////////////////

AL::ALValue HardwareAccessModule::getImageBufferBot() {
    unsigned char *image = this->bottom_camera->captureImage();
    AL::ALValue alimage;
    alimage.arraySetSize(1);
    alimage[0] = string(reinterpret_cast<char *>(image));
    return alimage;
}

//////////////////////////////////////////////////////////////////////////////////

void HardwareAccessModule::stopStream() {
    this->top_camera->stopCapturing();
    this->bottom_camera->stopCapturing();
}

