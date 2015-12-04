//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Joints.h>

using namespace rd;
using namespace AL;
using namespace boost;
using namespace std;


Joints::Joints(shared_ptr<AL::ALMemoryProxy> memory,
               const shared_ptr<DCMProxy> dcm)
        : Actuator(string("Joints"), memory, dcm) {
    this->keys.resize(JOINTS_COUNT);
    this->position_in_list.resize(JOINTS_COUNT);
    this->position_out_list.resize(JOINTS_COUNT);
    this->hardness_list.resize(JOINTS_COUNT);

    this->keys[HEAD_YAW] = string("HEAD_YAW");
    this->keys[HEAD_PITCH] = string("HEAD_PITCH");
    this->keys[L_SHOULDER_PITCH] = string("L_SHOULDER_PITCH");
    this->keys[L_SHOULDER_ROLL] = string("L_SHOULDER_ROLL");
    this->keys[L_ELBOW_YAW] = string("L_ELBOW_YAW");
    this->keys[L_ELBOW_ROLL] = string("L_ELBOW_ROLL");
    this->keys[L_WRIST_YAW] = string("L_WRIST_YAW");
    this->keys[L_HAND] = string("L_HAND");
    this->keys[L_HIP_YAW_PITCH] = string("L_HIP_YAW_PITCH");
    this->keys[L_HIP_ROLL] = string("L_HIP_ROLL");
    this->keys[L_HIP_PITCH] = string("L_HIP_PITCH");
    this->keys[L_KNEE_PITCH] = string("L_KNEE_PITCH");
    this->keys[L_ANKLE_PITCH] = string("L_ANKLE_PITCH");
    this->keys[L_ANKLE_ROLL] = string("L_ANKLE_ROLL");
    this->keys[R_HIP_YAW_PITCH] = string("R_HIP_YAW_PITCH");
    this->keys[R_HIP_ROLL] = string("R_HIP_ROLL");
    this->keys[R_HIP_PITCH] = string("R_HIP_PITCH");
    this->keys[R_KNEE_PITCH] = string("R_KNEE_PITCH");
    this->keys[R_ANKLE_PITCH] = string("R_ANKLE_PITCH");
    this->keys[R_ANKLE_ROLL] = string("R_ANKLE_ROLL");
    this->keys[R_SHOULDER_PITCH] = string("R_SHOULDER_PITCH");
    this->keys[R_SHOULDER_ROLL] = string("R_SHOULDER_ROLL");
    this->keys[R_ELBOW_YAW] = string("R_ELBOW_YAW");
    this->keys[R_ELBOW_ROLL] = string("R_ELBOW_ROLL");
    this->keys[R_WRIST_YAW] = string("R_WRIST_YAW");
    this->keys[R_HAND] = string("R_HAND");

    this->position_in_list[HEAD_YAW] = string(
            "Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
    this->position_in_list[HEAD_PITCH] = string(
            "Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
    this->position_in_list[L_SHOULDER_PITCH] = string(
            "Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
    this->position_in_list[L_SHOULDER_ROLL] = string(
            "Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
    this->position_in_list[L_ELBOW_YAW] = string(
            "Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
    this->position_in_list[L_ELBOW_ROLL] = string(
            "Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
    this->position_in_list[L_WRIST_YAW] = string(
            "Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
    this->position_in_list[L_HAND] = string(
            "Device/SubDeviceList/LHand/Position/Sensor/Value");
    this->position_in_list[L_HIP_YAW_PITCH] = string(
            "Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
    this->position_in_list[L_HIP_ROLL] = string(
            "Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
    this->position_in_list[L_HIP_PITCH] = string(
            "Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
    this->position_in_list[L_KNEE_PITCH] = string(
            "Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
    this->position_in_list[L_ANKLE_PITCH] = string(
            "Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
    this->position_in_list[L_ANKLE_ROLL] = string(
            "Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
    this->position_in_list[R_ANKLE_PITCH] = string(
            "Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
    this->position_in_list[R_ANKLE_ROLL] = string(
            "Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
    this->position_in_list[R_ELBOW_ROLL] = string(
            "Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
    this->position_in_list[R_ELBOW_YAW] = string(
            "Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
    this->position_in_list[R_HAND] = string(
            "Device/SubDeviceList/RHand/Position/Sensor/Value");
    this->position_in_list[R_HIP_PITCH] = string(
            "Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
    this->position_in_list[R_HIP_ROLL] = string(
            "Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
    this->position_in_list[R_HIP_YAW_PITCH] = string(
            "Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    this->position_in_list[R_KNEE_PITCH] = string(
            "Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    this->position_in_list[R_SHOULDER_PITCH] = string(
            "Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
    this->position_in_list[R_SHOULDER_ROLL] = string(
            "Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
    this->position_in_list[R_WRIST_YAW] = string(
            "Device/SubDeviceList/RWristYaw/Position/Sensor/Value");

    this->position_out_list[HEAD_PITCH] = string(
            "Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    this->position_out_list[HEAD_YAW] = string(
            "Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
    this->position_out_list[L_ANKLE_PITCH] = string(
            "Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    this->position_out_list[L_ANKLE_ROLL] = string(
            "Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    this->position_out_list[L_ELBOW_ROLL] = string(
            "Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
    this->position_out_list[L_ELBOW_YAW] = string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
    this->position_out_list[L_HAND] = string("Device/SubDeviceList/LHand/Position/Actuator/Value");
    this->position_out_list[L_HIP_PITCH] = string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    this->position_out_list[L_HIP_ROLL] = string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    this->position_out_list[L_HIP_YAW_PITCH] = string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    this->position_out_list[L_KNEE_PITCH] = string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
    this->position_out_list[L_SHOULDER_PITCH] = string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
    this->position_out_list[L_SHOULDER_ROLL] = string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
    this->position_out_list[L_WRIST_YAW] = string("Device/SubDeviceList/LWristYaw/Position/Actuator/Value");
    this->position_out_list[R_ANKLE_PITCH] = string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    this->position_out_list[R_ANKLE_ROLL] = string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    this->position_out_list[R_ELBOW_ROLL] = string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
    this->position_out_list[R_ELBOW_YAW] = string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
    this->position_out_list[R_HAND] = string("Device/SubDeviceList/RHand/Position/Actuator/Value");
    this->position_out_list[R_HIP_PITCH] = string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    this->position_out_list[R_HIP_ROLL] = string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    this->position_out_list[R_HIP_YAW_PITCH] = string("Device/SubDeviceList/RHipYawPitch/Position/Actuator/Value");
    this->position_out_list[R_KNEE_PITCH] = string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    this->position_out_list[R_SHOULDER_PITCH] = string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    this->position_out_list[R_SHOULDER_ROLL] = string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    this->position_out_list[R_WRIST_YAW] = string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");

    this->hardness_list[HEAD_PITCH] = string("Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value");
    this->hardness_list[HEAD_YAW] = string("Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value");
    this->hardness_list[L_ANKLE_PITCH] = string("Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value");
    this->hardness_list[L_ANKLE_ROLL] = string("Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value");
    this->hardness_list[L_ELBOW_ROLL] = string("Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value");
    this->hardness_list[L_ELBOW_YAW] = string("Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value");
    this->hardness_list[L_HAND] = string("Device/SubDeviceList/LHand/Hardness/Actuator/Value");
    this->hardness_list[L_HIP_PITCH] = string("Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value");
    this->hardness_list[L_HIP_ROLL] = string("Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value");
    this->hardness_list[L_HIP_YAW_PITCH] = string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value");
    this->hardness_list[L_KNEE_PITCH] = string("Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value");
    this->hardness_list[L_SHOULDER_PITCH] = string("Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value");
    this->hardness_list[L_SHOULDER_ROLL] = string("Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value");
    this->hardness_list[L_WRIST_YAW] = string("Device/SubDeviceList/LWristYaw/Hardness/Actuator/Value");
    this->hardness_list[R_ANKLE_PITCH] = string("Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value");
    this->hardness_list[R_ANKLE_ROLL] = string("Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value");
    this->hardness_list[R_ELBOW_ROLL] = string("Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value");
    this->hardness_list[R_ELBOW_YAW] = string("Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value");
    this->hardness_list[R_HAND] = string("Device/SubDeviceList/RHand/Hardness/Actuator/Value");
    this->hardness_list[R_HIP_PITCH] = string("Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value");
    this->hardness_list[R_HIP_ROLL] = string("Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value");
    this->hardness_list[R_HIP_YAW_PITCH] = string("Device/SubDeviceList/RHipYawPitch/Hardness/Actuator/Value");
    this->hardness_list[R_KNEE_PITCH] = string("Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value");
    this->hardness_list[R_SHOULDER_PITCH] = string("Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value");
    this->hardness_list[R_SHOULDER_ROLL] = string("Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value");
    this->hardness_list[R_WRIST_YAW] = string("Device/SubDeviceList/RWristYaw/Hardness/Actuator/Value");

    Joints::initKeysMap(this->position_in_map, this->keys, this->position_in_list);
    Joints::initKeysMap(this->position_out_map, this->keys, this->position_out_list);
    Joints::initKeysMap(this->hardness_map, this->keys, this->hardness_list);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Joints::initKeysMap(std::map<std::string, std::string> &map,
                         const std::vector<std::string> &keys,
                         const std::vector<std::string> &values) {
    vector<std::string>::const_iterator first_it = keys.begin();
    vector<std::string>::const_iterator second_it = values.begin();
    while (first_it != keys.end() && second_it != values.end()) {
        map.insert(make_pair<std::string, std::string>(*first_it, *second_it));
        ++first_it;
        ++second_it;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::vector<std::string> &Joints::getInputKeys() const {
    return this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::set(const std::vector<std::string> &keys, const std::vector<float> &values, int time_offset) {
    return this->setValues(this->position_in_map, keys, values, time_offset);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::set(const std::vector<int> &keys, const std::vector<float> &values, int time_offset) {
    return this->setValues(this->position_in_list, keys, values, time_offset);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::vector<std::string> &Joints::getOutputKeys() const {
    return this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<float> Joints::get(const std::vector<int> &keys) {
    return SensorData<float>(this->getValues(this->position_out_list, keys),
                             this->clock(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<float> Joints::get(const std::vector<std::string> &keys) {
    return SensorData<float>(this->getValues(this->position_out_map, keys),
                             this->clock(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<float> Joints::get() {
    return SensorData<float>(
            this->getValues(this->position_out_map, this->keys),
            this->clock(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(float value, int time_offset) {
    static vector<float> values(JOINTS_COUNT, value);
    return this->setValues(this->hardness_map, keys, values, time_offset);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(const std::vector<std::string> &keys,
                         const std::vector<float> &values,
                         int time_offset) {
    return this->setValues(this->hardness_map, keys, values, time_offset);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(const std::vector<int> &keys, const std::vector<float> &values, int time_offset) {
    return this->setValues(this->hardness_list, keys, values, time_offset);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<float> Joints::getHardness(const std::vector<int> &keys) {
    return SensorData<float>(this->getValues(this->hardness_list, keys), this->clock(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<float> Joints::getHardness(
        const std::vector<std::string> &keys) {
    return SensorData<float>(this->getValues(this->hardness_map, keys), this->clock(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<float> Joints::getHardness() {
    return SensorData<float>(this->getValues(this->hardness_map, this->keys), this->clock(0));
}
