//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Joints.h>

using namespace rd;
using namespace AL;
using namespace boost;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string Joints::DCM_POSITION_ALIAS("rd/actuators/joints/positions");
const std::string Joints::DCM_HARDNESS_ALIAS("rd/actuators/joints/hardness");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Joints::initKeysMap(map<string, string> &container,
                         const vector<string> &keys,
                         const vector<string> &values) {
    vector<string>::const_iterator first_it = keys.begin();
    vector<string>::const_iterator second_it = values.begin();
    while (first_it != keys.end() && second_it != values.end()) {
        container.insert(make_pair<std::string, std::string>(*first_it, *second_it));
        ++first_it;
        ++second_it;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Joints::makeAlias(const string &name, const ALValue &keys) {
    ALValue cmd;
    cmd.arraySetSize(2);
    cmd[0] = name;
    cmd[1] = keys;
    this->dcm->createAlias(cmd);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Joints::Joints(boost::shared_ptr<AL::DCMProxy> dcm,
               boost::shared_ptr<AL::ALMemoryProxy> memory)
        : dcm(dcm), mem(memory),
          keys(JOINTS_COUNT) {
    // Joint keys list
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
    // Input DCM position keys list
    this->position_in_list.arraySetSize(JOINTS_COUNT);
    this->position_in_list[HEAD_YAW] = string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
    this->position_in_list[HEAD_PITCH] = string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
    this->position_in_list[L_SHOULDER_PITCH] = string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
    this->position_in_list[L_SHOULDER_ROLL] = string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
    this->position_in_list[L_ELBOW_YAW] = string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
    this->position_in_list[L_ELBOW_ROLL] = string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
    this->position_in_list[L_WRIST_YAW] = string("Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
    this->position_in_list[L_HAND] = string("Device/SubDeviceList/LHand/Position/Sensor/Value");
    this->position_in_list[L_HIP_YAW_PITCH] = string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
    this->position_in_list[L_HIP_ROLL] = string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
    this->position_in_list[L_HIP_PITCH] = string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
    this->position_in_list[L_KNEE_PITCH] = string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
    this->position_in_list[L_ANKLE_PITCH] = string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
    this->position_in_list[L_ANKLE_ROLL] = string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
    this->position_in_list[R_ANKLE_PITCH] = string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
    this->position_in_list[R_ANKLE_ROLL] = string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
    this->position_in_list[R_ELBOW_ROLL] = string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
    this->position_in_list[R_ELBOW_YAW] = string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
    this->position_in_list[R_HAND] = string("Device/SubDeviceList/RHand/Position/Sensor/Value");
    this->position_in_list[R_HIP_PITCH] = string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
    this->position_in_list[R_HIP_ROLL] = string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
    this->position_in_list[R_HIP_YAW_PITCH] = string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    this->position_in_list[R_KNEE_PITCH] = string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    this->position_in_list[R_SHOULDER_PITCH] = string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
    this->position_in_list[R_SHOULDER_ROLL] = string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
    this->position_in_list[R_WRIST_YAW] = string("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");
    // Output DCM position keys list
    this->position_out_list.arraySetSize(JOINTS_COUNT);
    this->position_out_list[HEAD_PITCH] = string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    this->position_out_list[HEAD_YAW] = string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
    this->position_out_list[L_ANKLE_PITCH] = string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    this->position_out_list[L_ANKLE_ROLL] = string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    this->position_out_list[L_ELBOW_ROLL] = string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
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
    // DCM hardness keys list
    this->hardness_list.arraySetSize(JOINTS_COUNT);
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
    // Create output keys map
    for (int i = 0; i < keys.size(); ++i) this->out_map.insert(make_pair<string, int>(keys[i], i));
    // Create input keys map
    this->initKeysMap(this->position_map, this->keys, this->position_in_list);
    this->initKeysMap(this->hardness_map, this->keys, this->hardness_list);
    // Prepare DCM aliaces
    this->makeAlias(Joints::DCM_POSITION_ALIAS, this->position_out_list);
    this->makeAlias(Joints::DCM_HARDNESS_ALIAS, this->hardness_list);
    // Prepare DCM command
    this->dcm_cmd.arraySetSize(6);
    this->dcm_cmd[1] = string("ClearAll");
    this->dcm_cmd[2] = string("time-separate");
    this->dcm_cmd[3] = 0;
    this->dcm_cmd[4].arraySetSize(1);
    this->dcm_cmd[5].arraySetSize(keys.size());
    for (int i = 0; i < keys.size(); ++i) this->dcm_cmd[5][i].arraySetSize(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string> &Joints::getKeys() const {
    return this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setPosition(const vector<string> &keys,
                         const vector<double> &values) {
    bool success = true;
    int time = this->dcm->getTime(0);
    lock_guard<mutex> lock(this->synch);
    this->dcm_cmd[0] = Joints::DCM_POSITION_ALIAS;
    // Update time
    this->dcm_cmd[4][0] = time;
    // Read data from sensors
    ALValue data = this->mem->getListData(this->position_out_list);
    for (int i = 0; i < JOINTS_COUNT; ++i) this->dcm_cmd[5][i][0] = data[i];
    // Update positions
    for (int i = 0; i < keys.size(); ++i) {
        try {
            int index = this->out_map[keys[i]];
            this->dcm_cmd[5][index][0] = values[i];
        } catch (out_of_range &e) {
            success = false;
            continue;
        }
    }
    // Send cmd
    this->dcm->setAlias(dcm_cmd);
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setPosition(const std::vector<int> &keys,
                         const std::vector<double> &values) {
    bool success = true;
    int index;
    int time = this->dcm->getTime(0);
    lock_guard<mutex> lock(this->synch);
    this->dcm_cmd[0] = Joints::DCM_POSITION_ALIAS;
    // Update time
    this->dcm_cmd[4][0] = time;
    // Read data from sensors
    ALValue data = this->mem->getListData(this->position_out_list);
    for (int i = 0; i < JOINTS_COUNT; ++i) this->dcm_cmd[5][i][0] = data[i];
    // Update positions
    for (int i = 0; i < keys.size(); ++i) {
        try {
            index = keys[i];
            this->dcm_cmd[5][index][0] = values[i];
        } catch (out_of_range &e) {
            success = false;
            continue;
        }
    }
    // Send cmd
    this->dcm->setAlias(dcm_cmd);
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double> Joints::getPosition(const vector<int> &keys) {
    unsigned int length = keys.size();
    AL::ALValue data;
    data.arraySetSize(length);
    for (int i = 0; i < length; ++i) {
        try {
            data[i] = this->position_in_list[keys.at(i)];
        } catch (...) {
            return SensorData<double>(make_shared<std::vector<double> >(), 0);
        }
    }

    data = this->mem->getListData(data);
    boost::shared_ptr<std::vector<double> > res = boost::make_shared<std::vector<double> >(length);
    for (unsigned int i = 0; i < length; ++i) res->at(i) = data[i];
    return SensorData<double>(res, this->dcm->getTime(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double> Joints::getPosition(const vector<string> &keys) {
    unsigned int length = keys.size();
    AL::ALValue data;
    data.arraySetSize(length);
    for (int i = 0; i < length; ++i) {
        try {
            data[i] = this->position_map[keys.at(i)];
        } catch (...) {
            return SensorData<double>(make_shared<std::vector<double> >(), 0);
        }
    }

    data = this->mem->getListData(data);
    boost::shared_ptr<std::vector<double> > res = boost::make_shared<std::vector<double> >(length);
    for (unsigned int i = 0; i < length; ++i) res->at(i) = data[i];
    return SensorData<double>(res, this->dcm->getTime(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double> Joints::getPosition() {
    AL::ALValue data = this->mem->getListData(this->position_in_list);
    boost::shared_ptr<std::vector<double> > res = boost::make_shared<std::vector<double> >(this->keys.size());
    for (unsigned int i = 0; i < this->keys.size(); ++i) res->at(i) = data[i];
    return SensorData<double>(res, this->dcm->getTime(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(double value) {
    int time = this->dcm->getTime(0);
    lock_guard<mutex> lock(this->synch);
    this->dcm_cmd[0] = Joints::DCM_HARDNESS_ALIAS;
    // Update time
    this->dcm_cmd[4][0] = time;
    // Update positions
    for (int i = 0; i < keys.size(); ++i) this->dcm_cmd[5][i][0] = value;
    // Send cmd
    this->dcm->setAlias(dcm_cmd);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(const std::vector<std::string> &keys, const std::vector<double> &values) {
    bool success = true;
    int index;
    int time = this->dcm->getTime(0);
    lock_guard<mutex> lock(this->synch);
    this->dcm_cmd[0] = Joints::DCM_HARDNESS_ALIAS;
    // Update time
    this->dcm_cmd[4][0] = time;
    // Read data from sensors
    ALValue data = this->mem->getListData(this->hardness_list);
    for (int i = 0; i < JOINTS_COUNT; ++i) this->dcm_cmd[5][i][0] = data[i];
    // Update positions
    for (int i = 0; i < JOINTS_COUNT; ++i) {
        try {
            index = this->out_map[keys[i]];
            this->dcm_cmd[5][index][0] = values[i];
        } catch (out_of_range &e) {
            success = false;
            continue;
        }
    }
    // Send cmd

    this->dcm->setAlias(dcm_cmd);
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(const std::vector<int> &keys,
                         const std::vector<double> &values) {
    bool success = true;
    int index;
    int time = this->dcm->getTime(0);
    lock_guard<mutex> lock(this->synch);
    this->dcm_cmd[0] = Joints::DCM_HARDNESS_ALIAS;
    // Update time
    this->dcm_cmd[4][0] = time;
    // Read data from sensors
    this->dcm_cmd[5] = this->mem->getListData(this->hardness_list);
    // Update positions
    for (int i = 0; i < JOINTS_COUNT; ++i) {
        try {
            index = keys[i];
            this->dcm_cmd[5][index][0] = values[i];
        } catch (out_of_range &e) {
            success = false;
            continue;
        }
    }
    // Send cmd
    this->dcm->setAlias(dcm_cmd);
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double> Joints::getHardness(const vector<int> &keys) {
    unsigned int length = JOINTS_COUNT;
    AL::ALValue data;
    data.arraySetSize(length);
    for (int i = 0; i < length; ++i) {
        try {
            data[i] = this->hardness_list[keys.at(i)];
        } catch (...) {
            return SensorData<double>(make_shared<std::vector<double> >(), 0);
        }
    }

    data = this->mem->getListData(data);
    boost::shared_ptr<std::vector<double> > res = boost::make_shared<std::vector<double> >(length);
    for (unsigned int i = 0; i < length; ++i) res->at(i) = data[i];
    return SensorData<double>(res, this->dcm->getTime(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double> Joints::getHardness(const vector<string> &keys) {
    unsigned int length = JOINTS_COUNT;
    AL::ALValue data;
    data.arraySetSize(length);
    for (int i = 0; i < length; ++i) {
        try {
            data[i] = this->hardness_map[keys.at(i)];
        } catch (...) {
            return SensorData<double>(make_shared<std::vector<double> >(), 0);
        }
    }

    data = this->mem->getListData(data);
    boost::shared_ptr<std::vector<double> > res = boost::make_shared<std::vector<double> >(length);
    for (unsigned int i = 0; i < length; ++i) res->at(i) = data[i];
    return SensorData<double>(res, this->dcm->getTime(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double> Joints::getHardness() {
    AL::ALValue data = this->mem->getListData(this->hardness_list);
    boost::shared_ptr<std::vector<double> > res = boost::make_shared<std::vector<double> >(JOINTS_COUNT);
    for (unsigned int i = 0; i < JOINTS_COUNT; ++i) res->at(i) = data[i];
    return SensorData<double>(res, this->dcm->getTime(0));
}