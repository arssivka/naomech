//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Joints.h>

using namespace rd;
using namespace AL;
using namespace boost;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string Joints::DCM_HARDNESS_ALIAS("rd/actuators/joints/hardness");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Joints::makeAlias(const string &name, const ALValue &keys) {
    ALValue cmd;
    cmd.arraySetSize(2);
    cmd[0] = name;
    cmd[1] = keys;
    m_dcm->createAlias(cmd);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Joints::Joints(shared_ptr<ALBroker> broker)
        : m_dcm(make_shared<DCMProxy>(broker)), m_mem(make_shared<ALMemoryProxy>(broker)),
          m_hw(make_shared<ALProxy>(broker, "rd/hwcontroller")), m_keys(JOINTS_COUNT) {
    // Joint keys list
    m_keys[HEAD_YAW] = string("HEAD_YAW");
    m_keys[HEAD_PITCH] = string("HEAD_PITCH");
    m_keys[L_SHOULDER_PITCH] = string("L_SHOULDER_PITCH");
    m_keys[L_SHOULDER_ROLL] = string("L_SHOULDER_ROLL");
    m_keys[L_ELBOW_YAW] = string("L_ELBOW_YAW");
    m_keys[L_ELBOW_ROLL] = string("L_ELBOW_ROLL");
    m_keys[L_WRIST_YAW] = string("L_WRIST_YAW");
    m_keys[L_HAND] = string("L_HAND");
    m_keys[L_HIP_YAW_PITCH] = string("L_HIP_YAW_PITCH");
    m_keys[L_HIP_ROLL] = string("L_HIP_ROLL");
    m_keys[L_HIP_PITCH] = string("L_HIP_PITCH");
    m_keys[L_KNEE_PITCH] = string("L_KNEE_PITCH");
    m_keys[L_ANKLE_PITCH] = string("L_ANKLE_PITCH");
    m_keys[L_ANKLE_ROLL] = string("L_ANKLE_ROLL");
    m_keys[R_HIP_YAW_PITCH] = string("R_HIP_YAW_PITCH");
    m_keys[R_HIP_ROLL] = string("R_HIP_ROLL");
    m_keys[R_HIP_PITCH] = string("R_HIP_PITCH");
    m_keys[R_KNEE_PITCH] = string("R_KNEE_PITCH");
    m_keys[R_ANKLE_PITCH] = string("R_ANKLE_PITCH");
    m_keys[R_ANKLE_ROLL] = string("R_ANKLE_ROLL");
    m_keys[R_SHOULDER_PITCH] = string("R_SHOULDER_PITCH");
    m_keys[R_SHOULDER_ROLL] = string("R_SHOULDER_ROLL");
    m_keys[R_ELBOW_YAW] = string("R_ELBOW_YAW");
    m_keys[R_ELBOW_ROLL] = string("R_ELBOW_ROLL");
    m_keys[R_WRIST_YAW] = string("R_WRIST_YAW");
    m_keys[R_HAND] = string("R_HAND");
    // DCM hardness keys list
    m_hardness_list.arraySetSize(JOINTS_COUNT);
    m_hardness_list[HEAD_PITCH] = string("Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value");
    m_hardness_list[HEAD_YAW] = string("Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value");
    m_hardness_list[L_ANKLE_PITCH] = string("Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value");
    m_hardness_list[L_ANKLE_ROLL] = string("Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value");
    m_hardness_list[L_ELBOW_ROLL] = string("Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value");
    m_hardness_list[L_ELBOW_YAW] = string("Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value");
    m_hardness_list[L_HAND] = string("Device/SubDeviceList/LHand/Hardness/Actuator/Value");
    m_hardness_list[L_HIP_PITCH] = string("Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value");
    m_hardness_list[L_HIP_ROLL] = string("Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value");
    m_hardness_list[L_HIP_YAW_PITCH] = string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value");
    m_hardness_list[L_KNEE_PITCH] = string("Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value");
    m_hardness_list[L_SHOULDER_PITCH] = string("Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value");
    m_hardness_list[L_SHOULDER_ROLL] = string("Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value");
    m_hardness_list[L_WRIST_YAW] = string("Device/SubDeviceList/LWristYaw/Hardness/Actuator/Value");
    m_hardness_list[R_ANKLE_PITCH] = string("Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value");
    m_hardness_list[R_ANKLE_ROLL] = string("Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value");
    m_hardness_list[R_ELBOW_ROLL] = string("Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value");
    m_hardness_list[R_ELBOW_YAW] = string("Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value");
    m_hardness_list[R_HAND] = string("Device/SubDeviceList/RHand/Hardness/Actuator/Value");
    m_hardness_list[R_HIP_PITCH] = string("Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value");
    m_hardness_list[R_HIP_ROLL] = string("Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value");
    m_hardness_list[R_HIP_YAW_PITCH] = string("Device/SubDeviceList/RHipYawPitch/Hardness/Actuator/Value");
    m_hardness_list[R_KNEE_PITCH] = string("Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value");
    m_hardness_list[R_SHOULDER_PITCH] = string("Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value");
    m_hardness_list[R_SHOULDER_ROLL] = string("Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value");
    m_hardness_list[R_WRIST_YAW] = string("Device/SubDeviceList/RWristYaw/Hardness/Actuator/Value");
    // Create output keys map
    for (int i = 0; i < m_keys.size(); ++i) {
        m_out_map.insert(make_pair(m_keys[i], i));
        m_hardness_map.insert(make_pair(m_keys[i], m_hardness_list[i]));
    }
    this->makeAlias(Joints::DCM_HARDNESS_ALIAS, m_hardness_list);
    // Prepare DCM command
    m_dcm_cmd.arraySetSize(6);
    m_dcm_cmd[1] = string("ClearAll");
    m_dcm_cmd[2] = string("time-separate");
    m_dcm_cmd[3] = 0;
    m_dcm_cmd[4].arraySetSize(1);
    m_dcm_cmd[5].arraySetSize(m_keys.size());
    for (int i = 0; i < m_keys.size(); ++i) m_dcm_cmd[5][i].arraySetSize(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string> &Joints::getKeys() const {
    return m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setPositions(const vector<string>& keys,
                          const vector<double>& values) {
    bool success = true;
    // Read data from sensors
    ALValue data = m_hw->call<ALValue>("getJointData");
    // Update positions
    for (int i = 0; i < keys.size(); ++i) {
        try {
            int index = m_out_map[keys[i]];
            data[index] = values[i];
        } catch (out_of_range &e) {
            success = false;
            continue;
        }
    }
    // Send cmd
    m_hw->callVoid("setJointData", data);
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setPositions(const IntegerKeyVector& keys,
                          const ValuesVector& values) {
    bool success = true;
    // Read data from sensors
    ALValue data = m_hw->call<ALValue>("getJointData");
    // Update positions
    for (int i = 0; i < keys.size(); ++i) {
        try {
            int index = keys[i];
            data[index] = values[i];
        } catch (out_of_range &e) {
            success = false;
            continue;
        }
    }
    // Send cmd
    m_hw->callVoid("setJointData", data);
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double>::Ptr Joints::getPositions(const vector<int>& keys) {
    ALValue data = m_hw->call<ALValue>("getJointData");
    unsigned int length = keys.size();
    SensorData<double>::Ptr res = make_shared<SensorData<double> >(length, m_dcm->getTime(0));
    for (int i = 0; i < length; ++i) {
        if (keys[i] > JOINTS_COUNT)
            return make_shared<SensorData<double> >(0, m_dcm->getTime(0));
        res->data[i] = data[keys[i]];
    }
    return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double>::Ptr Joints::getPositions(const vector<string>& keys) {
    vector<int> k(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        const map<string, int>::iterator& found = m_out_map.find(keys[i]);
        if (found == m_out_map.end())
            return make_shared<SensorData<double> >(0, m_dcm->getTime(0));
        k[i] = found->second;
    }
    return this->getPositions(k);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double>::Ptr Joints::getPositions() {
    ALValue data = m_hw->call<ALValue>("getJointData");
    SensorData<double>::Ptr res = make_shared<SensorData<double> >(JOINTS_COUNT, m_dcm->getTime(0));
    for (unsigned int i = 0; i < JOINTS_COUNT; ++i) res->data[i] = data[i];
    return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(double value) {
    int time = m_dcm->getTime(0);
    lock_guard<mutex> lock(m_synch);
    m_dcm_cmd[0] = Joints::DCM_HARDNESS_ALIAS;
    // Update time
    m_dcm_cmd[4][0] = time;
    // Update positions
    for (int i = 0; i < m_keys.size(); ++i) m_dcm_cmd[5][i][0] = value;
    // Send cmd
    m_dcm->setAlias(m_dcm_cmd);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(const StringKeyVector &keys, const ValuesVector &values) {
    vector<int> keys_i(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        map<string, int>::const_iterator key_num;
        key_num = m_out_map.find(keys[i]);
        if (key_num == m_out_map.end())
            return make_shared<SensorData<double> >();
        keys_i[i] = key_num->second;
    }
    return setHardness(keys_i, values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Joints::setHardness(const IntegerKeyVector &keys, const ValuesVector &values) {
    if (keys.size() != values.size() || keys.empty())
        return false;
    int time = m_dcm->getTime(0);
    ALValue data = m_mem->getListData(m_hardness_list);
    lock_guard<mutex> lock(m_synch);
    m_dcm_cmd[0] = Joints::DCM_HARDNESS_ALIAS;
    // Update time
    m_dcm_cmd[4][0] = time;
    // Read data from sensors
    m_dcm_cmd[5].arraySetSize(JOINTS_COUNT);
    // Update positions
    for (int i = 0; i < JOINTS_COUNT; ++i)
        m_dcm_cmd[5][i][0] = data[i];
    for (int i = 0; i < keys.size(); ++i) {
        int index = keys[i];
        if (index < 0 || index >= JOINTS_COUNT)
            return false;
        m_dcm_cmd[5][index][0] = values[i];
    }
    // Send cmd
    m_dcm->setAlias(m_dcm_cmd);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double>::Ptr Joints::getHardness(const vector<int> &keys) {
    ALValue data;
    data.arraySetSize(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        int index = keys[i];
        if (index < 0 || index >= JOINTS_COUNT)
            return make_shared<SensorData<double> >();
        data[i] = m_hardness_list[index];
    }
    data = m_mem->getListData(data);
    SensorData<double>::Ptr res = make_shared<SensorData<double> >(keys.size(), m_dcm->getTime(0));
    for (unsigned int i = 0; i < keys.size(); ++i) res->data[i] = data[i];
    return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double>::Ptr Joints::getHardness(const vector<string> &keys) {
    vector<int> keys_i(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        map<string, int>::const_iterator key_num;
        key_num = m_out_map.find(keys[i]);
        if (key_num == m_out_map.end())
            return make_shared<SensorData<double> >();
        keys_i[i] = key_num->second;
    }
    return getHardness(keys_i);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double>::Ptr Joints::getHardness() {
    ALValue data = m_mem->getListData(m_hardness_list);
    SensorData<double>::Ptr res = make_shared<SensorData<double> >(JOINTS_COUNT, m_dcm->getTime(0));
    for (unsigned int i = 0; i < JOINTS_COUNT; ++i) res->data[i] = data[i];
    return res;
}
