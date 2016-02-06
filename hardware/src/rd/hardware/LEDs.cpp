//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/LEDs.h>


using namespace rd;
using namespace std;
using namespace boost;
using namespace AL;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LEDs::initKeysMap(map<string, string> &container,
                       const vector<string> &keys,
                       const vector<string> &values) {
    vector<string>::const_iterator first_it = keys.begin();
    vector<string>::const_iterator second_it = values.begin();
    while (first_it != keys.end() && second_it != values.end()) {
        container.insert(make_pair(*first_it, *second_it));
        ++first_it;
        ++second_it;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LEDs::LEDs(shared_ptr<ALBroker> broker)
        : m_mem(make_shared<ALMemoryProxy>(broker)), m_dcm(make_shared<DCMProxy>(broker)), m_keys(LEDS_COUNT),
          m_leds_list(LEDS_COUNT) {
    m_keys[CHEST_BOARD_RED] = string("CHEST_BOARD_RED");
    m_keys[CHEST_BOARD_GREEN] = string("CHEST_BOARD_GREEN");
    m_keys[CHEST_BOARD_BLUE] = string("CHEST_BOARD_BLUE");
    m_keys[L_EAR_0_DEG] = string("L_EAR_0_DEG");
    m_keys[L_EAR_108_DEG] = string("L_EAR_108_DEG");
    m_keys[L_EAR_144_DEG] = string("L_EAR_144_DEG");
    m_keys[L_EAR_180_DEG] = string("L_EAR_180_DEG");
    m_keys[L_EAR_216_DEG] = string("L_EAR_216_DEG");
    m_keys[L_EAR_252_DEG] = string("L_EAR_252_DEG");
    m_keys[L_EAR_288_DEG] = string("L_EAR_288_DEG");
    m_keys[L_EAR_324_DEG] = string("L_EAR_324_DEG");
    m_keys[L_EAR_36_DEG] = string("L_EAR_36_DEG");
    m_keys[L_EAR_72_DEG] = string("L_EAR_72_DEG");
    m_keys[R_EAR_0_DEG] = string("R_EAR_0_DEG");
    m_keys[R_EAR_108_DEG] = string("R_EAR_108_DEG");
    m_keys[R_EAR_144_DEG] = string("R_EAR_144_DEG");
    m_keys[R_EAR_180_DEG] = string("R_EAR_180_DEG");
    m_keys[R_EAR_216_DEG] = string("R_EAR_216_DEG");
    m_keys[R_EAR_252_DEG] = string("R_EAR_252_DEG");
    m_keys[R_EAR_288_DEG] = string("R_EAR_288_DEG");
    m_keys[R_EAR_324_DEG] = string("R_EAR_324_DEG");
    m_keys[R_EAR_36_DEG] = string("R_EAR_36_DEG");
    m_keys[R_EAR_72_DEG] = string("R_EAR_72_DEG");
    m_keys[L_FACE_RED_0_DEG] = string("L_FACE_RED_0_DEG");
    m_keys[L_FACE_GREEN_0_DEG] = string("L_FACE_GREEN_0_DEG");
    m_keys[L_FACE_BLUE_0_DEG] = string("L_FACE_BLUE_0_DEG");
    m_keys[L_FACE_RED_135_DEG] = string("L_FACE_RED_135_DEG");
    m_keys[L_FACE_GREEN_135_DEG] = string("L_FACE_GREEN_135_DEG");
    m_keys[L_FACE_BLUE_135_DEG] = string("L_FACE_BLUE_135_DEG");
    m_keys[L_FACE_RED_180_DEG] = string("L_FACE_RED_180_DEG");
    m_keys[L_FACE_GREEN_180_DEG] = string("L_FACE_GREEN_180_DEG");
    m_keys[L_FACE_BLUE_180_DEG] = string("L_FACE_BLUE_180_DEG");
    m_keys[L_FACE_RED_225_DEG] = string("L_FACE_RED_225_DEG");
    m_keys[L_FACE_GREEN_225_DEG] = string("L_FACE_GREEN_225_DEG");
    m_keys[L_FACE_BLUE_225_DEG] = string("L_FACE_BLUE_225_DEG");
    m_keys[L_FACE_RED_270_DEG] = string("L_FACE_RED_270_DEG");
    m_keys[L_FACE_GREEN_270_DEG] = string("L_FACE_GREEN_270_DEG");
    m_keys[L_FACE_BLUE_270_DEG] = string("L_FACE_BLUE_270_DEG");
    m_keys[L_FACE_RED_315_DEG] = string("L_FACE_RED_315_DEG");
    m_keys[L_FACE_GREEN_315_DEG] = string("L_FACE_GREEN_315_DEG");
    m_keys[L_FACE_BLUE_315_DEG] = string("L_FACE_BLUE_315_DEG");
    m_keys[L_FACE_RED_45_DEG] = string("L_FACE_RED_45_DEG");
    m_keys[L_FACE_GREEN_45_DEG] = string("L_FACE_GREEN_45_DEG");
    m_keys[L_FACE_BLUE_45_DEG] = string("L_FACE_BLUE_45_DEG");
    m_keys[L_FACE_RED_90_DEG] = string("L_FACE_RED_90_DEG");
    m_keys[L_FACE_GREEN_90_DEG] = string("L_FACE_GREEN_90_DEG");
    m_keys[L_FACE_BLUE_90_DEG] = string("L_FACE_BLUE_90_DEG");
    m_keys[R_FACE_RED_0_DEG] = string("R_FACE_RED_0_DEG");
    m_keys[R_FACE_GREEN_0_DEG] = string("R_FACE_GREEN_0_DEG");
    m_keys[R_FACE_BLUE_0_DEG] = string("R_FACE_BLUE_0_DEG");
    m_keys[R_FACE_RED_135_DEG] = string("R_FACE_RED_135_DEG");
    m_keys[R_FACE_GREEN_135_DEG] = string("R_FACE_GREEN_135_DEG");
    m_keys[R_FACE_BLUE_135_DEG] = string("R_FACE_BLUE_135_DEG");
    m_keys[R_FACE_RED_180_DEG] = string("R_FACE_RED_180_DEG");
    m_keys[R_FACE_GREEN_180_DEG] = string("R_FACE_GREEN_180_DEG");
    m_keys[R_FACE_BLUE_180_DEG] = string("R_FACE_BLUE_180_DEG");
    m_keys[R_FACE_RED_225_DEG] = string("R_FACE_RED_225_DEG");
    m_keys[R_FACE_GREEN_225_DEG] = string("R_FACE_GREEN_225_DEG");
    m_keys[R_FACE_BLUE_225_DEG] = string("R_FACE_BLUE_225_DEG");
    m_keys[R_FACE_RED_270_DEG] = string("R_FACE_RED_270_DEG");
    m_keys[R_FACE_GREEN_270_DEG] = string("R_FACE_GREEN_270_DEG");
    m_keys[R_FACE_BLUE_270_DEG] = string("R_FACE_BLUE_270_DEG");
    m_keys[R_FACE_RED_315_DEG] = string("R_FACE_RED_315_DEG");
    m_keys[R_FACE_GREEN_315_DEG] = string("R_FACE_GREEN_315_DEG");
    m_keys[R_FACE_BLUE_315_DEG] = string("R_FACE_BLUE_315_DEG");
    m_keys[R_FACE_RED_45_DEG] = string("R_FACE_RED_45_DEG");
    m_keys[R_FACE_GREEN_45_DEG] = string("R_FACE_GREEN_45_DEG");
    m_keys[R_FACE_BLUE_45_DEG] = string("R_FACE_BLUE_45_DEG");
    m_keys[R_FACE_RED_90_DEG] = string("R_FACE_RED_90_DEG");
    m_keys[R_FACE_GREEN_90_DEG] = string("R_FACE_GREEN_90_DEG");
    m_keys[R_FACE_BLUE_90_DEG] = string("R_FACE_BLUE_90_DEG");
    m_keys[L_HEAD_FRONT_0] = string("L_HEAD_FRONT_0");
    m_keys[L_HEAD_FRONT_1] = string("L_HEAD_FRONT_1");
    m_keys[R_HEAD_FRONT_0] = string("R_HEAD_FRONT_0");
    m_keys[R_HEAD_FRONT_1] = string("R_HEAD_FRONT_1");
    m_keys[L_HEAD_MIDDLE_0] = string("L_HEAD_MIDDLE_0");
    m_keys[R_HEAD_MIDDLE_0] = string("R_HEAD_MIDDLE_0");
    m_keys[L_HEAD_REAR_0] = string("L_HEAD_REAR_0");
    m_keys[L_HEAD_REAR_1] = string("L_HEAD_REAR_1");
    m_keys[L_HEAD_REAR_2] = string("L_HEAD_REAR_2");
    m_keys[R_HEAD_REAR_0] = string("R_HEAD_REAR_0");
    m_keys[R_HEAD_REAR_1] = string("R_HEAD_REAR_1");
    m_keys[R_HEAD_REAR_2] = string("R_HEAD_REAR_2");
    m_keys[L_FOOT_RED] = string("L_FOOT_RED");
    m_keys[L_FOOT_GREEN] = string("L_FOOT_GREEN");
    m_keys[L_FOOT_BLUE] = string("L_FOOT_BLUE");
    m_keys[R_FOOT_RED] = string("R_FOOT_RED");
    m_keys[R_FOOT_GREEN] = string("R_FOOT_GREEN");
    m_keys[R_FOOT_BLUE] = string("R_FOOT_BLUE");

    m_leds_list[CHEST_BOARD_BLUE] = string("Device/SubDeviceList/ChestBoard/Led/Blue/Actuator/Value");
    m_leds_list[CHEST_BOARD_GREEN] = string("Device/SubDeviceList/ChestBoard/Led/Green/Actuator/Value");
    m_leds_list[CHEST_BOARD_RED] = string("Device/SubDeviceList/ChestBoard/Led/Red/Actuator/Value");
    m_leds_list[L_EAR_0_DEG] = string("Device/SubDeviceList/Ears/Led/Left/0Deg/Actuator/Value");
    m_leds_list[L_EAR_108_DEG] = string("Device/SubDeviceList/Ears/Led/Left/108Deg/Actuator/Value");
    m_leds_list[L_EAR_144_DEG] = string("Device/SubDeviceList/Ears/Led/Left/144Deg/Actuator/Value");
    m_leds_list[L_EAR_180_DEG] = string("Device/SubDeviceList/Ears/Led/Left/180Deg/Actuator/Value");
    m_leds_list[L_EAR_216_DEG] = string("Device/SubDeviceList/Ears/Led/Left/216Deg/Actuator/Value");
    m_leds_list[L_EAR_252_DEG] = string("Device/SubDeviceList/Ears/Led/Left/252Deg/Actuator/Value");
    m_leds_list[L_EAR_288_DEG] = string("Device/SubDeviceList/Ears/Led/Left/288Deg/Actuator/Value");
    m_leds_list[L_EAR_324_DEG] = string("Device/SubDeviceList/Ears/Led/Left/324Deg/Actuator/Value");
    m_leds_list[L_EAR_36_DEG] = string("Device/SubDeviceList/Ears/Led/Left/36Deg/Actuator/Value");
    m_leds_list[L_EAR_72_DEG] = string("Device/SubDeviceList/Ears/Led/Left/72Deg/Actuator/Value");
    m_leds_list[R_EAR_0_DEG] = string("Device/SubDeviceList/Ears/Led/Right/0Deg/Actuator/Value");
    m_leds_list[R_EAR_108_DEG] = string("Device/SubDeviceList/Ears/Led/Right/108Deg/Actuator/Value");
    m_leds_list[R_EAR_144_DEG] = string("Device/SubDeviceList/Ears/Led/Right/144Deg/Actuator/Value");
    m_leds_list[R_EAR_180_DEG] = string("Device/SubDeviceList/Ears/Led/Right/180Deg/Actuator/Value");
    m_leds_list[R_EAR_216_DEG] = string("Device/SubDeviceList/Ears/Led/Right/216Deg/Actuator/Value");
    m_leds_list[R_EAR_252_DEG] = string("Device/SubDeviceList/Ears/Led/Right/252Deg/Actuator/Value");
    m_leds_list[R_EAR_288_DEG] = string("Device/SubDeviceList/Ears/Led/Right/288Deg/Actuator/Value");
    m_leds_list[R_EAR_324_DEG] = string("Device/SubDeviceList/Ears/Led/Right/324Deg/Actuator/Value");
    m_leds_list[R_EAR_36_DEG] = string("Device/SubDeviceList/Ears/Led/Right/36Deg/Actuator/Value");
    m_leds_list[R_EAR_72_DEG] = string("Device/SubDeviceList/Ears/Led/Right/72Deg/Actuator/Value");
    m_leds_list[L_FACE_BLUE_0_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/0Deg/Actuator/Value");
    m_leds_list[L_FACE_BLUE_135_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/135Deg/Actuator/Value");
    m_leds_list[L_FACE_BLUE_180_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/180Deg/Actuator/Value");
    m_leds_list[L_FACE_BLUE_225_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/225Deg/Actuator/Value");
    m_leds_list[L_FACE_BLUE_270_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/270Deg/Actuator/Value");
    m_leds_list[L_FACE_BLUE_315_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/315Deg/Actuator/Value");
    m_leds_list[L_FACE_BLUE_45_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/45Deg/Actuator/Value");
    m_leds_list[L_FACE_BLUE_90_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/90Deg/Actuator/Value");
    m_leds_list[R_FACE_BLUE_0_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/0Deg/Actuator/Value");
    m_leds_list[R_FACE_BLUE_135_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/135Deg/Actuator/Value");
    m_leds_list[R_FACE_BLUE_180_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/180Deg/Actuator/Value");
    m_leds_list[R_FACE_BLUE_225_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/225Deg/Actuator/Value");
    m_leds_list[R_FACE_BLUE_270_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/270Deg/Actuator/Value");
    m_leds_list[R_FACE_BLUE_315_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/315Deg/Actuator/Value");
    m_leds_list[R_FACE_BLUE_45_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/45Deg/Actuator/Value");
    m_leds_list[R_FACE_BLUE_90_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/90Deg/Actuator/Value");
    m_leds_list[L_FACE_GREEN_0_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/0Deg/Actuator/Value");
    m_leds_list[L_FACE_GREEN_135_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/135Deg/Actuator/Value");
    m_leds_list[L_FACE_GREEN_180_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/180Deg/Actuator/Value");
    m_leds_list[L_FACE_GREEN_225_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/225Deg/Actuator/Value");
    m_leds_list[L_FACE_GREEN_270_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/270Deg/Actuator/Value");
    m_leds_list[L_FACE_GREEN_315_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/315Deg/Actuator/Value");
    m_leds_list[L_FACE_GREEN_45_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/45Deg/Actuator/Value");
    m_leds_list[L_FACE_GREEN_90_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/90Deg/Actuator/Value");
    m_leds_list[R_FACE_GREEN_0_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/0Deg/Actuator/Value");
    m_leds_list[R_FACE_GREEN_135_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/135Deg/Actuator/Value");
    m_leds_list[R_FACE_GREEN_180_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/180Deg/Actuator/Value");
    m_leds_list[R_FACE_GREEN_225_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/225Deg/Actuator/Value");
    m_leds_list[R_FACE_GREEN_270_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/270Deg/Actuator/Value");
    m_leds_list[R_FACE_GREEN_315_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/315Deg/Actuator/Value");
    m_leds_list[R_FACE_GREEN_45_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/45Deg/Actuator/Value");
    m_leds_list[R_FACE_GREEN_90_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/90Deg/Actuator/Value");
    m_leds_list[L_FACE_RED_0_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/0Deg/Actuator/Value");
    m_leds_list[L_FACE_RED_135_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/135Deg/Actuator/Value");
    m_leds_list[L_FACE_RED_180_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/180Deg/Actuator/Value");
    m_leds_list[L_FACE_RED_225_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/225Deg/Actuator/Value");
    m_leds_list[L_FACE_RED_270_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/270Deg/Actuator/Value");
    m_leds_list[L_FACE_RED_315_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/315Deg/Actuator/Value");
    m_leds_list[L_FACE_RED_45_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/45Deg/Actuator/Value");
    m_leds_list[L_FACE_RED_90_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/90Deg/Actuator/Value");
    m_leds_list[R_FACE_RED_0_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/0Deg/Actuator/Value");
    m_leds_list[R_FACE_RED_135_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/135Deg/Actuator/Value");
    m_leds_list[R_FACE_RED_180_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/180Deg/Actuator/Value");
    m_leds_list[R_FACE_RED_225_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/225Deg/Actuator/Value");
    m_leds_list[R_FACE_RED_270_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/270Deg/Actuator/Value");
    m_leds_list[R_FACE_RED_315_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/315Deg/Actuator/Value");
    m_leds_list[R_FACE_RED_45_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/45Deg/Actuator/Value");
    m_leds_list[R_FACE_RED_90_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/90Deg/Actuator/Value");
    m_leds_list[L_HEAD_FRONT_0] = string("Device/SubDeviceList/Head/Led/Front/Left/0/Actuator/Value");
    m_leds_list[L_HEAD_FRONT_1] = string("Device/SubDeviceList/Head/Led/Front/Left/1/Actuator/Value");
    m_leds_list[R_HEAD_FRONT_0] = string("Device/SubDeviceList/Head/Led/Front/Right/0/Actuator/Value");
    m_leds_list[R_HEAD_FRONT_1] = string("Device/SubDeviceList/Head/Led/Front/Right/1/Actuator/Value");
    m_leds_list[L_HEAD_MIDDLE_0] = string("Device/SubDeviceList/Head/Led/Middle/Left/0/Actuator/Value");
    m_leds_list[R_HEAD_MIDDLE_0] = string("Device/SubDeviceList/Head/Led/Middle/Right/0/Actuator/Value");
    m_leds_list[L_HEAD_REAR_0] = string("Device/SubDeviceList/Head/Led/Rear/Left/0/Actuator/Value");
    m_leds_list[L_HEAD_REAR_1] = string("Device/SubDeviceList/Head/Led/Rear/Left/1/Actuator/Value");
    m_leds_list[L_HEAD_REAR_2] = string("Device/SubDeviceList/Head/Led/Rear/Left/2/Actuator/Value");
    m_leds_list[R_HEAD_REAR_0] = string("Device/SubDeviceList/Head/Led/Rear/Right/0/Actuator/Value");
    m_leds_list[R_HEAD_REAR_1] = string("Device/SubDeviceList/Head/Led/Rear/Right/1/Actuator/Value");
    m_leds_list[R_HEAD_REAR_2] = string("Device/SubDeviceList/Head/Led/Rear/Right/2/Actuator/Value");
    m_leds_list[L_FOOT_BLUE] = string("Device/SubDeviceList/LFoot/Led/Blue/Actuator/Value");
    m_leds_list[L_FOOT_GREEN] = string("Device/SubDeviceList/LFoot/Led/Green/Actuator/Value");
    m_leds_list[L_FOOT_RED] = string("Device/SubDeviceList/LFoot/Led/Red/Actuator/Value");
    m_leds_list[R_FOOT_BLUE] = string("Device/SubDeviceList/RFoot/Led/Blue/Actuator/Value");
    m_leds_list[R_FOOT_GREEN] = string("Device/SubDeviceList/RFoot/Led/Green/Actuator/Value");
    m_leds_list[R_FOOT_RED] = string("Device/SubDeviceList/RFoot/Led/Red/Actuator/Value");
    // Init map of the keys
    this->initKeysMap(m_leds_map, m_keys, m_leds_list);
    // Prepare command for DCM rpoxy
    m_cmd.arraySetSize(3);
    m_cmd[1] = "ClearAll";
    m_cmd[2].arraySetSize(1);
    m_cmd[2][0].arraySetSize(2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string> &LEDs::getKeys() const {
    return m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool LEDs::setBrightness(const vector<string> &keys, const vector<double> &values) {
    // TODO It's very slow. We need do something with it.
    // Compare vector lengths
    if (keys.size() != values.size()) return false;
    // Send commands
    lock_guard<mutex> guard(m_synch);
    m_cmd[2][0][1] = m_dcm->getTime(0);
    for (int i = 0; i < keys.size(); ++i) {
        // Update cmd and sent values to DCM
        m_cmd[0] = m_leds_map[keys[i]];
        m_cmd[2][0][0] = values[i];
        m_dcm->set(m_cmd);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool LEDs::setBrightness(const vector<int> &keys, const vector<double> &values) {
    // TODO It's very slow. We need do something with it.
    // Compare vector lengths
    if (keys.size() != values.size()) return false;
    // Prepare iterators
    vector<int>::const_iterator keys_it = keys.begin();
    vector<double>::const_iterator values_it = values.begin();
    while (keys_it != keys.end()) {
        // Update cmd and sent values to DCM
        {
            lock_guard<mutex> guard(m_synch);
            m_cmd[0] = m_leds_list[*keys_it];
            m_cmd[2][0][0] = *values_it;
            m_cmd[2][0][1] = m_dcm->getTime(0);
            m_dcm->set(m_cmd);
        }
        // Update iterators
        ++keys_it;
        ++values_it;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > LEDs::getBrightness(const vector<int> &keys) {
    ALValue k;
    k.arraySetSize(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        try {
            k[i] = m_leds_list[keys[i]];
        } catch (out_of_range &e) {
            return make_shared<SensorData<double> >();
        }
    }
    ALValue data = m_mem->getListData(k);
    shared_ptr<SensorData<double> > res = make_shared<SensorData<double> >(keys.size(), m_dcm->getTime(0));
    for (int i = 0; i < keys.size(); ++i) res->data[i] = data[i];
    return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > LEDs::getBrightness(const vector<string> &keys) {
    ALValue k;
    k.arraySetSize(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        try {
            k[i] = m_leds_map[keys[i]];
        } catch (out_of_range &e) {
            return make_shared<SensorData<double> >();
        }
    }
    ALValue data = m_mem->getListData(k);
    shared_ptr<SensorData<double> > res = make_shared<SensorData<double> >(keys.size(), m_dcm->getTime(0));
    for (int i = 0; i < keys.size(); ++i) res->data[i] = data[i];
    return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > LEDs::getBrightness() {
    shared_ptr<SensorData<double> > res = make_shared<SensorData<double> >(LEDS_COUNT, m_dcm->getTime(0));
    ALValue data = m_mem->getListData(m_leds_list);
    for (int i = 0; i < LEDS_COUNT; ++i) res->data[i] = data[i];
    return res;
}
