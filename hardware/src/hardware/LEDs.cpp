//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/LEDs.h>


using namespace rd;
using namespace std;
using namespace boost;
using namespace AL;

LEDs::LEDs(shared_ptr<ALBroker> broker)
        : keys(LEDS_COUNT), leds_list(LEDS_COUNT) {
    this->keys[CHEST_BOARD_RED] = string("CHEST_BOARD_RED");
    this->keys[CHEST_BOARD_GREEN] = string("CHEST_BOARD_GREEN");
    this->keys[CHEST_BOARD_BLUE] = string("CHEST_BOARD_BLUE");
    this->keys[L_EAR_0_DEG] = string("L_EAR_0_DEG");
    this->keys[L_EAR_108_DEG] = string("L_EAR_108_DEG");
    this->keys[L_EAR_144_DEG] = string("L_EAR_144_DEG");
    this->keys[L_EAR_180_DEG] = string("L_EAR_180_DEG");
    this->keys[L_EAR_216_DEG] = string("L_EAR_216_DEG");
    this->keys[L_EAR_252_DEG] = string("L_EAR_252_DEG");
    this->keys[L_EAR_288_DEG] = string("L_EAR_288_DEG");
    this->keys[L_EAR_324_DEG] = string("L_EAR_324_DEG");
    this->keys[L_EAR_36_DEG] = string("L_EAR_36_DEG");
    this->keys[L_EAR_72_DEG] = string("L_EAR_72_DEG");
    this->keys[R_EAR_0_DEG] = string("R_EAR_0_DEG");
    this->keys[R_EAR_108_DEG] = string("R_EAR_108_DEG");
    this->keys[R_EAR_144_DEG] = string("R_EAR_144_DEG");
    this->keys[R_EAR_180_DEG] = string("R_EAR_180_DEG");
    this->keys[R_EAR_216_DEG] = string("R_EAR_216_DEG");
    this->keys[R_EAR_252_DEG] = string("R_EAR_252_DEG");
    this->keys[R_EAR_288_DEG] = string("R_EAR_288_DEG");
    this->keys[R_EAR_324_DEG] = string("R_EAR_324_DEG");
    this->keys[R_EAR_36_DEG] = string("R_EAR_36_DEG");
    this->keys[R_EAR_72_DEG] = string("R_EAR_72_DEG");
    this->keys[L_FACE_RED_0_DEG] = string("L_FACE_RED_0_DEG");
    this->keys[L_FACE_GREEN_0_DEG] = string("L_FACE_GREEN_0_DEG");
    this->keys[L_FACE_BLUE_0_DEG] = string("L_FACE_BLUE_0_DEG");
    this->keys[L_FACE_RED_135_DEG] = string("L_FACE_RED_135_DEG");
    this->keys[L_FACE_GREEN_135_DEG] = string("L_FACE_GREEN_135_DEG");
    this->keys[L_FACE_BLUE_135_DEG] = string("L_FACE_BLUE_135_DEG");
    this->keys[L_FACE_RED_180_DEG] = string("L_FACE_RED_180_DEG");
    this->keys[L_FACE_GREEN_180_DEG] = string("L_FACE_GREEN_180_DEG");
    this->keys[L_FACE_BLUE_180_DEG] = string("L_FACE_BLUE_180_DEG");
    this->keys[L_FACE_RED_225_DEG] = string("L_FACE_RED_225_DEG");
    this->keys[L_FACE_GREEN_225_DEG] = string("L_FACE_GREEN_225_DEG");
    this->keys[L_FACE_BLUE_225_DEG] = string("L_FACE_BLUE_225_DEG");
    this->keys[L_FACE_RED_270_DEG] = string("L_FACE_RED_270_DEG");
    this->keys[L_FACE_GREEN_270_DEG] = string("L_FACE_GREEN_270_DEG");
    this->keys[L_FACE_BLUE_270_DEG] = string("L_FACE_BLUE_270_DEG");
    this->keys[L_FACE_RED_315_DEG] = string("L_FACE_RED_315_DEG");
    this->keys[L_FACE_GREEN_315_DEG] = string("L_FACE_GREEN_315_DEG");
    this->keys[L_FACE_BLUE_315_DEG] = string("L_FACE_BLUE_315_DEG");
    this->keys[L_FACE_RED_45_DEG] = string("L_FACE_RED_45_DEG");
    this->keys[L_FACE_GREEN_45_DEG] = string("L_FACE_GREEN_45_DEG");
    this->keys[L_FACE_BLUE_45_DEG] = string("L_FACE_BLUE_45_DEG");
    this->keys[L_FACE_RED_90_DEG] = string("L_FACE_RED_90_DEG");
    this->keys[L_FACE_GREEN_90_DEG] = string("L_FACE_GREEN_90_DEG");
    this->keys[L_FACE_BLUE_90_DEG] = string("L_FACE_BLUE_90_DEG");
    this->keys[R_FACE_RED_0_DEG] = string("R_FACE_RED_0_DEG");
    this->keys[R_FACE_GREEN_0_DEG] = string("R_FACE_GREEN_0_DEG");
    this->keys[R_FACE_BLUE_0_DEG] = string("R_FACE_BLUE_0_DEG");
    this->keys[R_FACE_RED_135_DEG] = string("R_FACE_RED_135_DEG");
    this->keys[R_FACE_GREEN_135_DEG] = string("R_FACE_GREEN_135_DEG");
    this->keys[R_FACE_BLUE_135_DEG] = string("R_FACE_BLUE_135_DEG");
    this->keys[R_FACE_RED_180_DEG] = string("R_FACE_RED_180_DEG");
    this->keys[R_FACE_GREEN_180_DEG] = string("R_FACE_GREEN_180_DEG");
    this->keys[R_FACE_BLUE_180_DEG] = string("R_FACE_BLUE_180_DEG");
    this->keys[R_FACE_RED_225_DEG] = string("R_FACE_RED_225_DEG");
    this->keys[R_FACE_GREEN_225_DEG] = string("R_FACE_GREEN_225_DEG");
    this->keys[R_FACE_BLUE_225_DEG] = string("R_FACE_BLUE_225_DEG");
    this->keys[R_FACE_RED_270_DEG] = string("R_FACE_RED_270_DEG");
    this->keys[R_FACE_GREEN_270_DEG] = string("R_FACE_GREEN_270_DEG");
    this->keys[R_FACE_BLUE_270_DEG] = string("R_FACE_BLUE_270_DEG");
    this->keys[R_FACE_RED_315_DEG] = string("R_FACE_RED_315_DEG");
    this->keys[R_FACE_GREEN_315_DEG] = string("R_FACE_GREEN_315_DEG");
    this->keys[R_FACE_BLUE_315_DEG] = string("R_FACE_BLUE_315_DEG");
    this->keys[R_FACE_RED_45_DEG] = string("R_FACE_RED_45_DEG");
    this->keys[R_FACE_GREEN_45_DEG] = string("R_FACE_GREEN_45_DEG");
    this->keys[R_FACE_BLUE_45_DEG] = string("R_FACE_BLUE_45_DEG");
    this->keys[R_FACE_RED_90_DEG] = string("R_FACE_RED_90_DEG");
    this->keys[R_FACE_GREEN_90_DEG] = string("R_FACE_GREEN_90_DEG");
    this->keys[R_FACE_BLUE_90_DEG] = string("R_FACE_BLUE_90_DEG");
    this->keys[L_HEAD_FRONT_0] = string("L_HEAD_FRONT_0");
    this->keys[L_HEAD_FRONT_1] = string("L_HEAD_FRONT_1");
    this->keys[R_HEAD_FRONT_0] = string("R_HEAD_FRONT_0");
    this->keys[R_HEAD_FRONT_1] = string("R_HEAD_FRONT_1");
    this->keys[L_HEAD_MIDDLE_0] = string("L_HEAD_MIDDLE_0");
    this->keys[R_HEAD_MIDDLE_0] = string("R_HEAD_MIDDLE_0");
    this->keys[L_HEAD_REAR_0] = string("L_HEAD_REAR_0");
    this->keys[L_HEAD_REAR_1] = string("L_HEAD_REAR_1");
    this->keys[L_HEAD_REAR_2] = string("L_HEAD_REAR_2");
    this->keys[R_HEAD_REAR_0] = string("R_HEAD_REAR_0");
    this->keys[R_HEAD_REAR_1] = string("R_HEAD_REAR_1");
    this->keys[R_HEAD_REAR_2] = string("R_HEAD_REAR_2");
    this->keys[L_FOOT_RED] = string("L_FOOT_RED");
    this->keys[L_FOOT_GREEN] = string("L_FOOT_GREEN");
    this->keys[L_FOOT_BLUE] = string("L_FOOT_BLUE");
    this->keys[R_FOOT_RED] = string("R_FOOT_RED");
    this->keys[R_FOOT_GREEN] = string("R_FOOT_GREEN");
    this->keys[R_FOOT_BLUE] = string("R_FOOT_BLUE");

    this->leds_list[CHEST_BOARD_BLUE] = string("Device/SubDeviceList/ChestBoard/Led/Blue/Actuator/Value");
    this->leds_list[CHEST_BOARD_GREEN] = string("Device/SubDeviceList/ChestBoard/Led/Green/Actuator/Value");
    this->leds_list[CHEST_BOARD_RED] = string("Device/SubDeviceList/ChestBoard/Led/Red/Actuator/Value");
    this->leds_list[L_EAR_0_DEG] = string("Device/SubDeviceList/Ears/Led/Left/0Deg/Actuator/Value");
    this->leds_list[L_EAR_108_DEG] = string("Device/SubDeviceList/Ears/Led/Left/108Deg/Actuator/Value");
    this->leds_list[L_EAR_144_DEG] = string("Device/SubDeviceList/Ears/Led/Left/144Deg/Actuator/Value");
    this->leds_list[L_EAR_180_DEG] = string("Device/SubDeviceList/Ears/Led/Left/180Deg/Actuator/Value");
    this->leds_list[L_EAR_216_DEG] = string("Device/SubDeviceList/Ears/Led/Left/216Deg/Actuator/Value");
    this->leds_list[L_EAR_252_DEG] = string("Device/SubDeviceList/Ears/Led/Left/252Deg/Actuator/Value");
    this->leds_list[L_EAR_288_DEG] = string("Device/SubDeviceList/Ears/Led/Left/288Deg/Actuator/Value");
    this->leds_list[L_EAR_324_DEG] = string("Device/SubDeviceList/Ears/Led/Left/324Deg/Actuator/Value");
    this->leds_list[L_EAR_36_DEG] = string("Device/SubDeviceList/Ears/Led/Left/36Deg/Actuator/Value");
    this->leds_list[L_EAR_72_DEG] = string("Device/SubDeviceList/Ears/Led/Left/72Deg/Actuator/Value");
    this->leds_list[R_EAR_0_DEG] = string("Device/SubDeviceList/Ears/Led/Right/0Deg/Actuator/Value");
    this->leds_list[R_EAR_108_DEG] = string("Device/SubDeviceList/Ears/Led/Right/108Deg/Actuator/Value");
    this->leds_list[R_EAR_144_DEG] = string("Device/SubDeviceList/Ears/Led/Right/144Deg/Actuator/Value");
    this->leds_list[R_EAR_180_DEG] = string("Device/SubDeviceList/Ears/Led/Right/180Deg/Actuator/Value");
    this->leds_list[R_EAR_216_DEG] = string("Device/SubDeviceList/Ears/Led/Right/216Deg/Actuator/Value");
    this->leds_list[R_EAR_252_DEG] = string("Device/SubDeviceList/Ears/Led/Right/252Deg/Actuator/Value");
    this->leds_list[R_EAR_288_DEG] = string("Device/SubDeviceList/Ears/Led/Right/288Deg/Actuator/Value");
    this->leds_list[R_EAR_324_DEG] = string("Device/SubDeviceList/Ears/Led/Right/324Deg/Actuator/Value");
    this->leds_list[R_EAR_36_DEG] = string("Device/SubDeviceList/Ears/Led/Right/36Deg/Actuator/Value");
    this->leds_list[R_EAR_72_DEG] = string("Device/SubDeviceList/Ears/Led/Right/72Deg/Actuator/Value");
    this->leds_list[L_FACE_BLUE_0_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/0Deg/Actuator/Value");
    this->leds_list[L_FACE_BLUE_135_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/135Deg/Actuator/Value");
    this->leds_list[L_FACE_BLUE_180_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/180Deg/Actuator/Value");
    this->leds_list[L_FACE_BLUE_225_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/225Deg/Actuator/Value");
    this->leds_list[L_FACE_BLUE_270_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/270Deg/Actuator/Value");
    this->leds_list[L_FACE_BLUE_315_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/315Deg/Actuator/Value");
    this->leds_list[L_FACE_BLUE_45_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/45Deg/Actuator/Value");
    this->leds_list[L_FACE_BLUE_90_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Left/90Deg/Actuator/Value");
    this->leds_list[R_FACE_BLUE_0_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/0Deg/Actuator/Value");
    this->leds_list[R_FACE_BLUE_135_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/135Deg/Actuator/Value");
    this->leds_list[R_FACE_BLUE_180_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/180Deg/Actuator/Value");
    this->leds_list[R_FACE_BLUE_225_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/225Deg/Actuator/Value");
    this->leds_list[R_FACE_BLUE_270_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/270Deg/Actuator/Value");
    this->leds_list[R_FACE_BLUE_315_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/315Deg/Actuator/Value");
    this->leds_list[R_FACE_BLUE_45_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/45Deg/Actuator/Value");
    this->leds_list[R_FACE_BLUE_90_DEG] = string("Device/SubDeviceList/Face/Led/Blue/Right/90Deg/Actuator/Value");
    this->leds_list[L_FACE_GREEN_0_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/0Deg/Actuator/Value");
    this->leds_list[L_FACE_GREEN_135_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/135Deg/Actuator/Value");
    this->leds_list[L_FACE_GREEN_180_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/180Deg/Actuator/Value");
    this->leds_list[L_FACE_GREEN_225_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/225Deg/Actuator/Value");
    this->leds_list[L_FACE_GREEN_270_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/270Deg/Actuator/Value");
    this->leds_list[L_FACE_GREEN_315_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/315Deg/Actuator/Value");
    this->leds_list[L_FACE_GREEN_45_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/45Deg/Actuator/Value");
    this->leds_list[L_FACE_GREEN_90_DEG] = string("Device/SubDeviceList/Face/Led/Green/Left/90Deg/Actuator/Value");
    this->leds_list[R_FACE_GREEN_0_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/0Deg/Actuator/Value");
    this->leds_list[R_FACE_GREEN_135_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/135Deg/Actuator/Value");
    this->leds_list[R_FACE_GREEN_180_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/180Deg/Actuator/Value");
    this->leds_list[R_FACE_GREEN_225_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/225Deg/Actuator/Value");
    this->leds_list[R_FACE_GREEN_270_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/270Deg/Actuator/Value");
    this->leds_list[R_FACE_GREEN_315_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/315Deg/Actuator/Value");
    this->leds_list[R_FACE_GREEN_45_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/45Deg/Actuator/Value");
    this->leds_list[R_FACE_GREEN_90_DEG] = string("Device/SubDeviceList/Face/Led/Green/Right/90Deg/Actuator/Value");
    this->leds_list[L_FACE_RED_0_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/0Deg/Actuator/Value");
    this->leds_list[L_FACE_RED_135_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/135Deg/Actuator/Value");
    this->leds_list[L_FACE_RED_180_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/180Deg/Actuator/Value");
    this->leds_list[L_FACE_RED_225_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/225Deg/Actuator/Value");
    this->leds_list[L_FACE_RED_270_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/270Deg/Actuator/Value");
    this->leds_list[L_FACE_RED_315_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/315Deg/Actuator/Value");
    this->leds_list[L_FACE_RED_45_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/45Deg/Actuator/Value");
    this->leds_list[L_FACE_RED_90_DEG] = string("Device/SubDeviceList/Face/Led/Red/Left/90Deg/Actuator/Value");
    this->leds_list[R_FACE_RED_0_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/0Deg/Actuator/Value");
    this->leds_list[R_FACE_RED_135_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/135Deg/Actuator/Value");
    this->leds_list[R_FACE_RED_180_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/180Deg/Actuator/Value");
    this->leds_list[R_FACE_RED_225_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/225Deg/Actuator/Value");
    this->leds_list[R_FACE_RED_270_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/270Deg/Actuator/Value");
    this->leds_list[R_FACE_RED_315_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/315Deg/Actuator/Value");
    this->leds_list[R_FACE_RED_45_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/45Deg/Actuator/Value");
    this->leds_list[R_FACE_RED_90_DEG] = string("Device/SubDeviceList/Face/Led/Red/Right/90Deg/Actuator/Value");
    this->leds_list[L_HEAD_FRONT_0] = string("Device/SubDeviceList/Head/Led/Front/Left/0/Actuator/Value");
    this->leds_list[L_HEAD_FRONT_1] = string("Device/SubDeviceList/Head/Led/Front/Left/1/Actuator/Value");
    this->leds_list[R_HEAD_FRONT_0] = string("Device/SubDeviceList/Head/Led/Front/Right/0/Actuator/Value");
    this->leds_list[R_HEAD_FRONT_1] = string("Device/SubDeviceList/Head/Led/Front/Right/1/Actuator/Value");
    this->leds_list[L_HEAD_MIDDLE_0] = string("Device/SubDeviceList/Head/Led/Middle/Left/0/Actuator/Value");
    this->leds_list[R_HEAD_MIDDLE_0] = string("Device/SubDeviceList/Head/Led/Middle/Right/0/Actuator/Value");
    this->leds_list[L_HEAD_REAR_0] = string("Device/SubDeviceList/Head/Led/Rear/Left/0/Actuator/Value");
    this->leds_list[L_HEAD_REAR_1] = string("Device/SubDeviceList/Head/Led/Rear/Left/1/Actuator/Value");
    this->leds_list[L_HEAD_REAR_2] = string("Device/SubDeviceList/Head/Led/Rear/Left/2/Actuator/Value");
    this->leds_list[R_HEAD_REAR_0] = string("Device/SubDeviceList/Head/Led/Rear/Right/0/Actuator/Value");
    this->leds_list[R_HEAD_REAR_1] = string("Device/SubDeviceList/Head/Led/Rear/Right/1/Actuator/Value");
    this->leds_list[R_HEAD_REAR_2] = string("Device/SubDeviceList/Head/Led/Rear/Right/2/Actuator/Value");
    this->leds_list[L_FOOT_BLUE] = string("Device/SubDeviceList/LFoot/Led/Blue/Actuator/Value");
    this->leds_list[L_FOOT_GREEN] = string("Device/SubDeviceList/LFoot/Led/Green/Actuator/Value");
    this->leds_list[L_FOOT_RED] = string("Device/SubDeviceList/LFoot/Led/Red/Actuator/Value");
    this->leds_list[L_FOOT_BLUE] = string("Device/SubDeviceList/RFoot/Led/Blue/Actuator/Value");
    this->leds_list[L_FOOT_GREEN] = string("Device/SubDeviceList/RFoot/Led/Green/Actuator/Value");
    this->leds_list[L_FOOT_RED] = string("Device/SubDeviceList/RFoot/Led/Red/Actuator/Value");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string> &LEDs::getKeys() const {
    return this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool LEDs::setBrightness(const vector<string> &keys, const vector<double> &values) {
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool LEDs::setBrightness(const vector<int> &keys, const vector<double> &values) {
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > LEDs::getBrightness(const vector<int> &keys) {
    return shared_ptr<SensorData<double> >();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > LEDs::getBrightness(const vector<keys> &keys) {
    return shared_ptr<SensorData<double> >();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > LEDs::getBrightness() {
    return shared_ptr<SensorData<double> >();
}
