//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Gyro.h>

using namespace std;
using namespace AL;
using namespace boost;
using namespace rd;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Gyro::Gyro(shared_ptr<ALBroker> broker)
        : m_keys(SENSOR_COUNT), m_mem(make_shared<ALMemoryProxy>(broker)), m_dcm(make_shared<DCMProxy>(broker)) {
    m_keys[X] = string("X");
    m_keys[Y] = string("Y");
    m_keys[Z] = string("Z");

    m_sensors.arraySetSize(SENSOR_COUNT);
    m_sensors[X] = string("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
    m_sensors[Y] = string("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
    m_sensors[Z] = string("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string> &Gyro::getKeys() {
    return m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SensorData<double>::Ptr Gyro::getAngularVelocity() {
    ALValue data = m_mem->getListData(m_sensors);
    SensorData<double>::Ptr res = make_shared<SensorData<double> >(SENSOR_COUNT, m_dcm->getTime(0));
    for (unsigned int i = 0; i < SENSOR_COUNT; ++i) res->data[i] = data[i];
    return res;
}
