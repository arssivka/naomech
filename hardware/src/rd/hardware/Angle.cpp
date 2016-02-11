//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Angle.h>

//
// Created by arssivka on 11/23/15.
//

using namespace std;
using namespace AL;
using namespace boost;
using namespace rd;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Angle::Angle(boost::shared_ptr<AL::ALBroker> broker)
        : m_keys(SENSOR_COUNT), m_mem(make_shared<ALMemoryProxy>(broker)), m_dcm(make_shared<DCMProxy>(broker)) {
    m_keys[X] = string("X");
    m_keys[Y] = string("Y");
    m_keys[Z] = string("Z");

    m_sensors.arraySetSize(SENSOR_COUNT);
    m_sensors[X] = string("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
    m_sensors[Y] = string("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
    m_sensors[Z] = string("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string>& Angle::getKeys() {
    return m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > Angle::getAngle() {
    ALValue data = m_mem->getListData(m_sensors);
    shared_ptr<SensorData<double> > res = make_shared<SensorData<double> >(SENSOR_COUNT, m_dcm->getTime(0));
    for (unsigned int i = 0; i < SENSOR_COUNT; ++i) res->data[i] = data[i];
    return res;
}
