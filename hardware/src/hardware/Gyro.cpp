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
        : keys(SENSOR_COUNT), mem(make_shared<ALMemoryProxy>(broker)), dcm(make_shared<DCMProxy>(broker)) {
    this->keys[REF] = string("REF");
    this->keys[X] = string("X");
    this->keys[Y] = string("Y");

    this->sensors.arraySetSize(SENSOR_COUNT);
    this->sensors[REF] = string("Device/SubDeviceList/InertialSensor/GyrRef/Sensor/Value");
    this->sensors[X] = string("Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value");
    this->sensors[Y] = string("Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string> &Gyro::getKeys() {
    return this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > Gyro::getAngularVelocity() {
    ALValue data = this->mem->getListData(this->sensors);
    shared_ptr<SensorData<double> > res = make_shared<SensorData<double> >(SENSOR_COUNT, this->dcm->getTime(0));
    for (unsigned int i = 0; i < SENSOR_COUNT; ++i) res->data[i] = data[i];
    return res;
}
