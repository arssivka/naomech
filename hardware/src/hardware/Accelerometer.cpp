//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Accelerometer.h>

//
// Created by arssivka on 11/23/15.
//

using namespace std;
using namespace AL;
using namespace boost;
using namespace rd;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Accelerometer::Accelerometer(boost::shared_ptr<AL::ALBroker> broker)
        : keys(SENSOR_COUNT), mem(make_shared<ALMemoryProxy>(broker)), dcm(make_shared<DCMProxy>(broker)) {
    this->keys[X] = string("X");
    this->keys[Y] = string("Y");
    this->keys[Z] = string("Z");

    this->sensors.arraySetSize(SENSOR_COUNT);
    this->sensors[X] = string("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
    this->sensors[Y] = string("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
    this->sensors[Z] = string("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<string> &Accelerometer::getKeys() {
    return this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<SensorData<double> > Accelerometer::getAcceleration() {
    ALValue data = this->mem->getListData(this->sensors);
    shared_ptr<SensorData<double> > res = make_shared<SensorData<double> >(SENSOR_COUNT, this->dcm->getTime(0));
    for (unsigned int i = 0; i < SENSOR_COUNT; ++i) res->data[i] = data[i];
    return res;
}
