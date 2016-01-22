//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Robot.h>
#include <alcommon/albrokermanager.h>

using namespace AL;
using namespace boost;
using namespace std;
using namespace rd;

Robot::Robot(string name, const string &ip, unsigned int port) {
    const string broker_ip = string("0.0.0.0");
    const int broker_port = 54000;
    try {
        this->broker = ALBroker::createBroker(name, broker_ip, broker_port, ip, port, 0);
    } catch (...) {
        cerr << "Fail to connect broker to: " << ip << ":" << port << std::endl;
        ALBrokerManager::getInstance()->killAllBroker();
        ALBrokerManager::kill();
        // TODO Raise exception
    }

    ALBrokerManager::setInstance(this->broker->fBrokerManager.lock());
    ALBrokerManager::getInstance()->addBroker(this->broker);
    this->dcm = make_shared<DCMProxy>(this->broker);
    this->mem = make_shared<ALMemoryProxy>(this->broker);

    this->joints = make_shared<Joints>(broker);
    this->leds = make_shared<LEDs>(broker);
    this->gyro = make_shared<Gyro>(broker);
    this->accelerometer = make_shared<Accelerometer>(broker);

    this->top_camera = make_shared<Camera>("/dev/video0", 320, 240, true, broker);
    this->bot_camera = make_shared<Camera>("/dev/video1", 320, 240, true, broker);
    this->top_camera->setFPS(30);
    this->bot_camera->setFPS(30);
    this->top_camera->enableCamera();
    this->bot_camera->enableCamera();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Joints> Robot::getJoints() {
    return this->joints;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Camera> Robot::getTopCamera() {
    return this->top_camera;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Camera> Robot::getBotCamera() {
    return this->bot_camera;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<LEDs> Robot::getLEDs() {
    return this->leds;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<Gyro> Robot::getGyro() {
    return this->gyro;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<Accelerometer> Robot::getAccelerometer() {
    return this->accelerometer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Robot::~Robot() {
    AL::ALBrokerManager::getInstance()->killAllBroker();
    AL::ALBrokerManager::kill();
}
