//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Robot.h>
#include <alcommon/almodule.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

using namespace AL;
using namespace boost;
using namespace std;

rd::Robot::Robot(string name, const string &ip, unsigned int port) {
    const string broker_ip = string("0.0.0.0");
    const int broker_port = 54000;
    shared_ptr<ALBroker> broker;
    try {
        broker = ALBroker::createBroker(
                name,
                broker_ip,
                broker_port,
                ip,
                port
        );
    } catch (...) {
        // TODO Log this shit!
        ALBrokerManager::getInstance()->killAllBroker();
        ALBrokerManager::kill();
        // TODO Raise exception
    }

    ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    ALBrokerManager::getInstance()->addBroker(broker);

    this->dcm = make_shared<DCMProxy>(broker);
    this->mem = make_shared<ALMemoryProxy>(broker);

    this->joints = make_shared<Joints>(mem, dcm);

    this->top_camera = shared_ptr<rd::Camera>(
                new rd::Camera("/dev/video0", 320, 240, true));
    this->bot_camera = shared_ptr<rd::Camera>(
                new rd::Camera("/dev/video1", 320, 240, true));
    this->top_camera->setFPS(25);
    this->bot_camera->setFPS(25);
    this->top_camera->startCapturing();
    this->bot_camera->startCapturing();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<rd::Joints> rd::Robot::getJoints() {
    return this->joints;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<rd::Camera> rd::Robot::getTopCamera() {
    return this->top_camera;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<rd::Camera> rd::Robot::getBotCamera() {
    return this->bot_camera;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::Robot::~Robot() {
    AL::ALBrokerManager::getInstance()->killAllBroker();
    AL::ALBrokerManager::kill();
}
