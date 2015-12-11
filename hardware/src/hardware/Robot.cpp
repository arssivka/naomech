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

    this->joints = make_shared<Joints>(dcm, mem);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<rd::Joints> rd::Robot::getJoints() {
    return this->joints;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::Robot::~Robot() {
    AL::ALBrokerManager::getInstance()->killAllBroker();
    AL::ALBrokerManager::kill();
}
