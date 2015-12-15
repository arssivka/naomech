//
// Created by arssivka on 12/15/15.
//

#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>
#include "RD/HardwareAccessModule/HardwareAccessModule.h"

using namespace RD;

extern "C"
{
int _createModule(boost::shared_ptr<AL::ALBroker> pBroker) {
    // init broker with the main broker instance
    // from the parent executable
    AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(pBroker);
    // create module instances
    AL::ALModule::createModule<HardwareAccessModule>(pBroker, "rd/hardware");
    return 0;
}

ALCALL int _closeModule() {
    return 0;
}
} // extern "C"