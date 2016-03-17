//
// Created by arssivka on 12/15/15.
//

#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>
#include <rd/naoqi/hwcontroller/HWController.h>

using namespace rd;
using namespace boost;
using namespace AL;

extern "C"
{
int _createModule(shared_ptr<ALBroker> pBroker) {
    // init broker with the main broker instance
    // from the parent executable
    ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    ALBrokerManager::getInstance()->addBroker(pBroker);
    // create module instances
    ALModule::createModule<HWController>(pBroker, "rd/hwcontroller");
    return 0;
}

int _closeModule() {
    return 0;
}
} // extern "C"
