//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Clock.h>

using namespace rd;
using namespace AL;
using namespace boost;


Clock::Clock(shared_ptr<ALBroker> broker) : dcm(make_shared<DCMProxy>(broker)) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int Clock::getTime(int offset) const {
    return this->dcm->getTime(0);
}
