//
// Created by arssivka on 11/23/15.
//

#include "RD/hardware/Clock.h"

using namespace RD;
using namespace AL;
using namespace boost;


Clock::Clock(shared_ptr<DCMProxy> dcm) : dcm(dcm) { }

////////////////////////////////////////////////////////////////////////////////


int Clock::operator()(int offset) const {
    return this->dcm->getTime(0);
}
