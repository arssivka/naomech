//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_CLOCK_H
#define NAOMECH_CLOCK_H


#include <alproxies/dcmproxy.h>

namespace rd {
    class Clock {
    public:
        Clock(boost::shared_ptr<AL::DCMProxy> dcm);

        int getDCMTime(int offset = 0) const;

    private:
        boost::shared_ptr<AL::DCMProxy> dcm;
    };
}


#endif //NAOMECH_CLOCK_H
