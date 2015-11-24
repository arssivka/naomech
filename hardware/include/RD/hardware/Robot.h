#ifndef NAOMECH_ROBOT_H
#define NAOMECH_ROBOT_H

#include <vector>
#include <RD/hardware/Accelerometer.h>
#include <RD/hardware/Gyro.h>
#include "LEDs.h"
#include <RD/hardware/>

namespace RD {


    class Robot {
    public:
        Robot();

    private:
        struct {
            std::vector<float> accelerometer;
            std::vector<float> gyro;
            std::vector<float> joints;
            std::vector<float> leds;
            // TODO Add cameras
        } data;

    };
}


#endif //NAOMECH_ROBOT_H
