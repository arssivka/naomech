#ifndef NAOMECH_ROBOT_H
#define NAOMECH_ROBOT_H

#include <vector>
#include <rd/hardware/Accelerometer.h>
#include <rd/hardware/Gyro.h>
#include <rd/hardware/Joints.h>
#include <rd/hardware/LEDs.h>


namespace rd {
    class Robot {
    public:
        Robot(std::string name, const std::string &ip, unsigned int port);

        boost::shared_ptr<Joints> getJoints();

        virtual ~Robot();

    private:
        boost::shared_ptr<AL::DCMProxy> dcm;
        boost::shared_ptr<AL::ALMemoryProxy> mem;

        boost::shared_ptr<Joints> joints;
    };
}


#endif //NAOMECH_ROBOT_H
