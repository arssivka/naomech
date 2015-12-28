#ifndef NAOMECH_ROBOT_H
#define NAOMECH_ROBOT_H

/*!        \defgroup robot Robot
           \ingroup hardware
 */

#include <vector>
#include <rd/hardware/Accelerometer.h>
#include <rd/hardware/Gyro.h>
#include <rd/hardware/Joints.h>
#include <rd/hardware/LEDs.h>
#include <rd/hardware/Camera.h>


namespace rd {
    /*!
     * \brief The Robot class
     */
    class Robot {
    public:
        Robot(std::string name, const std::string &ip, unsigned int port);

        boost::shared_ptr<Joints> getJoints();

        boost::shared_ptr<LEDs> getLEDs();

        boost::shared_ptr<Camera> getBotCamera();

        boost::shared_ptr<Camera> getTopCamera();

        virtual ~Robot();

    private:
        boost::shared_ptr<AL::ALBroker> broker;
        boost::shared_ptr<AL::DCMProxy> dcm;
        boost::shared_ptr<AL::ALMemoryProxy> mem;

        boost::shared_ptr<Joints> joints;
        boost::shared_ptr<LEDs> leds;
        boost::shared_ptr<Camera> bot_camera;
        boost::shared_ptr<Camera> top_camera;
    };
}


#endif //NAOMECH_ROBOT_H
