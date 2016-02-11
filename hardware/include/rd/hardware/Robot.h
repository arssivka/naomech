#ifndef NAOMECH_ROBOT_H
#define NAOMECH_ROBOT_H

/*!        \defgroup robot Robot
           \ingroup hardware
 */

#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <rd/hardware/Accelerometer.h>
#include <rd/hardware/Gyro.h>
#include <rd/hardware/Joints.h>
#include <rd/hardware/LEDs.h>
#include <rd/hardware/Camera.h>
#include "Angle.h"


namespace rd {
///@{
    /*!
       \brief Class that represents hardware parts of the robot

       This class stores objects of all classes, that represents each hardware part of the robot:
       Camera, Joints, LEDs, Gyro, Accelerometer.
     */
    class Robot : boost::noncopyable {
    public:
        /*!
           \brief Constructor for creating the Robot object
           \param name Who's that pokemon?
           \param ip Nao robot ip address
           \param port Port number
           \param config_filename Path to JSON configuration file
         */
        Robot(const std::string& name, const std::string& ip, unsigned int port, const std::string& config_filename);

        /*!
           \brief Returns the Joints object
           \return Shared pointer to Joints object
         */
        boost::shared_ptr<Joints> getJoints();

        /*!
           \brief Returns the LEDs object
           \return Shared pointer to LEDs object
         */
        boost::shared_ptr<LEDs> getLEDs();

        /*!
           \brief Returns the Gyro object
           \return Shared pointer to Gyro object
         */
        boost::shared_ptr<Gyro> getGyro();

        /*!
           \brief Returns the Accelerometer object
           \return Shared pointer to Accelerometer object
         */
        boost::shared_ptr<Accelerometer> getAccelerometer();

        /*!
           \brief Returns the Angle object
           \return Shared pointer to Accelerometer object
         */
        boost::shared_ptr<Angle> getAngle();

        /*!
           \brief Returns Camera object representing bottom camera
           \return Shared pointer to Camera object
         */
        boost::shared_ptr<Camera> getBotCamera();

        /*!
           \brief Returns Camera object representing top camera
           \return Shared pointer to Camera object
         */
        boost::shared_ptr<Camera> getTopCamera();

        /*!
           \brief Returns Clock object
           \return Reference to Clock object
         */
        boost::shared_ptr<Clock> getClock();

        /*!
           \brief Returns configuration property tree
           \return Reference to property tree
         */
        boost::shared_ptr<boost::property_tree::ptree> getConfig();

        /*!
           \brief Destructor
         */
        ~Robot();
        ///@}

    private:
        boost::shared_ptr<boost::property_tree::ptree> m_config;

        boost::shared_ptr<AL::ALBroker> m_broker;

        boost::shared_ptr<Angle> m_angle;
        boost::shared_ptr<Clock> m_clock;
        boost::shared_ptr<Joints> m_joints;
        boost::shared_ptr<LEDs> m_leds;
        boost::shared_ptr<Gyro> m_gyro;
        boost::shared_ptr<Accelerometer> m_accelerometer;
        boost::shared_ptr<Camera> m_bot_camera;
        boost::shared_ptr<Camera> m_top_camera;
    };
}


#endif //NAOMECH_ROBOT_H
