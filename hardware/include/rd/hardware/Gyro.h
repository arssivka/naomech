//
// Created by arssivka on 11/23/15.
//
#ifndef NAOMECH_GYRO_H
#define NAOMECH_GYRO_H

/*!        \defgroup leds LEDs
           \ingroup hardware
 */

#include <alcommon/albroker.h>
#include <rd/hardware/SensorData.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>

namespace rd {
///@{
/*!
       \brief Class for recieving the gyro data

           This class allows to recieve
           the data from the nao gyro.
     */
    class Gyro : boost::noncopyable {
    public:
        /*!
           \brief The Key enumeration representing the keys, needed for recieving data from the gyro
         */
        enum Key {
            X,
            Y,
            Z, // What this is kind of pokemon?! // What this is kind of the comment?!
            SENSOR_COUNT
        };

        /*!
           \brief Constructor for creating the Gyro class
           \param broker Broker that allows to connect to the Nao's modules.
         */
        Gyro(boost::shared_ptr<AL::ALBroker> broker);

        /*!
           \brief returns the keys of the gyro.
           \return vector of strings representing the keys of the gyro
         */
        const std::vector<std::string> &getKeys();

        /*!
           \brief Returns the angular velocity (The Gyro Data)
           \return shared pointer to the vector of SensorData
         */
        boost::shared_ptr<SensorData<double> > getAngularVelocity();
///@}
    private:
        std::vector<std::string> m_keys;
        boost::shared_ptr<AL::ALMemoryProxy> m_mem;
        boost::shared_ptr<AL::DCMProxy> m_dcm;
        AL::ALValue m_sensors;
    };
}

#endif //NAOMECH_GYRO_H
