//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_ACCELEROMETER_H
#define NAOMECH_ACCELEROMETER_H

/*!        \defgroup accelerometer Accelerometer
           \ingroup hardware
 */

#include <alcommon/albroker.h>
#include <rd/hardware/SensorData.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "TypeDefinition.h"

namespace rd {
///@{
/*!
       \brief Class for recieving the accelerometer data

           This class allows to recieve
           the data from the nao accelerometer.
     */
    class Accelerometer : boost::noncopyable {
    public:

        /*!
           \brief The Key enumeration representing the keys, nedded for recieving data form the accelerometer
         */
        enum Key {
            X, // What this is kind of pokemon?!
            Y,
            Z,
            SENSOR_COUNT
        };

        /*!
           \brief Constructor for creating the Accelerometer class
           \param broker Broker that allows to connect to the Nao's modules.
         */
        Accelerometer(boost::shared_ptr<AL::ALBroker> broker);

        /*!
           \brief returns the keys of the accelerometer.
           \return vector of strings representing the keys of the accelerometer
         */
        const StringKeyVector &getKeys();

        /*!
           \brief Returns the acceleration data
           \return shared pointer to the  SensorData
         */
        SensorData<double>::Ptr getAcceleration();
///@}

    private:
        StringKeyVector m_keys;
        boost::shared_ptr<AL::ALMemoryProxy> m_mem;
        boost::shared_ptr<AL::DCMProxy> m_dcm;
        AL::ALValue m_sensors;
    };
}

#endif //NAOMECH_ACCELEROMETER_H
