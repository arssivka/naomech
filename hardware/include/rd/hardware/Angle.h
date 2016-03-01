//
// Created by arssivka on 2/11/16.
//

#ifndef NAOMECH_ANGLE_H
#define NAOMECH_ANGLE_H


#include <alcommon/albroker.h>
#include <rd/hardware/SensorData.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "TypeDefinition.h"

namespace rd {
///@{
/*!
       \brief Class for recieving the angle data

           This class allows to recieve
           the data from the nao angles O_O.
     */
    class Angle : boost::noncopyable {
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
           \brief Constructor for creating the Angle class
           \param broker Broker that allows to connect to the Nao's modules.
         */
        Angle(boost::shared_ptr<AL::ALBroker> broker);

        /*!
           \brief returns the keys of the angles.
           \return vector of strings representing the keys of the accelerometer
         */
        const StringKeyVector& getKeys();

        /*!
           \brief Returns the angle data
           \return shared pointer to the  SensorData
         */
        SensorData<double>::Ptr getAngle();
///@}

    private:
        StringKeyVector m_keys;
        boost::shared_ptr<AL::ALMemoryProxy> m_mem;
        boost::shared_ptr<AL::DCMProxy> m_dcm;
        AL::ALValue m_sensors;
    };
}


#endif //NAOMECH_ANGLE_H
