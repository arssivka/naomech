//
// Created by arssivka on 11/27/15.
//

#ifndef NAOMECH_SENSORDATA_H
#define NAOMECH_SENSORDATA_H

/*!
   \defgroup sensordata SensorData
   \ingroup representation
 */
///@{
#include <boost/shared_ptr.hpp>
#include <xmlrpc-c/registry.hpp>

namespace rd {
    template<typename T>
    /*!
      \brief Class for managing different data that is used almost everywhere in the hardware modules
     */
    class SensorData {
    public:
        /*!
           \brief Basic constructor
         */
        SensorData() { }

        /*!
           \brief Constructor for creating the SensorData object
           \param length length of the vector that is stored in the class and needed for storing the data
           \param timestamp current timestamp
         */
        SensorData(const int length,
                   const int timestamp)
                : data(length), timestamp(timestamp) { }

        /*!
           \brief Constructor for creating the SensorData object
           \param data std vector of the data that needed to be stored in the SensorData class
           \param timestamp current timestamp
         */
        SensorData(const std::vector<T> &data,
                   const int timestamp)
                : data(data), timestamp(timestamp) { }

        /*!
           \brief data std vector needed for storing the data of type T in the class SensorData. It is a public member for the sake of usability.
         */
        std::vector<T> data;

        /*!
           \brief timestamp Timestamp. (Thanks, Captain!) It is a public member for the sake of usability.
         */
        int timestamp;
        ///@}
    };
}


#endif //NAOMECH_SENSORDATA_H
