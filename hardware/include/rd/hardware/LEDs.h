//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_LED_H
#define NAOMECH_LED_H


/*!        \defgroup leds LEDs
           \ingroup hardware
 */


#include <rd/hardware/SensorData.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#include "TypeDefinition.h"


namespace rd {
///@{
/*!
       \brief Class for controlling nao LEDs.

           This class allows to set and get
           brightnesses of nao LEDs.
     */
    class LEDs {
    public:

        /*!
           \brief The Key enumeration. Represetns the keys for managing nao LEDs.
         */
        enum Key {
            CHEST_BOARD_RED,
            CHEST_BOARD_GREEN,
            CHEST_BOARD_BLUE,
            L_EAR_0_DEG,
            L_EAR_108_DEG,
            L_EAR_144_DEG,
            L_EAR_180_DEG,
            L_EAR_216_DEG,
            L_EAR_252_DEG,
            L_EAR_288_DEG,
            L_EAR_324_DEG,
            L_EAR_36_DEG,
            L_EAR_72_DEG,
            R_EAR_0_DEG,
            R_EAR_108_DEG,
            R_EAR_144_DEG,
            R_EAR_180_DEG,
            R_EAR_216_DEG,
            R_EAR_252_DEG,
            R_EAR_288_DEG,
            R_EAR_324_DEG,
            R_EAR_36_DEG,
            R_EAR_72_DEG,
            L_FACE_RED_0_DEG,
            L_FACE_GREEN_0_DEG,
            L_FACE_BLUE_0_DEG,
            L_FACE_RED_135_DEG,
            L_FACE_GREEN_135_DEG,
            L_FACE_BLUE_135_DEG,
            L_FACE_RED_180_DEG,
            L_FACE_GREEN_180_DEG,
            L_FACE_BLUE_180_DEG,
            L_FACE_RED_225_DEG,
            L_FACE_GREEN_225_DEG,
            L_FACE_BLUE_225_DEG,
            L_FACE_RED_270_DEG,
            L_FACE_GREEN_270_DEG,
            L_FACE_BLUE_270_DEG,
            L_FACE_RED_315_DEG,
            L_FACE_GREEN_315_DEG,
            L_FACE_BLUE_315_DEG,
            L_FACE_RED_45_DEG,
            L_FACE_GREEN_45_DEG,
            L_FACE_BLUE_45_DEG,
            L_FACE_RED_90_DEG,
            L_FACE_GREEN_90_DEG,
            L_FACE_BLUE_90_DEG,
            R_FACE_RED_0_DEG,
            R_FACE_GREEN_0_DEG,
            R_FACE_BLUE_0_DEG,
            R_FACE_RED_135_DEG,
            R_FACE_GREEN_135_DEG,
            R_FACE_BLUE_135_DEG,
            R_FACE_RED_180_DEG,
            R_FACE_GREEN_180_DEG,
            R_FACE_BLUE_180_DEG,
            R_FACE_RED_225_DEG,
            R_FACE_GREEN_225_DEG,
            R_FACE_BLUE_225_DEG,
            R_FACE_RED_270_DEG,
            R_FACE_GREEN_270_DEG,
            R_FACE_BLUE_270_DEG,
            R_FACE_RED_315_DEG,
            R_FACE_GREEN_315_DEG,
            R_FACE_BLUE_315_DEG,
            R_FACE_RED_45_DEG,
            R_FACE_GREEN_45_DEG,
            R_FACE_BLUE_45_DEG,
            R_FACE_RED_90_DEG,
            R_FACE_GREEN_90_DEG,
            R_FACE_BLUE_90_DEG,
            L_HEAD_FRONT_0,
            L_HEAD_FRONT_1,
            R_HEAD_FRONT_0,
            R_HEAD_FRONT_1,
            L_HEAD_MIDDLE_0,
            R_HEAD_MIDDLE_0,
            L_HEAD_REAR_0,
            L_HEAD_REAR_1,
            L_HEAD_REAR_2,
            R_HEAD_REAR_0,
            R_HEAD_REAR_1,
            R_HEAD_REAR_2,
            L_FOOT_RED,
            L_FOOT_GREEN,
            L_FOOT_BLUE,
            R_FOOT_RED,
            R_FOOT_GREEN,
            R_FOOT_BLUE,
            LEDS_COUNT
        };

        /*!
           \brief Constructor for creating the nao LEDs class.
           \param broker Broker that allows to connect to the Nao's modules.
         */
        LEDs(boost::shared_ptr<AL::ALBroker> broker);

        /*!
           \brief Returns the vector containing keys of the LEDs
           \return vector of strings representing LEDs keys
         */
        const StringKeyVector &getKeys() const;

        /*!
           \brief Method for setting the brightnesses of the LEDs
           \param keys Keys of the LEDs needed to be set represented via vector of strings
           \param values Values to set to the LEDs
           \return Boolean true if succed otherwise false
         */
        bool setBrightness(const StringKeyVector &keys,
                           const ValuesVector &values);

        /*!
           \brief Method for setting the brightnesses of the LEDs
           \param keys Keys of the LEDs needed to be set represented via vector of integers from enumeration
           \param values Values to set to the LEDs
           \return Boolean true if succed otherwise false
         */
        bool setBrightness(const IntegerKeyVector &keys,
                           const ValuesVector &values);

        /*!
           \brief Method that returns values of the specified LEDs
           \param keys Keys of the LEDs represented via vector of strings
           \return shared pointer to the SensorData
         */
        SensorData<double>::Ptr getBrightness(const IntegerKeyVector &keys);

        /*!
           \brief Method that returns values of the specified LEDs
           \param keys Keys of the LEDs represented via vector of strings
           \return shared pointer to the SensorData
         */
        SensorData<double>::Ptr getBrightness(const StringKeyVector &keys);

        /*!
           \brief Method that returns values of all the LEDs
           \return shared pointer to the SensorData
         */
        SensorData<double>::Ptr getBrightness();
         ///@}

    private:
        boost::shared_ptr<AL::ALMemoryProxy> m_mem;
        boost::shared_ptr<AL::DCMProxy> m_dcm;
        boost::mutex m_synch;
        AL::ALValue m_cmd;

        StringKeyVector m_keys;
        StringKeyVector m_leds_list;
        std::map<std::string, std::string> m_leds_map;

        void initKeysMap(std::map<std::string, std::string> &container,
                         const StringKeyVector &keys,
                         const StringKeyVector &values);
    };
}


#endif //NAOMECH_LED_H
