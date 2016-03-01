#ifndef NAOMECH_JOINTS_H
#define NAOMECH_JOINTS_H

/*! \defgroup hardware Hardware*/

/*!        \defgroup jojnts Joints
           \ingroup hardware
 */

#include <vector>
#include <map>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alcommon/alproxy.h>

#include <rd/hardware/SensorData.h>
#include <rd/hardware/Clock.h>
#include "TypeDefinition.h"


namespace rd {
///@{
/*!
       \brief Class for controlling nao joints.

           This class allows to set and get
           values of nao joints. Also it allows
           to control hardness.
     */
    class Joints : boost::noncopyable {
    public:
        /*!
           \brief The Key enumeration. Represents the keys for managing the joints
         */
        enum Key {
            HEAD_YAW, HEAD_PITCH,
            L_SHOULDER_PITCH,
            L_SHOULDER_ROLL,
            L_ELBOW_YAW,
            L_ELBOW_ROLL,
            L_WRIST_YAW,
            L_HAND,
            L_HIP_YAW_PITCH,
            L_HIP_ROLL,
            L_HIP_PITCH,
            L_KNEE_PITCH,
            L_ANKLE_PITCH,
            L_ANKLE_ROLL,
            R_HIP_YAW_PITCH,
            R_HIP_ROLL,
            R_HIP_PITCH,
            R_KNEE_PITCH,
            R_ANKLE_PITCH,
            R_ANKLE_ROLL,
            R_SHOULDER_PITCH,
            R_SHOULDER_ROLL,
            R_ELBOW_YAW,
            R_ELBOW_ROLL,
            R_WRIST_YAW,
            R_HAND,
            JOINTS_COUNT
        };

        /*!
           \brief HEAD_JOINTS_COUNT constant that stores quantiy of head joints
         */
        static const int HEAD_JOINTS_COUNT = HEAD_PITCH - HEAD_YAW + 1;
        /*!
           \brief ARM_JOINTS_COUNT constant that stores quantity if arm joints
         */
        static const int ARM_JOINTS_COUNT = L_HAND - L_SHOULDER_PITCH + 1;
        /*!
           \brief LEG_JOINTS_COUNT constant that stores quantity of leg joints
         */
        static const int LEG_JOINTS_COUNT = L_ANKLE_ROLL - L_HIP_YAW_PITCH + 1;

        /*!
           \brief HEAD_FIRST_JOINT constant that stores number of the first head joint
         */
        static const int HEAD_FIRST_JOINT = HEAD_YAW;
        /*!
           \brief LEFT_ARM_FIRST_JOINT constatnt that stores number of the first left arm joint
         */
        static const int LEFT_ARM_FIRST_JOINT = L_SHOULDER_PITCH;
        /*!
           \brief RIGHT_ARM_FIRST_JOINT constatnt that stores number of the first right arm joint
         */
        static const int RIGHT_ARM_FIRST_JOINT = R_SHOULDER_PITCH;
        /*!
           \brief LEFT_LEG_FIRST_JOINT constant that stores number of the first left leg joint
         */
        static const int LEFT_LEG_FIRST_JOINT = L_HIP_YAW_PITCH;
        /*!
           \brief RIGHT_LEG_FIRST_JOINT constant that stores number of the first right leg joint
         */
        static const int RIGHT_LEG_FIRST_JOINT = R_HIP_YAW_PITCH;

        /*!
           \brief Constructor for creating the Joints class
           \param brocker Broker that allows to connect to the Nao's modules.
         */
        Joints(boost::shared_ptr<AL::ALBroker> broker);

        /*!
           \brief Returns the vector of keys needed to control nao's hardware
           \return vector of keys.
         */
        const StringKeyVector &getKeys() const;

        /*!
           \brief Sets the position of motors
           \param keys keys of motors to be set. String keys in this case.
           \param values values needed to be set to the motors
           \return true if succed, otherwise false
         */
        bool setPositions(const StringKeyVector& keys,
                          const ValuesVector& values);

        /*!
           \brief Sets the position of motors
           \param keys keys of motors to be set. Integer keys, represented by enum Key of this class.
           \param values values needed to be set to the motors
           \return true if succed, otherwise false.
         */
        bool setPositions(const IntegerKeyVector& keys,
                          const ValuesVector& values);

        /*!
           \brief Returns the positions of motors
           \param keys keys of motors, which positions needed to be returned. Integer keys, represented by enum Key
           of this class.
           \return Sensor data (positions of motors), vector of double values in this case.
         */
        SensorData<double>::Ptr getPositions(const IntegerKeyVector& keys);


        /*!
           \brief Returns the positions of motors
           \param keys keys of motors, which positions needed to be returned. String keys in this case.
           \return Sensor data (positions of motors), vector of double values in this case.
         */
        SensorData<double>::Ptr getPositions(const StringKeyVector& keys);

        /*!
           \brief Returns the positions of all motors
           \return Sensor data (positions of motors), vector of double values in this case.
         */
        SensorData<double>::Ptr getPositions();

        /*!
           \brief Sets hardness to all motors
           \param value value of hardness to be set
           \return true if succed, otherwise false.
         */
        bool setHardness(double value);

        /*!
           \brief Sets hardness to all motors
           \param keys keys of motors, which hardness needed to be set. String keys in this case.
           \param values value of hardness to be set
           \return true if succed, otherwise false.
         */
        bool setHardness(const StringKeyVector &keys,
                         const ValuesVector &values);

        /*!
           \brief Sets hardness to all motors
           \param keys keys of motors, which hardness needed to be set. Integer keys, represented by enum Key
           of this class
           \param values value of hardness to be set
           \return true if succed, otherwise false.
         */
        bool setHardness(const IntegerKeyVector &keys,
                         const ValuesVector &values);

        /*!
           \brief Returns hardness of motors
           \param keys keys of motors, which hardness needed to be returned. Integer keys, represented by enum Key
           of this class
           \return Sensor data (hardness of motors), vector of double values in this case.
         */
        SensorData<double>::Ptr getHardness(const IntegerKeyVector &keys);

        /*!
           \brief Returns hardness of motors
           \param keys keys of motors, which hardness needed to be returned. String keys in this keys
           \return Sensor data (hardness of motors), vector of double values in this case.
         */
        SensorData<double>::Ptr getHardness(const StringKeyVector &keys);

        /*!
           \brief Returns hardness of all motors
           \return Sensor data (hardness of motors), vector of double values in this case.
         */
        SensorData<double>::Ptr getHardness();
        ///@}

    private:
        const static std::string DCM_HARDNESS_ALIAS;

        boost::shared_ptr<AL::ALMemoryProxy> m_mem;
        boost::shared_ptr<AL::DCMProxy> m_dcm;
        boost::shared_ptr<AL::ALProxy> m_hw;

        boost::mutex m_synch;
        AL::ALValue m_dcm_cmd;

        StringKeyVector m_keys;


        AL::ALValue m_hardness_list;
        std::map<std::string, int> m_out_map;
        std::map<std::string, std::string> m_hardness_map;

        void makeAlias(const std::string &name, const AL::ALValue &keys);
    };
}


#endif //NAOMECH_JOINTS_H
