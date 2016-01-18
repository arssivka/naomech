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

#include <rd/representation/SensorData.h>
#include <rd/hardware/Clock.h>


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
           \brief Constructor for creating the Jojnts class
           \param brocker Broker that allows to connect to the Nao's modules.
         */
        Joints(boost::shared_ptr<AL::ALBroker> brocker);

        /*!
           \brief Returns the vector of keys needed to control nao's hardware
           \return vector of keys.
         */
        const std::vector<std::string> &getKeys() const;

        /*!
           \brief Sets the position of motors
           \param keys keys of motors to be set. String keys in this case.
           \param values values needed to be set to the motors
           \return true if succed, otherwise false
         */
        bool setPosition(const std::vector<std::string> &keys,
                         const std::vector<double> &values);

        /*!
           \brief Sets the position of motors
           \param keys keys of motors to be set. Integer keys, represented by enum Key of this class.
           \param values values needed to be set to the motors
           \return true if succed, otherwise false.
         */
        bool setPosition(const std::vector<int> &keys,
                         const std::vector<double> &values);

        /*!
           \brief Returns the positions of motors
           \param keys keys of motors, which positions needed to be returned. Integer keys, represented by enum Key
           of this class.
           \return Sensor data (positions of motors), vector of double values in this case.
         */
        boost::shared_ptr<SensorData<double> > getPosition(const std::vector<int> &keys);


        /*!
           \brief Returns the positions of motors
           \param keys keys of motors, which positions needed to be returned. String keys in this case.
           \return Sensor data (positions of motors), vector of double values in this case.
         */
        boost::shared_ptr<SensorData<double> > getPosition(const std::vector<std::string> &keys);

        /*!
           \brief Returns the positions of all motors
           \return Sensor data (positions of motors), vector of double values in this case.
         */
        boost::shared_ptr<SensorData<double> > getPosition();

        /*!
           \brief Sets hardness to all motors
           \param value value of hardness to be set
           \return true if succed, otherwise false.
         */
        bool setHardness(double value);

        /*!
           \brief Sets hardness to all motors
           \param keys keys of motors, which hardness needed to be set. String keys in this case.
           \param value value of hardness to be set
           \return true if succed, otherwise false.
         */
        bool setHardness(const std::vector<std::string> &keys,
                         const std::vector<double> &values);

        /*!
           \brief Sets hardness to all motors
           \param keys keys of motors, which hardness needed to be set. Integer keys, represented by enum Key
           of this class
           \param value value of hardness to be set
           \return true if succed, otherwise false.
         */
        bool setHardness(const std::vector<int> &keys,
                         const std::vector<double> &values);

        /*!
           \brief Returns hardness of motors
           \param keys keys of motors, which hardness needed to be returned. Integer keys, represented by enum Key
           of this class
           \return Sensor data (hardness of motors), vector of double values in this case.
         */
        boost::shared_ptr<SensorData<double> > getHardness(const std::vector<int> &keys);

        /*!
           \brief Returns hardness of motors
           \param keys keys of motors, which hardness needed to be returned. String keys in this keys
           \return Sensor data (hardness of motors), vector of double values in this case.
         */
        boost::shared_ptr<SensorData<double> > getHardness(const std::vector<std::string> &keys);

        /*!
           \brief Returns hardness of all motors
           \return Sensor data (hardness of motors), vector of double values in this case.
         */
        boost::shared_ptr<SensorData<double> > getHardness();
        ///@}

    private:
        const static std::string DCM_POSITION_ALIAS;
        const static std::string DCM_HARDNESS_ALIAS;

        boost::shared_ptr<AL::ALMemoryProxy> mem;
        boost::shared_ptr<AL::DCMProxy> dcm;
        boost::shared_ptr<AL::ALProxy> hw;

        boost::mutex synch;
        AL::ALValue dcm_cmd;

        std::vector<std::string> keys;

        AL::ALValue position_in_list;
        AL::ALValue position_out_list;
        AL::ALValue hardness_list;
        std::map<std::string, int> out_map;
        std::map<std::string, std::string> position_map;
        std::map<std::string, std::string> hardness_map;

        void makeAlias(const std::string &name, const AL::ALValue &keys);

        void initKeysMap(std::map<std::string, std::string> &container,
                         const std::vector<std::string> &keys,
                         const std::vector<std::string> &values);
    };
}


#endif //NAOMECH_JOINTS_H
