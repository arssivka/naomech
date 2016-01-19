//
// Created by arssivka on 12/15/15.
//

#ifndef NAOMECH_HWCONTROLLER_H
#define NAOMECH_HWCONTROLLER_H

/*!
  \defgroup naoqi NAOQi
  */

/*!
  \defgroup hwcontroler HWController
  \ingroup naoqi
  */


#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>


namespace AL {
    class ALBroker;
    class ALMemoryFastAccess;
    class DCMProxy;
}
///@{
namespace rd {
/*!
       \brief Enumeration for managing the hardware parts
     */
    enum SensorType {
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
        NUM_OF_JOINTS
    };


    /*!
       \brief This is the class for providing acces to the nao's hardware. This is the only naoqi-thing that we are using.
        It is the noaqi-module that allows you to exchange data with nao's hardware.
        We were not able to refuse usage of this naoqi-module because we
        failed to find other way to control nao actuators smoothly and without errors.
     */
    class HWController : public AL::ALModule {
    public:
        /*!
           \brief Construcot for creating the HWController
           \param pBroker broker for providing connection to the nao
           \param pName name of the module
         */
        HWController(boost::shared_ptr<AL::ALBroker> pBroker,
                     const std::string &pName);

        /*!
           \brief Destructor (Thanks, Captain!)
         */
        virtual ~HWController();

        /*!
           \brief init runs all the methods for initialize the HWController parts
         */
        virtual void init();

        /*!
           \brief Sends the desired values to the joints
           \param values the diserid values to be set to the joints
         */
        void setJointData(const AL::ALValue &values);

        /*!
           \brief It is for recieving current values from nao joints
           \return ALValue that stores all the needed values
         */
        AL::ALValue getJointData();
///@}
    private:

        void startLoop();

        void stopLoop();

        void initFastAccess();

        void connectToDCMloop();

        void synchronisedDCMcallback();

        void createPositionActuatorAlias();

        void preparePositionActuatorCommand();

        ProcessSignalConnection dcm_post_process_connection;
        boost::shared_ptr<AL::ALMemoryFastAccess> mem;
        boost::shared_ptr<AL::DCMProxy> dcm;

        std::vector<std::string> sensor_keys;

        std::vector<float> sensor_data;
        AL::ALValue sensor_values;

        boost::mutex actuator_mutex;
        boost::mutex sensor_mutex;

        boost::shared_ptr<std::vector<float> > work_actuator_values;
        AL::ALValue commands;
    };
}

#endif //NAOMECH_HWCONTROLLER_H
