#ifndef NAOMECH_REMOTEJOINTS_H
#define NAOMECH_REMOTEJOINTS_H

/*!
   \defgroup remote_jojnts RemoteJoints
   \ingroup remote_hardware
 */

#include <rd/network/RemoteModule.h>
#include <rd/hardware/Joints.h>

namespace rd {
///@{
    /*!
       \brief Class for remote controlling the nao's joints
       This class allows to remote control nao's joints via
       usage of RPC. This class is a child class of RemoteModule.
       The name of the module is joints.
     */
    class RemoteJoints : public RemoteModule {
    public:
        /*!
           \brief Constructor for creating the remote joints module
           \param joints Shared pointer to the Joints object.
         */
        RemoteJoints(boost::shared_ptr<Joints> joints);

        /*!
          \brief keys remote method that returns array of joint names
          \return array of joint names
         */
    private:
        class KeysMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote KeysMethod
               \param joints Shared pointer to the Joints object.

                KeysMethod is the class, object of which is stored in the RemoteJoints module and represents the remote
                method named keys, that you can call under the name proxy.joints.keys(). KeysMethod is the child class of
                the RemoteMethod and needed for recieving the keys of the mototrs.
             */
            KeysMethod(boost::shared_ptr<Joints> joints);

            /*!
               \brief This is the function that executes all the needed
               operations of the remote KeysMethod
               \param paramList it is must be empty in this case
               \param resultP there will be stored array of keys, that you will recieve as returning value.
             */
            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);
        private:
            xmlrpc_c::value m_keys;
        };

        class PositionMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote PositionMethod
               \param joints Shared pointer to the Joints object.

                PositionMethod is the class object of which is stored in the RemoteJoints module and represents the remote
                method named position, that you can call under the name proxyname.joints.position(params). PositionMethod is the child class of
                the RemoteMethod and needed for controlling the positions of motors
             */
            PositionMethod(boost::shared_ptr<Joints> joints);

            /*!
               \brief execute This is the function that executes all the needed
               operations of the remote PositionMethod
               \param paramList it has different variations:
               1. Empty
               2. Array of keys of the motors, which positions are nedded to be returned
               3. Two arrays: First array - array fo of keys of the motors; Second array - array of values that needed to be set to the mototrs
               \param resultP stores the returning value. It has different variations:
               1. Struct of array of positions of all motors and timestamp
               2. Struct of array of positions of motors and timestamp
               3. Empty
             */
            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Joints> joints;
        };

        class HardnessMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote PositionMethod
               \param joints Shared pointer to the Joints object.

                HardnessMethod is the class object of which is stored in the RemoteJoints module and represents the remote
                method named hardness, that you can call under the name proxyname.joints.hardness(params). HardnessMethod is the child class of
                the RemoteMethod and needed for the controlling the hardness of motors
             */
            HardnessMethod(boost::shared_ptr<Joints> joints);

            /*!
               \brief execute This is the function that executes all the needed
               operations of the remote HardnessMethod
               \param paramList it has different variations:
               1. Empty
               2. Array of keys of the motors, which hardness values are nedded to be returned
               3. Two arrays: First array - array of keys of the motors; Second array - array of hardness values that needed to be set to the mototrs
               4. Value that needed to be set to all the motors
               \param resultP stores the returning value. It has different variations:
               1. Struct of array of hardness values of all motors and timestamp
               2. Struct of array of hardness values of motors and timestamp
               3. Empty
               4. Empty
             */
            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);
            ///@}

        private:
            boost::shared_ptr<Joints> m_joints;
        };
    };
}


#endif //NAOMECH_REMOTEJOINTS_H
