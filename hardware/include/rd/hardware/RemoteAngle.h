//
// Created by arssivka on 2/11/16.
//

#ifndef NAOMECH_REMOTEANGLE_H
#define NAOMECH_REMOTEANGLE_H


/*! \defgroup remote_hardware RemoteHardware
    \ingroup hardware
*/

/*!
   \defgroup remote_Angle RemoteAngle
   \ingroup remote_hardware
 */


#include <rd/hardware/Angle.h>
#include <rd/network/RemoteModule.h>

namespace rd {
///@{
/*!
   \brief Class for recieving the Angle data remotly
   This class allows to recieve nao's angle data remotly via
   usage of RPC. This class is a child class of RemoteModule.
   The name of the module is Angle.
 */
    class RemoteAngle : public RemoteModule {
    public:

        /*!
           \brief Constructor for creating the RemoteAngle module
           \param Angle shared pointer to the Angle object
         */
        RemoteAngle(boost::shared_ptr<Angle> angle);

    private:
        class KeysMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote KeysMethod
               \param Angle Shared pointer to the Angle object.

                KeysMethod is the class, object of which is stored in the RemoteAngle module and represents the remote
                method named keys, that you can call under the name proxy.angle.keys(). KeysMethod is the child class of
                the RemoteMethod and needed for recieving the keys of the gyro.
             */
            KeysMethod(boost::shared_ptr<Angle> angle);

            /*!
               \brief This is the function that executes all the needed
               operations of the remote KeysMethod
               \param paramList it is must be empty in this case
               \param resultP there will be stored array of keys, that you will recieve as returning value.
             */
            virtual void execute(xmlrpc_c::paramList const& paramList,
                                 xmlrpc_c::value* const resultP);

        private:
            xmlrpc_c::value m_keys;
        };

        class AngleMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote AccelerationMethode
               \param Angle Shared pointer to the Angle object.

                AccelerationMethod is the class, object of which is stored in the RemoteAngle module and represents the remote
                method named acceleration, that you can call under the name proxyname.angle.acceleration(params). AccelerationMethod is the child class of
                the RemoteMethod and needed for recieving the Angle data
             */
            AngleMethod(boost::shared_ptr<Angle> angle);

            /*!
               \brief execute This is the function that executes all the needed
               operations of the remote AccelerationMethod
               \param paramList empty
               \param resultP stores the returning value. Struct of array of Angle values and timestamp
             */
            virtual void execute(xmlrpc_c::paramList const& paramList,
                                 xmlrpc_c::value* const resultP);
            ///@}
        private:
            boost::shared_ptr<Angle> m_angle;
        };
    };
}


#endif //NAOMECH_REMOTEANGLE_H
