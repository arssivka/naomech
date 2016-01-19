//
// Created by arssivka on 1/18/16.
//

#ifndef NAOMECH_REMOTEACCELEROMETER_H
#define NAOMECH_REMOTEACCELEROMETER_H

/*!
   \defgroup remote_accelerometer RemoteAccelerometer
   \ingroup hardware
 */


#include <rd/hardware/Accelerometer.h>
#include <rd/network/RemoteModule.h>

namespace rd {
///@{
/*!
   \brief Class for recieving the accelerometer data remotly
   This class allows to recieve nao's accelerometer data remotly via
   usage of RPC. This class is a child class of RemoteModule.
   The name of the module is accelerometer.
 */
    class RemoteAccelerometer : public RemoteModule {
    public:

        /*!
           \brief Constructor for creating the RemoteAccelerometer module
           \param accelerometer shared pointer to the Accelerometer object
         */
        RemoteAccelerometer(boost::shared_ptr<Accelerometer> accelerometer);

    private:
        class KeysMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote KeysMethod
               \param accelerometer Shared pointer to the Accelerometer object.

                KeysMethod is the class, object of which is stored in the RemoteAccelerometer module and represents the remote
                method named keys, that you can call under the name proxy.accelerometer.keys(). KeysMethod is the child class of
                the RemoteMethod and needed for recieving the keys of the gyro.
             */
            KeysMethod(boost::shared_ptr<Accelerometer> accelerometer);

            /*!
               \brief This is the function that executes all the needed
               operations of the remote KeysMethod
               \param paramList it is must be empty in this case
               \param resultP there will be stored array of keys, that you will recieve as returning value.
             */
            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            xmlrpc_c::value keys;
        };

        class AccelerationMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote AccelerationMethode
               \param accelerometer Shared pointer to the Accelerometer object.

                AccelerationMethod is the class, object of which is stored in the RemoteAccelerometer module and represents the remote
                method named acceleration, that you can call under the name proxyname.accelerometer.acceleration(params). AccelerationMethod is the child class of
                the RemoteMethod and needed for recieving the accelerometer data
             */
            AccelerationMethod(boost::shared_ptr<Accelerometer> accelerometer);

            /*!
               \brief execute This is the function that executes all the needed
               operations of the remote AccelerationMethod
               \param paramList empty
               \param resultP stores the returning value. Struct of array of accelerometer values and timestamp
             */
            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);
        ///@}
        private:
            boost::shared_ptr<Accelerometer> accelerometer;
        };
    };
}


#endif //NAOMECH_REMOTEACCELEROMETER_H
