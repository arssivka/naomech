//
// Created by arssivka on 12/28/15.
//

#ifndef NAOMECH_REMOTEGYRO_H
#define NAOMECH_REMOTEGYRO_H

/*!
   \defgroup remote_gyro RemoteGyro
   \ingroup hardware
 */


#include <rd/hardware/Gyro.h>
#include <rd/network/RemoteModule.h>

namespace rd {
///@{
/*!
   \brief Class for recieving the gyro data remotly
   This class allows to recieve nao's gyro data remotly via
   usage of RPC. This class is a child class of RemoteModule.
   The name of the module is gyro.
 */
    class RemoteGyro : public RemoteModule {
    public:

        /*!
           \brief Constructor for creating the RemoteGyro object
           \param gyro Shared pointer to the Gyro object
         */
        RemoteGyro(boost::shared_ptr<Gyro> gyro);

    private:
        class KeysMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote KeysMethod
               \param leds Shared pointer to the Gyro object.

                KeysMethod is the class, object of which is stored in the RemoteGyro module and represents the remote
                method named keys, that you can call under the name proxy.gyro.keys(). KeysMethod is the child class of
                the RemoteMethod and needed for recieving the keys of the gyro.
             */
            KeysMethod(boost::shared_ptr<Gyro> gyro);

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

        class AngularVelocityMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote AngularVelocityMethod
               \param joints Shared pointer to the Gyro object.

                AngularVelocity is the class, object of which is stored in the RemoteGyro module and represents the remote
                method named angularvelocity, that you can call under the name proxyname.gyro.angularvelocity(params). AngularVelocityMethod is the child class of
                the RemoteMethod and needed for recieving the gyro data
             */
            AngularVelocityMethod(boost::shared_ptr<Gyro> gyro);

            /*!
               \brief execute This is the function that executes all the needed
               operations of the remote AngularVelocityMethod
               \param paramList empty
               \param resultP stores the returning value. Struct of array of gyro values and timestamp
             */
            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);
        ///@}
        private:
            boost::shared_ptr<Gyro> gyro;
        };
    };
}


#endif //NAOMECH_REMOTEGYRO_H
