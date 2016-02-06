//
// Created by arssivka on 12/25/15.
//


#ifndef NAOMECH_REMOTELEDS_H
#define NAOMECH_REMOTELEDS_H


/*!
   \defgroup remote_leds RemoteLEDs
   \ingroup remote_hardware
 */


#include <rd/network/RemoteModule.h>
#include <rd/hardware/LEDs.h>

namespace rd {
///@{
/*!
   \brief Class for remote controlling the nao's LEDs
   This class allows to remote control nao's LEDs via
   usage of RPC. This class is a child class of RemoteModule.
   The name of the module is leds.
 */
    class RemoteLEDs : public RemoteModule {
    public:
        /*!
           \brief Constructor for creating the remote leds module
           \param leds shared pointer to the LEDs object
         */
        RemoteLEDs(boost::shared_ptr<LEDs> leds);

    private:
        class KeysMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote KeysMethod
               \param leds Shared pointer to the LEDs object.

                KeysMethod is the class, object of which is stored in the RemoteLEDs module and represents the remote
                method named keys, that you can call under the name proxy.leds.keys(). KeysMethod is the child class of
                the RemoteMethod and needed for recieving the keys of the LEDs.
             */
            KeysMethod(boost::shared_ptr<LEDs> leds);

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

        class BrightnessMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote BrightnessMethod
               \param leds Shared pointer to the LEDs object.

                BrightnessMethod is the class object of which is stored in the RemoteLEDs module and represents the remote
                method named brightness, that you can call under the name proxyname.leds.brightness(params). BrightnessMethod is the child class of
                the RemoteMethod and needed for controlling the brightnesses of LEDs
             */
            BrightnessMethod(boost::shared_ptr<LEDs> leds);

            /*!
               \brief execute This is the function that executes all the needed
               operations of the remote BrightnessMethod
               \param paramList it has different variations:
               1. Empty
               2. Array of keys of the LEDs, which values are nedded to be returned specified via strings or integers
               3. Two arrays: First array - array of keys of the LEDs; Second array - array of brightness values that needed to be set to the LEDs
               \param resultP stores the returning value. It has different variations:
               1. Struct of array of all LEDs values and timestamp
               2. Struct of array of LEDs values and timestamp
               3. Empty
             */
            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);
            ///@}
        private:
            boost::shared_ptr<LEDs> m_leds;
        };
    };
}


#endif //NAOMECH_REMOTELEDS_H
