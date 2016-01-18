//
// Created by arssivka on 12/25/15.
//

/*!
   \defgroup remote_jojnts RemoteJoints
   \ingroup hardware
 */

#ifndef NAOMECH_REMOTELEDS_H
#define NAOMECH_REMOTELEDS_H


#include <rd/network/RemoteModule.h>
#include <rd/hardware/LEDs.h>

namespace rd {
///@{
/*!
   \brief Class for remote controlling of nao's LEDs
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
            KeysMethod(boost::shared_ptr<LEDs> leds);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            xmlrpc_c::value keys;
        };

        class BrightnessMethod : public RemoteMethod {
        public:
            BrightnessMethod(boost::shared_ptr<LEDs> leds);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<LEDs> leds;
        };
    };
}


#endif //NAOMECH_REMOTELEDS_H
