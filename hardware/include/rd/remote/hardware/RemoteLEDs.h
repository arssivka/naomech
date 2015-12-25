//
// Created by arssivka on 12/25/15.
//

#ifndef NAOMECH_REMOTELEDS_H
#define NAOMECH_REMOTELEDS_H


#include <rd/network/RemoteModule.h>
#include <rd/hardware/LEDs.h>

namespace rd {
    class RemoteLEDs : public RemoteModule {
    public:
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
