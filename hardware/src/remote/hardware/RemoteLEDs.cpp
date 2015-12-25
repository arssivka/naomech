//
// Created by arssivka on 12/25/15.
//

#include <rd/remote/hardware/RemoteLEDs.h>
#include <xmlrpc-c/girerr.hpp>

using namespace boost;
using namespace rd;
using namespace std;
using namespace xmlrpc_c;


RemoteLEDs::RemoteLEDs(shared_ptr<LEDs> leds) {
    this->addMethod(shared_ptr<RemoteMethod>(new KeysMethod(leds)));
    this->addMethod(shared_ptr<RemoteMethod>(new BrightnessMethod(leds)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteLEDs::KeysMethod::KeysMethod(boost::shared_ptr<LEDs> leds) {
    const vector<string> &keys = leds->getKeys();
    vector<value> values;
    for (vector<string>::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(value_string(*it));
    }
    this->keys = value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void RemoteLEDs::KeysMethod::execute(xmlrpc_c::paramList const &paramList,
                                     xmlrpc_c::value *const resultP) {
    *resultP = this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteLEDs::BrightnessMethod::BrightnessMethod(boost::shared_ptr<LEDs> leds)
        : leds(leds) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteLEDs::BrightnessMethod::execute(xmlrpc_c::paramList const &paramList,
                                           xmlrpc_c::value *const resultP) {
    // n:AA
    if (paramList.size() == 2) {
        vector<value> keys = paramList.getArray(0);
        vector<value> values = paramList.getArray(1);
        // Size check
        if (keys.size() != values.size()) throw girerr::error("Keys vector and values vector doesn't have same size");
        // Empty check
        if (keys.empty()) return;
        bool integer_keys = keys[0].type() == xmlrpc_c::value::TYPE_INT;
        vector<double> data;
        for (int i = 0; i < values.size(); ++i) data.push_back((double) value_double(values[i]));
        if (integer_keys) {
            vector<int> joint_names;
            for (int i = 0; i < keys.size(); ++i) joint_names.push_back((int) value_int(keys[i]));
            this->leds->setBrightness(joint_names, data);
        } else {
            vector<string> joint_names;
            for (int i = 0; i < keys.size(); ++i) joint_names.push_back((string) value_string(keys[i]));
            this->leds->setBrightness(joint_names, data);
        }
        *resultP = value_nil();
        return;
    }
    // S:
    shared_ptr<SensorData<double> > data;
    if (paramList.size() == 0) {
        data = this->leds->getBrightness();
    } else if (paramList.size() == 1) {
        // Empty check
        vector<value> keys = paramList.getArray(0);
        if (keys.empty()) return;
        bool integer_keys = keys[0].type() == xmlrpc_c::value::TYPE_INT;
        if (integer_keys) {
            vector<string> joint_names;
            for (int i = 0; i < keys.size(); ++i) joint_names.push_back((string) value_string(keys[i]));
            data = this->leds->getBrightness(joint_names);
        } else {
            vector<int> joint_names;
            for (int i = 0; i < keys.size(); ++i) joint_names.push_back((int) value_int(keys[i]));
            data = this->leds->getBrightness(joint_names);
        }
    } else throw girerr::error("Unknown signature for hardness function");

    // TODO Check for memory leaks
    // Some optimisation by using C library of xmlrpc-c
    xmlrpc_env env;
    xmlrpc_env_init(&env);

    xmlrpc_value *elem;
    xmlrpc_value *values;
    xmlrpc_value *result;

    // Init result array
    for (int i = 0; i < data->data.size(); ++i) {
        elem = xmlrpc_double_new(&env, data->data[i]);
        xmlrpc_array_append_item(&env, values, elem);
        xmlrpc_DECREF(elem);
    }

    // XMLRPC-C timestamp
    elem = xmlrpc_double_new(&env, data->timestamp);

    // Create result struct
    result = xmlrpc_struct_new(&env);
    xmlrpc_struct_set_value(&env, result, "data", values);
    xmlrpc_struct_set_value(&env, result, "timestamp", elem);

    // Apply result
    resultP->instantiate(result);

    // Clean this shit!
    xmlrpc_DECREF(elem);
    xmlrpc_DECREF(values);
    xmlrpc_DECREF(result);
    xmlrpc_env_clean(&env);
}

