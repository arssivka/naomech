//
// Created by arssivka on 12/25/15.
//

#include <rd/hardware/RemoteLEDs.h>
#include <xmlrpc-c/girerr.hpp>

using namespace boost;
using namespace rd;
using namespace std;
using namespace xmlrpc_c;


RemoteLEDs::RemoteLEDs(shared_ptr<LEDs> leds)
        : RemoteModule("leds") {
    this->addMethod(shared_ptr<RemoteMethod>(new KeysMethod(leds)));
    this->addMethod(shared_ptr<RemoteMethod>(new BrightnessMethod(leds)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteLEDs::KeysMethod::KeysMethod(boost::shared_ptr<LEDs> leds)
        : RemoteMethod("keys", "A:", "Return array of LEDs names") {
    const vector<string> &keys = leds->getKeys();
    vector<value> values;
    for (vector<string>::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(value_string(*it));
    }
    m_keys = value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void RemoteLEDs::KeysMethod::execute(xmlrpc_c::paramList const &paramList,
                                     xmlrpc_c::value *const resultP) {
    *resultP = m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteLEDs::BrightnessMethod::BrightnessMethod(boost::shared_ptr<LEDs> leds)
        : RemoteMethod("brightness", "S:,S:A,n:AA", "Brightness control method"), m_leds(leds) { }

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
        vector<double> data(values.size());
        for (int i = 0; i < values.size(); ++i) data[i] = value_double(values[i]);
        if (integer_keys) {
            vector<int> joint_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) joint_names[i] = value_int(keys[i]);
            m_leds->setBrightness(joint_names, data);
        } else {
            vector<string> joint_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) joint_names[i] = value_string(keys[i]);
            m_leds->setBrightness(joint_names, data);
        }
        *resultP = value_nil();
        return;
    }
    // S:
    SensorData<double>::Ptr data;
    if (paramList.size() == 0) {
        data = m_leds->getBrightness();
    // S:A
    } else if (paramList.size() == 1) {
        // Empty check
        vector<value> keys = paramList.getArray(0);
        if (keys.empty()) return;
        bool integer_keys = keys[0].type() == xmlrpc_c::value::TYPE_INT;
        if (integer_keys) {
            vector<int> joint_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) joint_names[i] = (int) value_int(keys[i]);
            data = m_leds->getBrightness(joint_names);
        } else {
            vector<string> joint_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) joint_names[i] = (string) value_string(keys[i]);
            data = m_leds->getBrightness(joint_names);
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
    values = xmlrpc_array_new(&env);
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

