//
// Created by arssivka on 12/28/15.
//

#include <rd/hardware/RemoteAngle.h>

using namespace boost;
using namespace rd;
using namespace std;
using namespace xmlrpc_c;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteAngle::RemoteAngle(shared_ptr<Angle> angle)
        : RemoteModule("angle") {
    this->addMethod(shared_ptr<RemoteMethod>(new KeysMethod(angle)));
    this->addMethod(shared_ptr<RemoteMethod>(new AngleMethod(angle)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteAngle::KeysMethod::KeysMethod(shared_ptr<Angle> angle)
        : RemoteMethod("keys", "A:", "Return array of angle sensor names") {
    const vector<string>& keys = angle->getKeys();
    vector<value> values;
    for (vector<string>::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(value_string(*it));
    }
    m_keys = value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteAngle::KeysMethod::execute(const xmlrpc_c::paramList& paramList,
                                      xmlrpc_c::value* const resultP) {
    *resultP = m_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteAngle::AngleMethod::AngleMethod(shared_ptr<Angle> angle)
        : RemoteMethod("angles", "S:", "Getting data from angle"), m_angle(angle) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteAngle::AngleMethod::execute(xmlrpc_c::paramList const& paramList,
                                       xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(0);
    SensorData<double>::Ptr data = m_angle->getAngle();

    // TODO Check for memory leaks
    // Some optimisation by using C library of xmlrpc-c
    xmlrpc_env env;
    xmlrpc_env_init(&env);

    xmlrpc_value* elem;
    xmlrpc_value* values;
    xmlrpc_value* result;

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
