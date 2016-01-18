//
// Created by arssivka on 12/28/15.
//

#include <rd/remote/hardware/RemoteAccelerometer.h>

using namespace boost;
using namespace rd;
using namespace std;
using namespace xmlrpc_c;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteAccelerometer::RemoteAccelerometer(shared_ptr<Accelerometer> accelerometer)
        : RemoteModule("accelerometer") {
    this->addMethod(shared_ptr<RemoteMethod>(new KeysMethod(accelerometer)));
    this->addMethod(shared_ptr<RemoteMethod>(new AngularVelocityMethod(accelerometer)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteAccelerometer::KeysMethod::KeysMethod(shared_ptr<Accelerometer> accelerometer)
        : RemoteMethod("keys", "A:", "Return array of accelerometer sensor names") {
    const vector<string> &keys = accelerometer->getKeys();
    vector<value> values;
    for (vector<string>::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(value_string(*it));
    }
    this->keys = value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteAccelerometer::KeysMethod::execute(const xmlrpc_c::paramList &paramList,
                                              xmlrpc_c::value *const resultP) {
    *resultP = this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteAccelerometer::AccelerationMethod::AngularVelocityMethod(shared_ptr<Accelerometer> accelerometer)
        : RemoteMethod("acceleration", "S:", "Getting data from accelerometer"), accelerometer(accelerometer) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteAccelerometer::AccelerationMethod::execute(xmlrpc_c::paramList const &paramList,
                                                         xmlrpc_c::value *const resultP) {
    paramList.verifyEnd(0);
    shared_ptr<SensorData<double> > data = this->accelerometer->getAcceleration();

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
