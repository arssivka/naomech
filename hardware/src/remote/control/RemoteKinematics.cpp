//
// Created by arssivka on 1/18/16.
//

#include <rd/remote/control/RemoteKinematics.h>
#include <xmlrpc-c/girerr.hpp>

using namespace boost;
using namespace rd;
using namespace std;
using namespace xmlrpc_c;


RemoteKinematics::RemoteKinematics(shared_ptr<Kinematics> kinematics)
        : RemoteModule("kinematics") {
    this->addMethod(shared_ptr<RemoteMethod>(new KeysMethod(kinematics)));
    this->addMethod(shared_ptr<RemoteMethod>(new PositionMethod(kinematics)));
    this->addMethod(shared_ptr<RemoteMethod>(new LookAtMethod(kinematics)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteKinematics::KeysMethod::KeysMethod(boost::shared_ptr<Kinematics> kinematics)
        : RemoteMethod("keys", "A:", "Return array of Kinematics names") {
    const vector<string>& keys = kinematics->getKeys();
    vector<value> values;
    for (vector<string>::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(value_string(*it));
    }
    this->keys = value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void RemoteKinematics::KeysMethod::execute(xmlrpc_c::paramList const& paramList,
                                           xmlrpc_c::value* const resultP) {
    *resultP = this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteKinematics::PositionMethod::PositionMethod(boost::shared_ptr<Kinematics> kinematics)
        : RemoteMethod("positions", "S:,S:A,n:AA", "Position control method"), kinematics(kinematics) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteKinematics::PositionMethod::execute(xmlrpc_c::paramList const& paramList,
                                               xmlrpc_c::value* const resultP) {
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
            this->kinematics->setPosition(joint_names, data);
        } else {
            vector<string> joint_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) joint_names[i] = value_string(keys[i]);
            this->kinematics->setPosition(joint_names, data);
        }
        *resultP = value_nil();
        return;
    }
    // S:
    shared_ptr<SensorData<double> > data;
    if (paramList.size() == 0) {
        data = this->kinematics->getPosition();
        // S:A
    } else if (paramList.size() == 1) {
        // Empty check
        vector<value> keys = paramList.getArray(0);
        if (keys.empty()) return;
        bool integer_keys = keys[0].type() == xmlrpc_c::value::TYPE_INT;
        if (integer_keys) {
            vector<int> joint_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) joint_names[i] = (int) value_int(keys[i]);
            data = this->kinematics->getPosition(joint_names);
        } else {
            vector<string> joint_names(keys.size());
            for (int i = 0; i < keys.size(); ++i) joint_names[i] = (string) value_string(keys[i]);
            data = this->kinematics->getPosition(joint_names);
        }
    } else throw girerr::error("Unknown signature for hardness function");
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteKinematics::LookAtMethod::LookAtMethod(boost::shared_ptr<Kinematics> kinematics)
        : RemoteMethod("lookAt", "n:dddb", "Looks at point in the world"), kinematics(kinematics) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteKinematics::LookAtMethod::execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value* const resultP) {
    paramList.verifyEnd(4);
    double x = paramList.getDouble(0);
    double y = paramList.getDouble(1);
    double z = paramList.getDouble(2);
    bool top_camera = paramList.getBoolean(3);
    this->kinematics->lookAt(x, y, z, top_camera);
    *resultP = value_nil();
}
