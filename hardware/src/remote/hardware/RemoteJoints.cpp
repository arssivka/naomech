//
// Created by arssivka on 11/27/15.
//

#include <rd/remote/hardware/RemoteJoints.h>
#include <xmlrpc-c/girerr.hpp>

using namespace xmlrpc_c;
using namespace boost;
using namespace std;
using namespace rd;

RemoteJoints::RemoteJoints(shared_ptr<Joints> joints)
        : RemoteModule("joints") {
    this->addMethod(shared_ptr<RemoteMethod>(new KeysMethod(joints)));
    this->addMethod(shared_ptr<RemoteMethod>(new HardnessMethod(joints)));
    this->addMethod(shared_ptr<RemoteMethod>(new PositionMethod(joints)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteJoints::KeysMethod::KeysMethod(shared_ptr<Joints> joints)
        : RemoteMethod("keys", "A:", "Return array of joint names") {
    const vector<string> &keys = joints->getInputKeys();
    vector<value> values;
    for (vector<string>::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(value_string(*it));
    }
    this->keys = value_array(values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteJoints::KeysMethod::execute(paramList const &paramList, value *const resultP) {
    *resultP = this->keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteJoints::HardnessMethod::HardnessMethod(shared_ptr<Joints> joints)
        : RemoteMethod("hardness", "S:,S:A,n:AAi,n:di", "Stifness control method"), joints(joints) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteJoints::HardnessMethod::execute(paramList const &paramList, value *const resultP) {
    // n:di
    if (paramList.size() == 2) {
        double value = paramList.getDouble(0);
        int time_offset = paramList.getInt(1);
        this->joints->setHardness(value, time_offset);
        return;
    }
    // n:AAi
    if (paramList.size() == 3) {
        vector<value> keys = paramList.getArray(0);
        vector<value> values = paramList.getArray(1);
        int time_offset = paramList.getInt(2);
        // Size check
        if (keys.size() != values.size()) throw girerr::error("Keys vector and values vector doesn't have same size");
        // Empty check
        if (keys.empty()) return;
        bool integer_keys = keys[0].type() == xmlrpc_c::value::TYPE_INT;
        vector<float> data;
        for (int i = 0; i < values.size(); ++i) data.push_back((float) value_double(values[i]));
        if (integer_keys) {
            vector<string> joint_names;
            for (int i = 0; i < keys.size(); ++i) joint_names.push_back((string) value_string(keys[i]));
            this->joints->setHardness(joint_names, data, time_offset);
        } else {
            vector<int> joint_names;
            for (int i = 0; i < keys.size(); ++i) joint_names.push_back((int) value_int(keys[i]));
            this->joints->setHardness(joint_names, data, time_offset);
        }
        return;
    }
    // S:
    SensorData<float> data;
    if (paramList.size() == 0) {
        data = this->joints->getHardness();
    } else if (paramList.size() == 1) {
        // Empty check
        vector<value> keys = paramList.getArray(0);
        if (keys.empty()) return;
        bool integer_keys = keys[0].type() == xmlrpc_c::value::TYPE_INT;
        if (integer_keys) {
            vector<string> joint_names;
            for (int i = 0; i < keys.size(); ++i) joint_names.push_back((string) value_string(keys[i]));
            data = this->joints->getHardness(joint_names);
        } else {
            vector<int> joint_names;
            for (int i = 0; i < keys.size(); ++i) joint_names.push_back((int) value_int(keys[i]));
            data = this->joints->getHardness(joint_names);
        }
        return;
    } else throw girerr::error("Unknown signature for hardness function");
//    vector<value> values;
//    map<string, value> result;
//    for(int i = 0; i < data.data->size(); ++i) values.push_back(value_double(data.data->at(i)));
//    result.insert(make_pair<string, value>("data", value_array(values)));
//    result.insert(make_pair<string, value>("timestamp", value_int(data.timestamp)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RemoteJoints::PositionMethod::PositionMethod(shared_ptr<Joints> joints)
        : RemoteMethod("position", "A:,S:A,n:AA", "Position control method"), joints(joints) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RemoteJoints::PositionMethod::execute(paramList const &paramList, value *const resultP) {

}
