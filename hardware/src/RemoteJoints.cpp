//
// Created by arssivka on 11/27/15.
//

#include "rd/hardware/RemoteJoints.h"

using namespace xmlrpc_c;
using namespace boost;
using namespace std;
using namespace rd;

RemoteJoints::RemoteJoints(shared_ptr<Joints> &joints)
        : RPCModule("joints") {
    this->addMethod(shared_ptr<RPCMethod>(new KeysMethod(joints)));
    this->addMethod(shared_ptr<RPCMethod>(new HardnessMethod(joints)));
    this->addMethod(shared_ptr<RPCMethod>(new PositionMethod(joints)));
}

////////////////////////////////////////////////////////////////////////////////

RemoteJoints::KeysMethod::KeysMethod(
        shared_ptr<Joints> &joints)
        : RPCMethod("keys", "A:", "Return array of joint names") {
    const vector<string> &keys = joints->getInputKeys();
    vector<value> values;
    for (vector<string>::const_iterator it = keys.begin();
         it != keys.end(); ++it) {
        values.push_back(value_string(*it));
    }
    this->keys = value_array(values);
}

////////////////////////////////////////////////////////////////////////////////

void RemoteJoints::KeysMethod::execute(
        paramList const &paramList, value *const resultP) {
    *resultP = this->keys;
}

////////////////////////////////////////////////////////////////////////////////

RemoteJoints::HardnessMethod::HardnessMethod(
        shared_ptr<Joints> &joints)
        : RPCMethod("hardness", "A:,A:A,:AA,:d", "Stifness control method"),
          joints(joints) { }

////////////////////////////////////////////////////////////////////////////////

void RemoteJoints::HardnessMethod::execute(
        paramList const &paramList, value *const resultP) {

}

////////////////////////////////////////////////////////////////////////////////

RemoteJoints::PositionMethod::PositionMethod(
        shared_ptr<Joints> &joints)
        : RPCMethod("position", "A:,A:A,:AA", "Position control method"),
          joints(joints) { }

////////////////////////////////////////////////////////////////////////////////

void RemoteJoints::PositionMethod::execute(
        paramList const &paramList, value *const resultP) {

}
