//
// Created by arssivka on 11/26/15.
//

#include <rd/network/RPCMethod.h>

rd::RPCMethod::RPCMethod(std::string name, std::string sig, std::string help)
        : name(name) {
    this->_signature = sig;
    this->_help = help;
}

////////////////////////////////////////////////////////////////////////////////

void rd::RPCMethod::setName(const std::string &name) {
    this->name = name;
}

////////////////////////////////////////////////////////////////////////////////

const std::string &rd::RPCMethod::getName() const {
    return name;
}

////////////////////////////////////////////////////////////////////////////////

rd::RPCMethod::~RPCMethod() { }
