//
// Created by arssivka on 11/26/15.
//

#include <RD/network/RPCMethod.h>

RD::RPCMethod::RPCMethod(std::string name, std::string sig, std::string help)
        : name(name) {
    this->_signature = sig;
    this->_help = help;
}

////////////////////////////////////////////////////////////////////////////////

void RD::RPCMethod::setName(const std::string &name) {
    this->name = name;
}

////////////////////////////////////////////////////////////////////////////////

const std::string &RD::RPCMethod::getName() const {
    return name;
}
