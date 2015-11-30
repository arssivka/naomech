//
// Created by arssivka on 11/9/15.
//

#include <rd/network/RPCModule.h>

using namespace boost;
using namespace std;

rd::RPCModule::RPCModule(const std::string &name)
        : name(name) { }

////////////////////////////////////////////////////////////////////////////////

void rd::RPCModule::setName(const std::string &name) {
    this->name = name;
}

////////////////////////////////////////////////////////////////////////////////

const std::string &rd::RPCModule::getName() const {
    return this->name;
}

////////////////////////////////////////////////////////////////////////////////

void rd::RPCModule::addMethod(boost::shared_ptr<RPCMethod> func,
                              const bool change_name) {
    if (change_name) func->setName(this->name + "." + func->getName());
    this->func_container.push_back(func);
}

////////////////////////////////////////////////////////////////////////////////

bool rd::RPCModule::deleteMethod(const std::string &name) {
    vector<shared_ptr<rd::RPCMethod> >::iterator it;
    for (it = this->func_container.begin();
         it != this->func_container.end(); ++it) {
        if (name == it->get()->getName()) break;
    }

    if (it != this->func_container.end()) {
        this->func_container.erase(it);
        return true;
    } else {
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////

const vector<shared_ptr<rd::RPCMethod> > &rd::RPCModule::getMethods() const {
    return this->func_container;
}

////////////////////////////////////////////////////////////////////////////////

rd::RPCModule::~RPCModule() { }
