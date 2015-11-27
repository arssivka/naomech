//
// Created by arssivka on 11/9/15.
//

#include <rd/network/RPCServer.h>
#include <xmlrpc-c/server_abyss.hpp>
#include <boost/make_shared.hpp>

using namespace std;
using namespace boost;


rd::RPCServer::RPCServer(int port) : port(port) { }

////////////////////////////////////////////////////////////////////////////////

int rd::RPCServer::getPort() const {
    return port;
}
////////////////////////////////////////////////////////////////////////////////

void rd::RPCServer::setPort(int port) {
    this->port = port;
}

////////////////////////////////////////////////////////////////////////////////

void rd::RPCServer::addModule(shared_ptr<RPCModule> module) {
    this->modules_container.push_back(module);
}

////////////////////////////////////////////////////////////////////////////////

bool rd::RPCServer::removeModule(std::string name) {
    vector<boost::shared_ptr<rd::RPCModule> >::iterator it;
    for (it = this->modules_container.begin();
         it != this->modules_container.end(); ++it) {
        if (name == it->get()->getName()) break;
    }

    if (it != this->modules_container.end()) {
        this->modules_container.erase(it);
        return true;
    } else {
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////

const vector<shared_ptr<rd::RPCModule> > &rd::RPCServer::getModules() const {
    return this->modules_container;
}

////////////////////////////////////////////////////////////////////////////////

void rd::RPCServer::addMethod(shared_ptr<RPCMethod> func) {
    this->func_container.push_back(func);
}

////////////////////////////////////////////////////////////////////////////////

bool rd::RPCServer::deleteMethod(const string &name) {
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

const vector<shared_ptr<rd::RPCMethod> > &rd::RPCServer::getMethods() const {
    return this->func_container;
}

////////////////////////////////////////////////////////////////////////////////

void rd::RPCServer::run() {
    vector<shared_ptr<RPCMethod> > methods(this->func_container);
    {
        vector<shared_ptr<rd::RPCModule> >::iterator it;
        for (it = this->modules_container.begin();
             it != this->modules_container.end(); ++it) {
            const vector<shared_ptr<rd::RPCMethod> > &m = it->get()->getMethods();
            methods.insert(methods.begin(), m.begin(), m.end());
        }
    }

    xmlrpc_c::registry reg;
    {
        vector<shared_ptr<RPCMethod> >::iterator it;
        for (it = methods.begin(); it != methods.end(); ++it) {
            RPCMethod *method = it->get();
            reg.addMethod(method->getName(), method);
        }
    }

    xmlrpc_c::serverAbyss srv(xmlrpc_c::serverAbyss::constrOpt()
                                      .registryP(&reg)
                                      .portNumber(this->port));
}
