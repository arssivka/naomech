//
// Created by arssivka on 11/9/15.
//

#include <rd/network/RPCServer.h>
#include <xmlrpc-c/server_abyss.hpp>
#include <boost/make_shared.hpp>

using namespace rd;
using namespace std;
using namespace boost;


RPCServer::RPCServer(int port) : port(port) { }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int RPCServer::getPort() const {
    return port;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RPCServer::setPort(int port) {
    this->port = port;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RPCServer::addModule(shared_ptr<RemoteModule> module) {
    this->modules_container.push_back(module);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RPCServer::removeModule(std::string name) {
    vector<boost::shared_ptr<RemoteModule> >::iterator it;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<shared_ptr<RemoteModule> > &RPCServer::getModules() const {
    return this->modules_container;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RPCServer::addMethod(shared_ptr<RemoteMethod> func) {
    this->func_container.push_back(func);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RPCServer::deleteMethod(const string &name) {
    vector<shared_ptr<RemoteMethod> >::iterator it;
    for (it = this->func_container.begin(); it != this->func_container.end(); ++it) {
        if (name == it->get()->getName()) break;
    }

    if (it != this->func_container.end()) {
        this->func_container.erase(it);
        return true;
    } else {
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const vector<shared_ptr<RemoteMethod> > &RPCServer::getMethods() const {
    return this->func_container;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RPCServer::run() {
    vector<shared_ptr<RemoteMethod> > methods(this->func_container);
    {
        vector<shared_ptr<RemoteModule> >::iterator it;
        for (it = this->modules_container.begin(); it != this->modules_container.end(); ++it) {
            const vector<shared_ptr<RemoteMethod> > &m = (*it)->getMethods();
            methods.insert(methods.end(), m.begin(), m.end());
        }
    }

    xmlrpc_c::registry reg;
    {
        vector<shared_ptr<RemoteMethod> >::iterator it;
        for (it = methods.begin(); it != methods.end(); ++it) {
            RemoteMethod *method = it->get();
            reg.addMethod(method->getName(), method);
        }
    }

    xmlrpc_c::serverAbyss srv(xmlrpc_c::serverAbyss::constrOpt()
                                      .registryP(&reg)
                                      .portNumber(this->port));
    srv.run();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RPCServer::~RPCServer() { }
