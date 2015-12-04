//
// Created by arssivka on 11/9/15.
//

#ifndef NAOMECH_RPCSERVER_H
#define NAOMECH_RPCSERVER_H


#include <boost/shared_ptr.hpp>
#include "RemoteModule.h"

namespace rd {
    class RPCServer {
    public:
        RPCServer(int port);

        int getPort() const;

        void setPort(int port);

        void addModule(boost::shared_ptr<RemoteModule> module);

        bool removeModule(std::string name);

        const std::vector<boost::shared_ptr<RemoteModule> > &getModules() const;

        void addMethod(boost::shared_ptr<RemoteMethod> func);

        bool deleteMethod(const std::string &name);

        const std::vector<boost::shared_ptr<RemoteMethod> > &getMethods() const;

        void run();

        virtual ~RPCServer();

    private:
        int port;
        std::vector<boost::shared_ptr<RemoteModule> > modules_container;
        std::vector<boost::shared_ptr<RemoteMethod> > func_container;
    };
}


#endif //NAOMECH_RPCSERVER_H
