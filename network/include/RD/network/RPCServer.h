//
// Created by arssivka on 11/9/15.
//

#ifndef NAOMECH_RPCSERVER_H
#define NAOMECH_RPCSERVER_H


#include <boost/shared_ptr.hpp>
#include <RD/network/RPCModule.h>

namespace RD {
    class RPCServer {
    public:
        RPCServer(int port);

        int getPort() const;

        void setPort(int port);

        void addModule(boost::shared_ptr<RPCModule> module);

        bool removeModule(std::string name);

        const std::vector<boost::shared_ptr<RPCModule> > &getModules() const;

        void addMethod(boost::shared_ptr<RPCMethod> func);

        bool deleteMethod(const std::string &name);

        const std::vector<boost::shared_ptr<RPCMethod> > &getMethods() const;

        void run();

    private:
        int port;
        std::vector<boost::shared_ptr<RPCModule> > modules_container;
        std::vector<boost::shared_ptr<RPCMethod> > func_container;
    };
}


#endif //NAOMECH_RPCSERVER_H
