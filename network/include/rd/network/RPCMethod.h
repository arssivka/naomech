//
// Created by arssivka on 11/26/15.
//

#ifndef NAOMECH_RPCMETHOD_H
#define NAOMECH_RPCMETHOD_H

#include <xmlrpc-c/registry.hpp>

namespace rd {
    class RPCMethod : public xmlrpc_c::method {
    public:
        RPCMethod(std::string name, std::string sig, std::string help);

        void setName(const std::string &name);

        const std::string& getName() const;

        virtual ~RPCMethod();

    private:
        std::string name;

    };
}


#endif //NAOMECH_RPCMETHOD_H
