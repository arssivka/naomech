//
// Created by arssivka on 11/26/15.
//

#ifndef NAOMECH_REMOTEMETHOD_H
#define NAOMECH_REMOTEMETHOD_H

#include <xmlrpc-c/registry.hpp>

namespace rd {
    class RemoteMethod : public xmlrpc_c::method {
    public:
        RemoteMethod();

        RemoteMethod(std::string name, std::string sig, std::string help);

        void setName(const std::string &name);

        const std::string& getName() const;

        virtual ~RemoteMethod();

    private:
        std::string name;

    };
}


#endif //NAOMECH_REMOTEMETHOD_H
