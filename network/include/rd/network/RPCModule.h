//
// Created by arssivka on 11/9/15.
//

#ifndef NAOMECH_RPCMODULE_H
#define NAOMECH_RPCMODULE_H

#include <string>
#include <vector>
#include <rd/network/RPCMethod.h>
#include <boost/smart_ptr/shared_ptr.hpp>

namespace rd {
    class RPCModule {
    public:
        RPCModule(const std::string &name);

        void setName(const std::string &name);

        const std::string &getName() const;

        void addMethod(boost::shared_ptr<RPCMethod> func,
                       const bool change_name = true);

        bool deleteMethod(const std::string &name);

        const std::vector<boost::shared_ptr<RPCMethod> > &getMethods() const;

    private:
        std::string name;
        std::vector<boost::shared_ptr<RPCMethod> > func_container;

    };
}


#endif //NAOMECH_RPCMODULE_H
