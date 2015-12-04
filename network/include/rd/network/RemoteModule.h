//
// Created by arssivka on 11/9/15.
//

#ifndef NAOMECH_REMOTEMODULE_H
#define NAOMECH_REMOTEMODULE_H

#include <string>
#include <vector>
#include "RemoteMethod.h"
#include <boost/smart_ptr/shared_ptr.hpp>

namespace rd {
    class RemoteModule {
    public:
        RemoteModule();

        RemoteModule(const std::string &name);

        void setName(const std::string &name);

        const std::string &getName() const;

        void addMethod(boost::shared_ptr<RemoteMethod> func,
                       const bool change_name = true);

        bool deleteMethod(const std::string &name);

        const std::vector<boost::shared_ptr<RemoteMethod> > &getMethods() const;

        virtual ~RemoteModule();

    private:
        std::string name;
        std::vector<boost::shared_ptr<RemoteMethod> > func_container;

    };
}


#endif //NAOMECH_REMOTEMODULE_H
