#ifndef NAOMECH_REMOTEMODULE_H
#define NAOMECH_REMOTEMODULE_H


/*!
    \defgroup remote_module RemoteModule
    \ingroup network
*/

#include <string>
#include <vector>
#include "RemoteMethod.h"
#include <boost/smart_ptr/shared_ptr.hpp>

namespace rd {
///@{
/*!
      \brief Parent class for creating remote modules

     This calss represents a way for creating modules,
     that can be registered in the server and store
     remote methods.
     */
    class RemoteModule {
    public:
        /*!
          \brief Basic constructor.
         */
        RemoteModule();

        /*!
           \brief Constructor for creating the remote module.

           This is the main constructor that nedded for
           creating the remote modules

           \param name name of the remote module.
         */
        RemoteModule(const std::string &name);

        /*!
           \brief Sets the name of the remote module.
           \param name new name of the module.
         */
        void setName(const std::string &name);

        /*!
         * \brief Returns name of the remote module.
         * \returns Name of the module.
         */
        const std::string &getName() const;

        /*!
           \brief Adds a new remote method to the remote module.
           \param func a shared pointer to the remote method, that
            needs to be added to the remote module
           \param change_name if true adds the name of the module to the name
           of the method
         */
        void addMethod(boost::shared_ptr<RemoteMethod> func,
                       const bool change_name = true);

        /*!
           \brief Removes the remote method from the
           module with specified name
           \param name name of the method that nedds to be
           deleted
           \returns true if method was removed succesfully, otherwise false
         */
        bool removeMethod(const std::string &name);

        /*!
           \brief Returns vector that stores all the remote methods
           of the module
           \returns vector containing remote methods
         */
        const std::vector<boost::shared_ptr<RemoteMethod> > &getMethods() const;

        /*!
         * \brief Destructor.
         */
        ///@}
        virtual ~RemoteModule();

    private:
        std::string name;
        std::vector<boost::shared_ptr<RemoteMethod> > func_container;

    };
}

#endif //NAOMECH_REMOTEMODULE_H
