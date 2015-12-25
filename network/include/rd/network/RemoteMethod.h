#ifndef NAOMECH_REMOTEMETHOD_H
#define NAOMECH_REMOTEMETHOD_H

/*! \defgroup network Network*/

/*!        \defgroup remote_method RemoteMethod
           \ingroup network
 */

#include <xmlrpc-c/registry.hpp>

namespace rd {
///@{
/*!
       \brief Parent class for creating remote methods.

           This class represents the way for creating
           remote methods,
           that will be stored in the remote
           module.
     */
    class RemoteMethod : public xmlrpc_c::method {
    public:
        /*!
           \brief Basic constructor.
         */
        RemoteMethod();
        /*!
           \brief Constructor for creating the remote method.

           This is the main constructor needed for creating the
           remote methods in the remote module.

           \param name name of the remote method
           \param sig signature of the method
           \param help help infromation
         */
        RemoteMethod(std::string name, std::string sig, std::string help);

        /*!
         * \brief Method for setting the name of the remote method.
         * \param name new name of the method
         */
        void setName(const std::string &name);

        /*!
         * \brief Returns the name of the remote method
         * \returns name of the remote method
         */
        const std::string& getName() const;

        /*!
         * \brief Destructor
         */
        ///@}
        virtual ~RemoteMethod();

    private:
        std::string name;

    };
}

#endif //NAOMECH_REMOTEMETHOD_H
