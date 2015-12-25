#ifndef NAOMECH_RPCSERVER_H
#define NAOMECH_RPCSERVER_H

/*!
  \defgroup rpcsrver RPCServer
  \ingroup network
*/


#include <boost/shared_ptr.hpp>
#include "RemoteModule.h"

namespace rd {

///@{
/*!
       \brief Server, which allows you to call methods remotly

  This is the RPC server that stores remote modules with their methods and
  remote methods and when it's running allows to call them remotly.
     */
    class RPCServer {
    public:
        /*!
           \brief Constructor for creating rpc-server.
           \param port number of port for the server.
         */
        RPCServer(int port);

        /*!
           \brief Returns port of the server.
           \returns number of port of the server.
         */
        int getPort() const;

        /*!
           \brief Sets port of the server.
           \param port Number of the new port.
         */
        void setPort(int port);

        /*!
           \brief Adds a new remote module to the server.
           \param module a shared pointer to the remote module that
           needs to be added to server.
         */
        void addModule(boost::shared_ptr<RemoteModule> module);

        /*!
           \brief Removes remote module from the server.
           \param name Name of module to remove.
           \returns true if module was removed succesfully, oterwise false.
         */
        bool removeModule(std::string name);

        /*!
           \brief Returns vector that storess remote modules of the server.
           \returns vector containing remote methods
         */
        const std::vector<boost::shared_ptr<RemoteModule> > &getModules() const;

        /*!
           \brief adds remote method to the server
           \param func a shared pointer to the remote method, that nedds to be added
         */
        void addMethod(boost::shared_ptr<RemoteMethod> func);

        /*!
           \brief Removes method from the server.
           \param name name of method to remove.
           \returns true if method was removed succesfully, otherwise false.
         */
        bool removeMethod(const std::string &name);

        /*!
           \brief Returns vector that stores remote methods of the server.
           \returns vector containing remote methods.
         */
        const std::vector<boost::shared_ptr<RemoteMethod> > &getMethods() const;

        /*!
           \brief Runs the server.
         */
        void run();

        /*!
         * \brief Destructor
         */
        ///@}
        virtual ~RPCServer();

    private:
        int port;
        std::vector<boost::shared_ptr<RemoteModule> > modules_container;
        std::vector<boost::shared_ptr<RemoteMethod> > func_container;
    };
}

#endif //NAOMECH_RPCSERVER_H
