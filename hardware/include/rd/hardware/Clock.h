//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_CLOCK_H
#define NAOMECH_CLOCK_H

/*!        \defgroup clock Clock
           \ingroup hardware
 */

#include <alproxies/dcmproxy.h>

namespace rd {
///@{
/*!
       \brief This is the class, responsible for
       recieving system time on the nao
     */
    class Clock : boost::noncopyable {
    public:

        /*!
           \brief Constructor for creating the object of the Clock class
           \param Broker that allows to connect to the Nao's modules.
         */
        Clock(boost::shared_ptr<AL::ALBroker> broker);

        /*!
           \brief Returns the current timestamp
           \param offset Offset for timestamp
           \return timestamp
         */
        int getTime(int offset = 0) const;
///@}
    private:
        boost::shared_ptr<AL::DCMProxy> dcm;
    };
}


#endif //NAOMECH_CLOCK_H
