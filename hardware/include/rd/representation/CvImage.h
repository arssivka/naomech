#ifndef NAOMECH_CVIMAGE_H
#define NAOMECH_CVIMAGE_H

/*! \defgroup representation Representation
    \ingroup hardware
*/

/*!
   \defgroup cvimage CvImage
   \ingroup representation
 */
///@{
#include "opencv2/core/core.hpp"

namespace rd {
    /*!
       \brief Class for managing the data of cameras in opencv format
       Stores opencv image and timestamp
     */
    class CvImage {
    public:
        /*!
           \brief Basic constructor
         */
        CvImage() {}
        /*!
           \brief Constructor for creating the CvImage object
           \param img cv::Mat image
           \param timestamp Timestamp (Thanks, Captain!)
         */
        CvImage(cv::Mat img, int timestamp): img(img), timestamp(timestamp) { }
        /*!
           \brief timestamp Timestamp. It is a public member for the sake of usability.
         */
        int timestamp;
        /*!
           \brief img cv::Mat image. It is a public member for the sake of usability.
         */
        cv::Mat img;
        ///@}
    };
}

#endif //NAOMECH_CVIMAGE_H
