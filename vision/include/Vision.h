//
// Created by nikitas on 26.03.16.
//

#ifndef NAOMECH_VISION_H
#define NAOMECH_VISION_H

#include <boost/shared_ptr.hpp>

#include <detectors/BallDetector.h>
#include <detectors/LineDetector.h>

namespace rd {

    class Vision {
    public:
        Vision(const boost::property_tree::ptree &config);

        cv::Rect ballDetect();

        std::vector<cv::Vec4i> lineDetect();

        void setFrame(const cv::Mat &frame);

        BallDetector m_ballDetector;
        LineDetector m_lineDetector;
    private:
        cv::Mat m_image;
    };


}


#endif //NAOMECH_VISION_H
