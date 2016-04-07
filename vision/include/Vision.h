//
// Created by nikitas on 26.03.16.
//

#ifndef NAOMECH_VISION_H
#define NAOMECH_VISION_H

#include <detectors/BallDetector.h>
#include <detectors/LineDetector.h>

namespace rd {

    class Vision {
    public:
        Vision(const std::string &cfg_path = "/home/nao/vision.conf");

        cv::Rect ballDetect();

        std::vector<cv::Vec4i> lineDetect();

        void setFrame(const cv::Mat &frame);

        const int m_w, m_h;

    private:
        BallDetector m_ballDetector;
        LineDetector m_lineDetector;
        cv::Mat m_image;
    };


}


#endif //NAOMECH_VISION_H
