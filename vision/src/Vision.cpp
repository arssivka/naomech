//
// Created by nikitas on 26.03.16.
//

#include "Vision.h"

namespace rd {

    Vision::Vision(const std::string &cfg_path) : m_w(320), m_h(280), m_image(),
                                                  m_ballDetector(
                                                          BallDetector::configuration::load("ball_detector.conf")),
                                                  m_lineDetector(
                                                          LineDetector::configuration::load("line_detector.conf")) { }

    cv::Rect Vision::ballDetect() {
        const cv::Mat preprocImage = m_ballDetector.preproccess(m_image);
        return m_ballDetector.detect(preprocImage);
    }

    std::vector<cv::Vec4i> Vision::lineDetect() {
        const cv::Mat preprocImage = m_lineDetector.preproccess(m_image);
        return m_lineDetector.detect(preprocImage);
    }


    void Vision::setFrame(const cv::Mat &frame) {
        m_image = frame.clone();
    }


}
