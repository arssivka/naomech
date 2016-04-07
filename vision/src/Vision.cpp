//
// Created by nikitas on 26.03.16.
//

#include "Vision.h"

namespace rd {

    Vision::Vision(const boost::property_tree::ptree &config) {
        const boost::property_tree::ptree vision_config = config.get_child("Vision");
        m_lineDetector.load(vision_config);
        m_ballDetector.load(vision_config);
    }

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
