//
// Created by nikitas on 4/1/16.
//

#ifndef NAOMECH_BALLDETECTOR_H
#define NAOMECH_BALLDETECTOR_H

#include "BaseDetector.h"

namespace rd {

    class BallDetector : public BaseDetector {
    public:
        BallDetector();

        cv::Mat preproccess(const cv::Mat &image);

        cv::Rect detect(const cv::Mat &image);

        struct configuration {

            struct {
                uchar min_1, max_1;
                uchar min_2, max_2;
                uchar min_3, max_3;
            } ColorThresh;

            struct {
                uchar min_1, max_1;
                uchar min_2, max_2;
                uchar min_3, max_3;
            } GaborThresh;

            int median_blur_size;
        };

        void load(const boost::property_tree::ptree &ball_config);

        boost::property_tree::ptree get_params();

        configuration m_conf;

    private:

    };

}

#endif //NAOMECH_BALLDETECTOR_H
