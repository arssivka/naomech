//
// Created by nikitas on 4/1/16.
//

#ifndef NAOMECH_BALLDETECTOR_H
#define NAOMECH_BALLDETECTOR_H

#include "BaseDetector.h"

namespace rd {

    class BallDetector : public BaseDetector {
    public:
        struct configuration {
            configuration();

            void save(const std::string &path);

            static configuration load(const std::string &path);

            struct {
                uchar min_1, max_1;
                uchar min_2, max_2;
                uchar min_3, max_3;
            } ColorThresh;

            int median_blur_size;
        };

        cv::Mat preproccess(const cv::Mat &image);

        BallDetector();

        BallDetector(const configuration &conf);

        cv::Rect detect(const cv::Mat &image);

    private:

        configuration m_conf;
    };

}

#endif //NAOMECH_BALLDETECTOR_H
