//
// Created by nikitas on 26.03.16.
//

#ifndef NAOMECH_LINEDETECTOR_H
#define NAOMECH_LINEDETECTOR_H

#include "BaseDetector.h"

namespace rd {

    class LineDetector : public BaseDetector {
    public:

        struct configuration {
            configuration();

            void save(const std::string &path = "line_detector.conf");
            static configuration load(const std::string &path = "line_detector.conf");

            struct HoughLines {
                double rho;
                double theta;
                double min_line_length;
                double max_line_gap;
                int threshold;
            } HoughLines;

            struct Preproc {
                int min_thresh;
                int kernel_size;
            } Preproc;

            struct LineEqualPredicate {
                bool operator()(const cv::Vec4i &line1, const cv::Vec4i &line2);

                float angle_eps;
                int error_px;
            } LineEqualPredicate;
        };

        LineDetector();

        LineDetector(const configuration &conf);

        cv::Mat preproccess(const cv::Mat &image);

        std::vector<cv::Vec4i> detect(const cv::Mat &preprocImage);

    private:
        void __get_skeleton(const cv::Mat &img, cv::Mat &result);
        void __join_lines(std::vector<cv::Vec4i> &lines);

        configuration m_conf;
    };


}


#endif //NAOMECH_LINEDETECTOR_H
