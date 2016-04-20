//
// Created by nikitas on 26.03.16.
//

#ifndef NAOMECH_LINEDETECTOR_H
#define NAOMECH_LINEDETECTOR_H

#include "BaseDetector.h"

namespace rd {

    class LineDetector : public BaseDetector {
    public:

        LineDetector();

        cv::Mat preproccess(const cv::Mat &image);

        std::vector<cv::Vec4i> detect(const cv::Mat &preprocImage);

        struct configuration {

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
                int kernel_size_2;
                struct {
                    uchar min_1, max_1;
                    uchar min_2, max_2;
                    uchar min_3, max_3;
                } ColorThresh;
            } Preproc;

            struct LineEqualPredicate {
                float angle_eps;
                int error_px;

                bool operator()(const cv::Vec4i &line1, const cv::Vec4i &line2);
            } LineEqualPredicate;
        };

        void load(const boost::property_tree::ptree &line_config);

        boost::property_tree::ptree get_params();

        configuration m_conf;
    private:
        void __get_skeleton(const cv::Mat &img, cv::Mat &result);

        void __join_lines(std::vector<cv::Vec4i> &lines);
    };


}


#endif //NAOMECH_LINEDETECTOR_H
