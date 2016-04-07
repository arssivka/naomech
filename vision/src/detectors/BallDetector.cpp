//
// Created by nikitas on 4/1/16.
//


#include <detectors/BallDetector.h>

namespace rd {


    BallDetector::BallDetector() : BaseDetector("BallDetector") { }

    cv::Rect BallDetector::detect(const cv::Mat &preprocImage) {
        cv::Rect ans;
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(preprocImage, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) {
            double maxArea = 0;
            int maxAreaIdx = 0;

            for (int i = 0; i < contours.size(); i++) {
                const double area = cv::contourArea(contours[i]);
                if (area > maxArea) maxArea = area, maxAreaIdx = i;
            }

            ans = cv::boundingRect(contours[maxAreaIdx]);
        }
        return ans;
    }

    cv::Mat BallDetector::preproccess(const cv::Mat &image) {
        // TODO: работает в YUV или все таки в BGR
        cv::Mat preprocImage;
        cv::cvtColor(image, preprocImage, CV_YUV2BGR);

        cv::Mat medianBlurFrame;
        cv::medianBlur(preprocImage, medianBlurFrame, m_conf.median_blur_size);

        const cv::Scalar minColor(m_conf.ColorThresh.min_1,
                                  m_conf.ColorThresh.min_2,
                                  m_conf.ColorThresh.min_3);

        const cv::Scalar maxColor(m_conf.ColorThresh.max_1,
                                  m_conf.ColorThresh.max_2,
                                  m_conf.ColorThresh.max_3);

        cv::inRange(medianBlurFrame, minColor, maxColor, preprocImage);
        return preprocImage;
    }

    BallDetector::configuration::configuration() {
        median_blur_size = 7;
        ColorThresh.min_1 = 0;
        ColorThresh.min_2 = 35;
        ColorThresh.min_3 = 167;
        ColorThresh.max_1 = 86;
        ColorThresh.max_2 = 210;
        ColorThresh.max_3 = 221;
    }

    void BallDetector::load(const boost::property_tree::ptree &config) {
        const boost::property_tree::ptree ball_config = config.get_child(detectorName());
        m_conf.median_blur_size = ball_config.get<int>("median_blur_size");
        m_conf.ColorThresh.min_1 = ball_config.get<uchar>("ColorThresh.min_1");
        m_conf.ColorThresh.min_2 = ball_config.get<uchar>("ColorThresh.min_2");
        m_conf.ColorThresh.min_3 = ball_config.get<uchar>("ColorThresh.min_3");
        m_conf.ColorThresh.max_1 = ball_config.get<uchar>("ColorThresh.max_1");
        m_conf.ColorThresh.max_2 = ball_config.get<uchar>("ColorThresh.max_2");
        m_conf.ColorThresh.max_3 = ball_config.get<uchar>("ColorThresh.max_3");
    }


}