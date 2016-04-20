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
        cv::Mat preprocImage, prepImage;
        cv::cvtColor(image, preprocImage, CV_YUV2BGR);

        cv::Mat afterGaborRange, gaborImage;
        cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10, 7.8, 9.4, 0);
        cv::filter2D(image, gaborImage, -1, kernel);
        const cv::Scalar minGaborColor(m_conf.GaborThresh.min_1,
                                       m_conf.GaborThresh.min_2,
                                       m_conf.GaborThresh.min_3);
        const cv::Scalar maxGaborColor(m_conf.GaborThresh.max_1,
                                       m_conf.GaborThresh.max_2,
                                       m_conf.GaborThresh.max_3);
        cv::Mat gaborInRange;
        cv::inRange(gaborImage, minGaborColor, maxGaborColor, gaborInRange);
        cv::morphologyEx(gaborInRange, afterGaborRange, CV_MOP_DILATE, cv::Mat::ones(3, 3, CV_8UC1));

        cv::Mat medianBlurFrame;
        cv::medianBlur(preprocImage, medianBlurFrame, m_conf.median_blur_size);

        cv::morphologyEx(preprocImage, preprocImage, CV_MOP_OPEN, cv::Mat::ones(3, 3, CV_8UC1));

        const cv::Scalar minColor(m_conf.ColorThresh.min_1,
                                  m_conf.ColorThresh.min_2,
                                  m_conf.ColorThresh.min_3);
        const cv::Scalar maxColor(m_conf.ColorThresh.max_1,
                                  m_conf.ColorThresh.max_2,
                                  m_conf.ColorThresh.max_3);

        cv::inRange(medianBlurFrame, minColor, maxColor, preprocImage);
//        cv::imshow("gab", afterGaborRange);
//        cv::imshow("preproc1", preprocImage);

        cv::Mat ball = preprocImage.clone();
        for (int r = 0; r < ball.rows; r++) {
            for (int c = 0; c < ball.cols; c++) {
                if (afterGaborRange.at<uchar>(r, c) != 0) {
//                    cv::floodFill(afterGaborRange, cv::Point(c, r), 0);
                    cv::floodFill(ball, cv::Point(c, r), 0);
                }
            }
        }

//        cv::imshow("preproc11", ball);
        return preprocImage - ball;
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
        m_conf.GaborThresh.min_1 = ball_config.get<uchar>("GaborThresh.min_1");
        m_conf.GaborThresh.min_2 = ball_config.get<uchar>("GaborThresh.min_2");
        m_conf.GaborThresh.min_3 = ball_config.get<uchar>("GaborThresh.min_3");
        m_conf.GaborThresh.max_1 = ball_config.get<uchar>("GaborThresh.max_1");
        m_conf.GaborThresh.max_2 = ball_config.get<uchar>("GaborThresh.max_2");
        m_conf.GaborThresh.max_3 = ball_config.get<uchar>("GaborThresh.max_3");
    }


    boost::property_tree::ptree BallDetector::get_params() {
        boost::property_tree::ptree ball_config, ptree;

        ball_config.put("median_blur_size", m_conf.median_blur_size);
        ball_config.put("ColorThresh.min_1", m_conf.ColorThresh.min_1);
        ball_config.put("ColorThresh.min_2", m_conf.ColorThresh.min_2);
        ball_config.put("ColorThresh.min_3", m_conf.ColorThresh.min_3);
        ball_config.put("ColorThresh.max_1", m_conf.ColorThresh.max_1);
        ball_config.put("ColorThresh.max_2", m_conf.ColorThresh.max_2);
        ball_config.put("ColorThresh.max_3", m_conf.ColorThresh.max_3);
        ball_config.put("GaborThresh.min_1", m_conf.GaborThresh.min_1);
        ball_config.put("GaborThresh.min_2", m_conf.GaborThresh.min_2);
        ball_config.put("GaborThresh.min_3", m_conf.GaborThresh.min_3);
        ball_config.put("GaborThresh.max_1", m_conf.GaborThresh.max_1);
        ball_config.put("GaborThresh.max_2", m_conf.GaborThresh.max_2);
        ball_config.put("GaborThresh.max_3", m_conf.GaborThresh.max_3);

        ptree.put_child(detectorName(), ball_config);
        return ptree;
    }


}