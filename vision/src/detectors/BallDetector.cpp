//
// Created by nikitas on 4/1/16.
//


#include <detectors/BallDetector.h>

namespace rd {


    BallDetector::BallDetector() : BaseDetector("BallDetector") { }

    BallDetector::BallDetector(const configuration &conf) :
            BaseDetector("BallDetector"), m_conf(conf) { }

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

        const cv::Scalar maxColor(m_conf.ColorThresh.min_1,
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
        ColorThresh.max_3 = 167;
    }

    void BallDetector::configuration::save(const std::string &path) {
        boost::property_tree::ptree conf;
        conf.put("BallDetector.median_blur_size", median_blur_size);

        conf.put("BallDetector.ColorThresh.min_1", ColorThresh.min_1);
        conf.put("BallDetector.ColorThresh.min_2", ColorThresh.min_2);
        conf.put("BallDetector.ColorThresh.min_3", ColorThresh.min_3);

        conf.put("BallDetector.ColorThresh.max_1", ColorThresh.min_1);
        conf.put("BallDetector.ColorThresh.max_2", ColorThresh.min_2);
        conf.put("BallDetector.ColorThresh.max_3", ColorThresh.min_3);

        boost::property_tree::write_xml(path, conf);
    }

    BallDetector::configuration BallDetector::configuration::load(const std::string &path) {
        boost::property_tree::ptree conf;
        boost::property_tree::read_xml(path, conf);
        configuration c;

        c.median_blur_size = conf.get<int>("BallDetector.median_blur_size");
        c.ColorThresh.min_1 = conf.get<uchar>("BallDetector.ColorThresh.min_1");
        c.ColorThresh.min_2 = conf.get<uchar>("BallDetector.ColorThresh.min_2");
        c.ColorThresh.min_3 = conf.get<uchar>("BallDetector.ColorThresh.min_3");
        c.ColorThresh.max_1 = conf.get<uchar>("BallDetector.ColorThresh.min_1");
        c.ColorThresh.max_2 = conf.get<uchar>("BallDetector.ColorThresh.min_2");
        c.ColorThresh.max_3 = conf.get<uchar>("BallDetector.ColorThresh.min_3");

        return c;
    }


}