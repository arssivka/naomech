//
// Created by nikitas on 26.03.16.
//

#include "detectors/LineDetector.h"
#include "utils/CoreFuntions.h"

namespace rd {

    LineDetector::LineDetector() : BaseDetector("LineDetector") { }

    LineDetector::LineDetector(const configuration &conf) :
            BaseDetector("LineDetector"), m_conf(conf) { }

    std::vector<cv::Vec4i> LineDetector::detect(const cv::Mat &preprocImage) {
        cv::Mat skeleton;

        __get_skeleton(preprocImage, skeleton); // O(n) n = h*w of img
        std::cout << m_conf.HoughLines.rho << m_conf.HoughLines.theta << m_conf.HoughLines.threshold <<
        m_conf.HoughLines.min_line_length << m_conf.HoughLines.max_line_gap;
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(skeleton, lines,
                        m_conf.HoughLines.rho, m_conf.HoughLines.theta, m_conf.HoughLines.threshold,
                        m_conf.HoughLines.min_line_length, m_conf.HoughLines.max_line_gap);

        __join_lines(lines); // O(m^2) - m = countLines

        return lines;
    }


    cv::Mat LineDetector::preproccess(const cv::Mat &image) {
        cv::Mat preprocImage(image.rows, image.cols, CV_8UC1), bgrImg;
        cv::cvtColor(image, bgrImg, CV_YUV2BGR);
        cv::cvtColor(bgrImg, preprocImage, CV_BGR2GRAY);


        cv::Mat threshImage;
        cv::threshold(preprocImage, threshImage,
                      m_conf.Preproc.min_thresh,
                      255,
                      CV_THRESH_BINARY);

        const cv::Matx<uchar, 3, 3> k(0, 0, 0, 0, 1, 0, 0, 0, 0);
        cv::adaptiveThreshold(preprocImage, preprocImage, 255,
                              CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,
                              m_conf.Preproc.kernel_size, 0.0);
        cv::morphologyEx(preprocImage, preprocImage, CV_MOP_OPEN, k);

        cv::Mat colorImg = preprocImage.clone();

        for (int r = 0; r < colorImg.rows; r++) {
            for (int c = 0; c < colorImg.cols; c++) {
                if (threshImage.at<uchar>(r, c) != 0) {
                    cv::floodFill(colorImg, cv::Point(c, r), 0);
                    cv::floodFill(threshImage, cv::Point(c, r), 0);
                }
            }
        }

        return preprocImage - colorImg;
    }


    void LineDetector::__join_lines(std::vector<cv::Vec4i> &lines) {
        std::vector<int> clusters;

        const int num_clusters = cv::partition(lines, clusters, m_conf.LineEqualPredicate);

        std::vector<cv::Vec4i> joinLines((std::size_t) num_clusters);

        for (std::size_t i = 0; i < lines.size(); ++i)
            joinLines[clusters[i]] += lines[i];

        lines = joinLines;
    }

    void LineDetector::__get_skeleton(const cv::Mat &img, cv::Mat &result) {
        result = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

        //scan x
        const uchar white = 255;
        for (int r = 0; r < img.rows; r++) {
            for (int c = 0, e; c < img.cols; c++) {
                uchar px = img.at<uchar>(r, c);
                if (px != white) continue;
                e = c;
                while (px == white && e < img.cols)
                    px = img.at<uchar>(r, e++);
                int dx = e - c;
                if (dx > 1 && dx < 30)
                    result.at<uchar>(r, c + dx / 2) = 255;
                c = e;
            }
        }

        //scan y
        for (int c = 0; c < img.cols; c++) {
            for (int r = 0, e; r < img.rows; r++) {
                uchar px = img.at<uchar>(r, c);
                if (px != white) continue;
                e = r;
                while (px == white && e < img.rows)
                    px = img.at<uchar>(e++, c);
                int dx = e - r;
                if (dx > 1 && dx < 30)
                    result.at<uchar>(r + dx / 2, c) = 255;
                r = e;
            }
        }
    }

    LineDetector::configuration::configuration() {
        HoughLines.rho = 0.5;
        HoughLines.theta = M_PI / 180.0;
        HoughLines.min_line_length = 6.5;
        HoughLines.max_line_gap = 0.0;
        HoughLines.threshold = 16;

        Preproc.kernel_size = 3;
        Preproc.min_thresh = 180;

        LineEqualPredicate.angle_eps = 0.039;
        LineEqualPredicate.error_px = 7;
    }

    bool LineDetector::configuration::LineEqualPredicate::operator()
            (const cv::Vec4i &line1, const cv::Vec4i &line2) {
        using namespace utils;

        const cv::Point a(line1(0), line1(1));
        const cv::Point b(line1(2), line1(3));
        const cv::Point c(line2(2), line2(3));

        const bool parallel = getAngle(getVector<float>(line1),
                                       getVector<float>(line2)) < angle_eps;
        const bool pointInLineC = std::abs(getAltitude(c, a, b)) < error_px;

        return parallel && pointInLineC;
    }

    void LineDetector::configuration::save(const std::string &path) {
        boost::property_tree::ptree conf;
        conf.put("LineDetector.HoughLines.max_line_gap", HoughLines.max_line_gap);
        conf.put("LineDetector.HoughLines.min_line_length", HoughLines.min_line_length);
        conf.put("LineDetector.HoughLines.rho", HoughLines.rho);
        conf.put("LineDetector.HoughLines.theta", HoughLines.theta);
        conf.put("LineDetector.HoughLines.threshold", HoughLines.threshold);
        conf.put("LineDetector.LineEqualPredicate.angle_eps", LineEqualPredicate.angle_eps);
        conf.put("LineDetector.LineEqualPredicate.error_px", LineEqualPredicate.error_px);
        conf.put("LineDetector.Preproc.kernel_size", Preproc.kernel_size);
        conf.put("LineDetector.Preproc.min_thresh", Preproc.min_thresh);
        boost::property_tree::write_xml(path, conf);
    }

    LineDetector::configuration LineDetector::configuration::load(const std::string &path) {
        boost::property_tree::ptree conf;
        boost::property_tree::read_xml(path, conf);
        configuration c;

        c.HoughLines.max_line_gap = conf.get<double>("LineDetector.HoughLines.max_line_gap");
        c.HoughLines.min_line_length = conf.get<double>("LineDetector.HoughLines.min_line_length");
        c.HoughLines.rho = conf.get<double>("LineDetector.HoughLines.rho");
        c.HoughLines.theta = conf.get<double>("LineDetector.HoughLines.theta");
        c.HoughLines.threshold = conf.get<int>("LineDetector.HoughLines.threshold");
        c.LineEqualPredicate.angle_eps = conf.get<float>("LineDetector.LineEqualPredicate.angle_eps");
        c.LineEqualPredicate.error_px = conf.get<int>("LineDetector.LineEqualPredicate.error_px");
        c.Preproc.kernel_size = conf.get<int>("LineDetector.Preproc.kernel_size");
        c.Preproc.min_thresh = conf.get<int>("LineDetector.Preproc.min_thresh");

        return c;
    }
}