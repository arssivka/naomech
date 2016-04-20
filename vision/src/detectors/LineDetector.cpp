//
// Created by nikitas on 26.03.16.
//

#include <detectors/LineDetector.h>
#include "detectors/LineDetector.h"
#include "utils/CoreFuntions.h"

namespace rd {

    LineDetector::LineDetector() : BaseDetector("LineDetector") { }

    std::vector<cv::Vec4i> LineDetector::detect(const cv::Mat &preprocImage) {
        cv::Mat skeleton;

        __get_skeleton(preprocImage, skeleton); // O(n) n = h*w of img

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(skeleton, lines,
                        m_conf.HoughLines.rho, m_conf.HoughLines.theta, m_conf.HoughLines.threshold,
                        m_conf.HoughLines.min_line_length, m_conf.HoughLines.max_line_gap);

        __join_lines(lines); // O(m^2) - m = countLines

        return lines;
    }


    cv::Mat LineDetector::preproccess(const cv::Mat &image) {
        cv::Mat hsv_img, gray_img, bgr_img, buffer;
        cv::cvtColor(image, bgr_img, CV_YUV2BGR);
        cv::cvtColor(bgr_img, gray_img, CV_BGR2GRAY);
        cv::cvtColor(bgr_img, hsv_img, CV_BGR2HSV);

        const cv::Mat kernel = cv::getGaborKernel(cv::Size(3, 3), 10.0, 4.1, M_PI, 0.0);
        cv::Mat filtred_image, thresh_filt_image;
        cv::filter2D(bgr_img, filtred_image, -1, kernel);
        cv::cvtColor(filtred_image, buffer, CV_BGR2Lab);

        const cv::Scalar min_color(m_conf.Preproc.ColorThresh.min_1,
                                   m_conf.Preproc.ColorThresh.min_2,
                                   m_conf.Preproc.ColorThresh.min_3);
        const cv::Scalar max_color(m_conf.Preproc.ColorThresh.max_1,
                                   m_conf.Preproc.ColorThresh.max_2,
                                   m_conf.Preproc.ColorThresh.max_3);
        cv::inRange(buffer, min_color, max_color, thresh_filt_image);

        cv::adaptiveThreshold(gray_img, gray_img, 255,
                              CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,
                              m_conf.Preproc.kernel_size, 0.0);

        cv::Mat colorImg = gray_img.clone();

        for (int r = 0; r < colorImg.rows; r++) {
            for (int c = 0; c < colorImg.cols; c++) {
                if (thresh_filt_image.at<uchar>(r, c) != 0) {
                    cv::floodFill(colorImg, cv::Point(c, r), 0);
                    cv::floodFill(thresh_filt_image, cv::Point(c, r), 0);
                }
            }
        }
#ifdef IMSHOW_RESULT
        cv::imshow("adaptive thresh", gray_img);
        cv::imshow("filt", thresh_filt_image * 255);
        cv::imshow("filt2", filtred_image);
#endif
        const cv::Mat result_matrix = gray_img - colorImg;

        return result_matrix;
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

    void LineDetector::load(const boost::property_tree::ptree &config) {
        const boost::property_tree::ptree line_config = config.get_child(detectorName());

        m_conf.HoughLines.max_line_gap = line_config.get<double>("HoughLines.max_line_gap");
        m_conf.HoughLines.min_line_length = line_config.get<double>("HoughLines.min_line_length");
        m_conf.HoughLines.rho = line_config.get<double>("HoughLines.rho");
        m_conf.HoughLines.theta = line_config.get<double>("HoughLines.theta");
        m_conf.HoughLines.threshold = line_config.get<int>("HoughLines.threshold");
        m_conf.LineEqualPredicate.angle_eps = line_config.get<float>("LineEqualPredicate.angle_eps");
        m_conf.LineEqualPredicate.error_px = line_config.get<int>("LineEqualPredicate.error_px");
        m_conf.Preproc.kernel_size = line_config.get<int>("Preproc.kernel_size");
        m_conf.Preproc.min_thresh = line_config.get<int>("Preproc.min_thresh");
        m_conf.Preproc.ColorThresh.min_1 = line_config.get<uchar>("Preproc.ColorThresh.min_1");
        m_conf.Preproc.ColorThresh.min_2 = line_config.get<uchar>("Preproc.ColorThresh.min_2");
        m_conf.Preproc.ColorThresh.min_3 = line_config.get<uchar>("Preproc.ColorThresh.min_3");
        m_conf.Preproc.ColorThresh.max_1 = line_config.get<uchar>("Preproc.ColorThresh.max_1");
        m_conf.Preproc.ColorThresh.max_2 = line_config.get<uchar>("Preproc.ColorThresh.max_2");
        m_conf.Preproc.ColorThresh.max_3 = line_config.get<uchar>("Preproc.ColorThresh.max_3");
    }


    boost::property_tree::ptree LineDetector::get_params() {
        boost::property_tree::ptree line_config, ptree;

        line_config.put("HoughLines.max_line_gap", m_conf.HoughLines.max_line_gap);
        line_config.put("HoughLines.min_line_length", m_conf.HoughLines.min_line_length);
        line_config.put("HoughLines.rho", m_conf.HoughLines.rho);
        line_config.put("HoughLines.theta", m_conf.HoughLines.theta);
        line_config.put("HoughLines.threshold", m_conf.HoughLines.threshold);
        line_config.put("LineEqualPredicate.angle_eps", m_conf.LineEqualPredicate.angle_eps);
        line_config.put("LineEqualPredicate.error_px", m_conf.LineEqualPredicate.error_px);
        line_config.put("Preproc.kernel_size", m_conf.Preproc.kernel_size);
        line_config.put("Preproc.min_thresh", m_conf.Preproc.min_thresh);
        line_config.put("Preproc.ColorThresh.min_1", m_conf.Preproc.ColorThresh.min_1);
        line_config.put("Preproc.ColorThresh.min_2", m_conf.Preproc.ColorThresh.min_2);
        line_config.put("Preproc.ColorThresh.min_3", m_conf.Preproc.ColorThresh.min_3);
        line_config.put("Preproc.ColorThresh.max_1", m_conf.Preproc.ColorThresh.max_1);
        line_config.put("Preproc.ColorThresh.max_2", m_conf.Preproc.ColorThresh.max_2);
        line_config.put("Preproc.ColorThresh.max_3", m_conf.Preproc.ColorThresh.max_3);

        ptree.put_child(detectorName(), line_config);
        return ptree;
    }


}