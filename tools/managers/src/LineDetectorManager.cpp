//
// Created by nikitas on 26.03.16.
//


#include "LineDetectorManager.h"

namespace manager {

    const std::string LineDetectorManager::win_name = "LineDetectorManager";

    LineDetectorManager::LineDetectorManager(const std::vector<std::string> &abs_paths) :
            BaseManager(abs_paths, "LineDetectorManager::"), m_conf() {
        m_params["rho"] = utils::Param("Distance resolution(HL)");
        m_params["theta"] = utils::Param("Angle resolution(HL)");
        m_params["max_line_gap"] = utils::Param("Maximum allowed gap(HL)");
        m_params["min_line_length"] = utils::Param("Minimum line length(HL)");
        m_params["threshold"] = utils::Param("Accumulator minThresh(HL)");
        m_params["kernel_size"] = utils::Param("Moph Kernel Size(preproccess)");
        m_params["min_thresh"] = utils::Param("Threashold(preproccess)");
        m_params["angle_eps"] = utils::Param("Error parallel vector(joinLines)");
        m_params["error_px"] = utils::Param("Error in pixel(joinLines)");

        __from_conf();
        __update_trackbars(win_name);
    }

    void LineDetectorManager::__to_conf() {
        m_conf.HoughLines.rho = m_params["rho"].get<double>();
        m_conf.HoughLines.theta = m_params["theta"].get<double>();
        m_conf.HoughLines.max_line_gap = m_params["max_line_gap"].get<double>();
        m_conf.HoughLines.min_line_length = m_params["min_line_length"].get<double>();
        m_conf.HoughLines.threshold = m_params["threshold"].get<int>();
        m_conf.Preproc.kernel_size = m_params["kernel_size"].get<int>();
        m_conf.Preproc.min_thresh = m_params["min_thresh"].get<int>();
        m_conf.LineEqualPredicate.angle_eps = m_params["angle_eps"].get<float>();
        m_conf.LineEqualPredicate.error_px = m_params["error_px"].get<int>();
    }


    void LineDetectorManager::__from_conf() {
        m_params["rho"] = m_conf.HoughLines.rho;
        m_params["theta"] = m_conf.HoughLines.theta;
        m_params["max_line_gap"] = m_conf.HoughLines.max_line_gap;
        m_params["min_line_length"] = m_conf.HoughLines.min_line_length;
        m_params["threshold"] = m_conf.HoughLines.threshold;
        m_params["kernel_size"] = m_conf.Preproc.kernel_size;
        m_params["min_thresh"] = m_conf.Preproc.min_thresh;
        m_params["angle_eps"] = m_conf.LineEqualPredicate.angle_eps;
        m_params["error_px"] = m_conf.LineEqualPredicate.error_px;
    }


    void LineDetectorManager::__main(const cv::Mat &image) {
        __to_conf();

        rd::LineDetector lineDetector(m_conf);

        m_stopwatch.start();

        cv::Mat preprocImage = lineDetector.preproccess(image);
        cv::imshow("LineDetector::manager::preprocImage", preprocImage);
        std::vector<cv::Vec4i> lines = lineDetector.detect(preprocImage);

        m_logger << m_stopwatch.lap() << "ms, mean =" << m_stopwatch.mean();

        {
            cv::Mat linesImage = image.clone();
            std::vector<cv::Vec4i>::const_iterator it;
            for (it = lines.begin(); it != lines.end(); ++it) {
                cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
                cv::line(linesImage, cv::Point((*it)[0], (*it)[1]),
                         cv::Point((*it)[2], (*it)[3]), color, 2, 8);
            }
            cv::imshow("LineDetector::manager::linesImage", linesImage);
        }
    }


    void LineDetectorManager::__save() {
        m_logger << "try save";
        m_conf.save();
    }

    void LineDetectorManager::__load() {
        m_logger << "try load";
        m_conf = m_conf.load();
        __update_trackbars(win_name);
    }


}