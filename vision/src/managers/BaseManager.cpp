//
// Created by nikitas on 27.03.16.
//

#include "managers/BaseManager.h"

namespace manager {

    BaseManager::BaseManager(const std::vector<std::string> &abs_paths, const std::string &logger_promt)
            : m_abs_paths(abs_paths), m_cur_path(0), m_logger(logger_promt) { }

    void manager::BaseManager::run() {
        __init();
        while (true) {
            cv::Mat image;
            try {
                image = cv::imread(m_abs_paths.at(m_cur_path));
            }
            catch (std::out_of_range &) {
                image = cv::Mat::zeros(280, 320, CV_8UC3);
                m_logger << "limit paths";
            }
            catch (std::exception &error) {
                image = cv::Mat::zeros(280, 320, CV_8UC3);
                m_logger << "bad file" << m_abs_paths.at(m_cur_path);
            }

            __main(image);

            int key = cv::waitKey(50);
            switch (key) {
                case Next:
                    m_cur_path++;
                    break;
                case Prev:
                    m_cur_path--;
                    break;
                case Save:
                    __save();
                    break;
                case Load:
                    __load();
                    break;
                case Exit:
                    __exit();
                    return;
                default:
                    __switch(key);
                    break;
            }
        }
    }

    void BaseManager::__init() { }

    void BaseManager::__main(const cv::Mat &image) { }

    void BaseManager::__exit() { }

    void BaseManager::__save() { }

    void BaseManager::__load() { }

    void BaseManager::__switch(int) { }


    void BaseManager::__update_trackbars(const std::string &win_name) {
        cv::namedWindow(win_name, CV_WINDOW_AUTOSIZE);

        ParamsMap::iterator it;
        for (it = m_params.begin(); it != m_params.end(); ++it) {
            utils::Param &p = it->second;
            cv::createTrackbar(p.tbname, win_name, p.pval(), p.max);
        }
    }

}