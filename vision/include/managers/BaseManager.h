//
// Created by nikitas on 24.03.16.
//

#ifndef NAOMECH_BASEMANAGER_H
#define NAOMECH_BASEMANAGER_H

#include <stdexcept>

#include <opencv2/opencv.hpp>

#include "utils/Logger.h"
#include "utils/Param.h"

namespace manager {

    class BaseManager {
    public:

        BaseManager(const std::vector<std::string> &abs_paths, const std::string &logger_promt = "BaseManager::");

        enum Key {
            Next = '0',
            Prev = '9',
            Save = 's',
            Load = 'l',
            Exit = 27
        };

        void run();

    protected:
        virtual void __init();

        virtual void __main(const cv::Mat &image);

        virtual void __exit();

        virtual void __save();

        virtual void __load();

        virtual void __switch(int);

        void __update_trackbars(const std::string &win_name);

        typedef std::map<std::string, utils::Param> ParamsMap;
        ParamsMap m_params;

    private:

        const std::vector<std::string> m_abs_paths;
        std::vector<std::string>::size_type m_cur_path;
        utils::Logger m_logger;
    };

}
#endif //NAOMECH_BASEMANAGER_H
