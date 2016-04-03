//
// Created by nikitas on 26.03.16.
//

#ifndef NAOMECH_LINEDETECTORMANAGER_H
#define NAOMECH_LINEDETECTORMANAGER_H

#include "detectors/LineDetector.h"
#include "utils/Stopwatch.h"

#include "BaseManager.h"

namespace manager {

    class LineDetectorManager : public BaseManager {
    public:
        LineDetectorManager(const std::vector<std::string> &abs_paths);

    protected:
        void __main(const cv::Mat &image);

        void __save();

        void __load();

    private:
        void __to_conf();

        void __from_conf();

        rd::LineDetector::configuration m_conf;
        utils::Logger m_logger;
        utils::Stopwatch m_stopwatch;
        static const std::string win_name;
    };

}


#endif //NAOMECH_LINEDETECTORMANAGER_H
