//
// Created by nikitas on 20.03.16.
//

#ifndef LINEDETECTOR_BASEDETECTOR_H
#define LINEDETECTOR_BASEDETECTOR_H

#include <string>

namespace detector {

    class BaseDetector {
    public:
        BaseDetector(const std::string &detector_name = "");

        const std::string &detectorName();

    private:
        const std::string m_detector_name;
    };

}

#endif //LINEDETECTOR_BASEDETECTOR_H
