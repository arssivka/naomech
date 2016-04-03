//
// Created by nikitas on 20.03.16.
//

#include "detectors/BaseDetector.h"

namespace rd {

    BaseDetector::BaseDetector(const std::string &detector_name) :
            m_detector_name(detector_name) { }


    const std::string &BaseDetector::detectorName() {
        return m_detector_name;
    }

}

