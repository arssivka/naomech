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


    void BaseDetector::BaseConf::save(const std::string &path) {
        boost::property_tree::write_xml(path, ptree);
    }


    void BaseDetector::BaseConf::load(const std::string &path) {
        boost::property_tree::read_xml(path, ptree);
    }


}

