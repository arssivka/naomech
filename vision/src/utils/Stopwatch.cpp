//
// Created by nikitas on 21.03.16.
//

#include "utils/Stopwatch.h"

namespace utils {
    Stopwatch::Stopwatch() : m_clock(0), m_mean(0.0), m_count(0) { }

    void Stopwatch::start() { m_clock = std::clock(); }

    double Stopwatch::lap() {
        const double diff = __diff_time();
        m_mean = (m_mean * m_count + diff) / (++m_count);
        return diff;
    }

    double Stopwatch::__diff_time() const {
        const double diff = std::clock() - m_clock;
        return 1000.0 * (diff / CLOCKS_PER_SEC);
    }


    double Stopwatch::mean() const { return m_mean; }
}