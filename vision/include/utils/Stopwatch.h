//
// Created by nikitas on 21.03.16.
//

#ifndef NAOMECH_STOPWATCH_H
#define NAOMECH_STOPWATCH_H

#include <ctime>

namespace utils {
    class Stopwatch {
    public:
        Stopwatch();

        void start();

        double lap();

        double mean() const;

    private:
        double __diff_time() const;

        std::clock_t m_clock;
        double m_mean;
        unsigned int m_count;
    };
}


#endif //NAOMECH_STOPWATCH_H
