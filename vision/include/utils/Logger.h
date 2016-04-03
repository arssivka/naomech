//
// Created by nikitas on 22.03.16.
//

#ifndef NAOMECH_LOGGER_H
#define NAOMECH_LOGGER_H

#include <iostream>
#include <string>
#include <ctime>

namespace utils {

    class Logger {
    public:
        Logger(const std::string &promt = std::string(), bool time_print = true,
               std::ostream &stream = std::clog) : m_stream(stream.rdbuf()),
                                                   m_promt(promt), m_time_print(time_print) { }

        class LoggerBuffer {
        public:
            LoggerBuffer(std::ostream &stream) : m_stream(stream) { }

            template<typename Type>
            LoggerBuffer &operator<<(const Type &type) {
                m_stream << ' ' << type;
                return *this;
            }

        private:
            std::ostream &m_stream;
        };

        template<typename Type>
        LoggerBuffer operator<<(const Type &type) {
            if (m_latch) m_latch = false;
            else m_stream << std::endl;

            if (m_time_print) m_stream << __get_time();
            return m_stream << m_promt << ' ' << type;
        }

    private:
        const char *__get_time() {
            static const char fmt[] = "[ %H:%M:%S ] ";
            static char str_time[sizeof(fmt)];
            const time_t cur_time = std::time(0);
            std::strftime(str_time, sizeof(str_time), fmt, std::gmtime(&cur_time));
            return str_time;
        }

        std::ostream m_stream;
        std::string m_promt;
        bool m_time_print;
        static bool m_latch;
    };

}


#endif //NAOMECH_LOGGER_H
