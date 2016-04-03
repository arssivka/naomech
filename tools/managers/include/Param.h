//
// Created by nikitas on 26.03.16.
//

#ifndef NAOMECH_PARAM_H
#define NAOMECH_PARAM_H

#include <string>

namespace utils {

    struct Param {
        Param() { }

        Param(const std::string &tbname, double val = 0,
              double div = 1.0, int min = 0, int max = 100) :
                tbname(tbname), val(static_cast<int>(val * div)),
                min(min), max(max), div(div) { }

        template<typename ParamType>
        ParamType get() {
            while (min > val) val++;
            return val / div;
        }

        template<typename ParamType>
        void operator=(ParamType new_val) {
            val = static_cast<int>(new_val *
                                   static_cast<ParamType>(div));
        }

        int *pval() { return &val; }


        std::string tbname;
        int max;
    private:
        int min;
        double div;
        int val;
    };

}


#endif //NAOMECH_PARAM_H
