//
// Created by nikitas on 27.03.16.
//

#ifndef NAOMECH_COREFUNTIONS_H
#define NAOMECH_COREFUNTIONS_H

#include "opencv2/core/core.hpp"

namespace utils {

    template<class _Tp, int m, int n>
    inline
    float norm(const cv::Matx<_Tp, m, n> &M) {
        float sum = 0.0f;
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                sum += M(i, j) * M(i, j);
            }
        }
        return std::sqrt(sum);
    }


    template<class _Tp>
    inline
    cv::Vec<_Tp, 2> getVector(const cv::Vec4i &segment) {
        return cv::Vec<_Tp, 2>(segment(2) - segment(0), segment(3) - segment(1));
    }


    inline float getAngle(const cv::Vec2f &vec1, const cv::Vec2f &vec2) {
        const float cosAlpha = vec1.dot(vec2) / (utils::norm(vec1) * utils::norm(vec2));
        return std::acos(cosAlpha);
    }


    inline float getAltitude(const cv::Point &a, const cv::Point &b, const cv::Point &c) {
        const float dividend = std::abs((b.y - c.y) * a.x + (c.x - b.x) * a.y +
                                        (b.x * c.y - c.x * b.y));
        const float sum = (c.x - b.x) * (c.x - b.x) + (b.y - c.y) * (b.y - c.y);
        const float divider = std::sqrt(sum);
        return dividend / divider;
    }

    inline
    bool cmp(const cv::Point &p1, const cv::Point &p2) {
        return cv::norm(p1) < cv::norm(p2);
    }

}


#endif //NAOMECH_COREFUNTIONS_H
