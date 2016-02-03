#pragma once

template<class V>
inline V sgn(V a) {
    return a < 0 ? V(-1) : a > 0 ? V(1) : 0;
}

template<class V>
inline V sqr(const V &a) { return a * a; }

const float pi = 3.1415926535897932384626433832795f;
const float pi2 = 2.0f * pi;
const float pi3_2 = 1.5f * pi;
const float pi_2 = 0.5f * pi;
const float pi_180 = pi / 180.0f;
const float pi_4 = pi * 0.25f;
const float pi3_4 = pi * 0.75f;
const float _180_pi = 180.0f / pi;

template<class V>
V toDegrees(V angle) { return angle * V(_180_pi); }

template<class V>
V fromDegrees(V degrees) { return degrees * V(pi_180); }

inline float fromDegrees(int degrees) { return fromDegrees((float) degrees); }

template<class V>
inline V normalize(V data) {
    if (data >= -V(pi) && data < V(pi))
        return data;
    else {
        data = data - ((int) (data / V(pi2))) * V(pi2);
        return data >= V(pi) ? data - V(pi2) : data < -V(pi) ? data + V(pi2)
                                                             : data;
    }
}