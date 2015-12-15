#pragma once

#include "Tools/Math/Vector2.h"
#include "Range.h"

template<class T = float>
class Boundary {
public:
    Range<T> x, y;

    Boundary() : x(0, 0), y(0, 0) { }

    Boundary(T min, T max) : x(max, min), y(max, min) { }

    Boundary(const Range<T> &x, const Range<T> &y) : x(x), y(y) { }

    void add(const Vector2 <T> &p) {
        x.add(p.x);
        y.add(p.y);
    }

    void add(const Boundary<T> &b) {
        x.add(b.x);
        y.add(b.y);
    }

    bool isInside(const Vector2 <T> &p) const {
        return x.isInside(p.x) && y.isInside(p.y);
    }

    bool isEmpty() const { return x.min > x.max || y.min > y.max; }

    void clip(Vector2 <T> &p) const {
        p.x = x.limit(p.x);
        p.y = y.limit(p.y);
    }
};
