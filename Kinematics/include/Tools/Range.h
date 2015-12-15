#pragma once

template<class T = float>
class Range {
public:

    T min, max;

    Range() { min = max = T(); }

    Range(T minmax) { min = max = minmax; }

    Range(T min, T max) {
        Range::min = min;
        Range::max = max;
    }

    Range<T> &add(T t) {
        if (min > t)
            min = t;
        if (max < t)
            max = t;
        return *this;
    }

    Range<T> &add(const Range<T> &r) {
        add(r.min);
        add(r.max);
        return *this;
    }

    bool isInside(T t) const {
        return min <= max ? t >= min && t <= max : t >= min || t <= max;
    }

    T limit(T t) const {
        return t < min ? min : t > max ? max : t;
    } //sets a limit for a Range

    Range<T> limit(const Range<T> &r) const {
        return Range<T>(limit(r.min), limit(r.max));
    } //sets the limit of a Range

    T getSize() const { return max - min; }

    T getCenter() const { return (max + min) / 2; }

    bool operator==(const Range<T> &r) const {
        return min == r.min && max == r.max;
    }

    bool operator<(const Range<T> &r) const { return max < r.min; }

    bool operator>(const Range<T> &r) const { return min > r.max; }

    bool meets(const Range<T> &r) const { return max == r.min; }

    bool metBy(const Range<T> &r) const { return min == r.max; }

    bool overlaps(const Range<T> &r) const {
        return min < r.min && max < r.max && max > r.min;
    }

    bool overlappedBy(const Range<T> &r) const {
        return min > r.min && max > r.max && min < r.max;
    }

    bool starts(const Range<T> &r) const { return min == r.min && max < r.max; }

    bool startedBy(const Range<T> &r) const {
        return min == r.min && max > r.max;
    }

    bool finishes(const Range<T> &r) const {
        return max == r.max && min > r.min;
    }

    bool finishedBy(const Range<T> &r) const {
        return max == r.max && min < r.min;
    }

    bool during(const Range<T> &r) const { return min > r.min && max < r.max; }

    bool contains(const Range<T> &r) const {
        return min < r.min && max > r.max;
    }
};
