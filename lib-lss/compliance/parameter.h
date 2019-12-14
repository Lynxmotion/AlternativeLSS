//
// Created by guru on 12/13/19.
//

#pragma once

#include "../analytics/numerics.h"
#include "../analytics/range.h"

template<class T, class CT=T>
class Parameter {
public:
    explicit inline Parameter(T v=0, T __min=std::numeric_limits<T>::min(), T __max=std::numeric_limits<T>::max()) : _current(v), _target(v), _min(__min), _max(__max) {}
    inline Parameter(const Parameter<T>& copy) : _current(copy._current), _target(copy._target), _min(copy._min), _max(copy._max) {}

    inline Parameter<T, CT>& operator=(const Parameter<T>& copy) {
        _current = copy.value;
        _target = copy.target;
        _min = copy.min;
        _max = copy.max;
        return *this;
    }

    inline Parameter<T, CT>& operator=(const T& v) {
        target(v);
        return *this;
    }

    inline operator const T() const { return _current; }
    inline operator T() { return _current; }

    inline Parameter<T, CT>& operator+(T v) { target(_target.value + v); return *this; }
    inline Parameter<T, CT>& operator-(T v) { target(_target.value - v); return *this; }
    inline Parameter<T, CT>& operator*(T v) { target(_target.value * v); return *this; }
    inline Parameter<T, CT>& operator/(T v) { target(_target.value / v); return *this; }
    inline Parameter<T, CT>& operator%(T v) { target(_target.value % v); return *this; }


    inline CT& current() { return _current; }
    inline const CT& current() const { return _current; }
    inline void current(T v) { _current = ::clamp(v, _min, _max); }

    inline void current(T v, unsigned long long ts) {
        // todo: we must adjust the mmeasurement based on when we read it
        _current = ::clamp(v, _min, _max);
    }

    inline CT& target() { return _target; }
    inline const CT& target() const { return _target; }
    void target(T v) {
        auto t = ::clamp(v, _min, _max);
        if(t != _target) {
            _targetChanged = true;
            _target = t;
        }
    }

    inline T delta() const { return (T)_current - (T)_target; }

    bool changed(bool new_state=true) {
        bool b = _targetChanged;
        _targetChanged = new_state;
        return b;   // return previous state
    }

    inline void limits(int __min, int __max) { _min = __min; _max = __max; }

    inline T min() const { return _min; }
    inline void min(T v) { _min = v; }

    inline T max() const { return _max; }
    inline void max(T v) { _max = v; }

    inline T clamp(T __min, T __max) const { return ::clamp(_current, __min, __max); }

private:
    CT _current;
    CT _target;
    T _min;
    T _max;
    bool _targetChanged;
};


