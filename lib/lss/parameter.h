//
// Created by guru on 12/13/19.
//

#pragma once

#include "analytics/numerics.h"
#include "analytics/range.h"

template<class T, class CT=T>
class Parameter {
public:
    explicit inline Parameter(T v=0, T ___min=std::numeric_limits<T>::min(), T ___max=std::numeric_limits<T>::max()) : _current(v), _target(v), __min(___min), __max(___max) {}
    inline Parameter(const Parameter<T>& copy) : _current(copy._current), _target(copy._target), __min(copy.__min), __max(copy.__max) {}

    inline Parameter<T, CT>& operator=(const Parameter<T>& copy) {
        _current = copy.value;
        _target = copy.target;
        __min = copy.__min;
        __max = copy.__max;
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
    inline void current(T v) { _current = ::clamp(v, __min, __max); }

    inline void current(T v, unsigned long long /*ts*/) {
        // todo: we must adjust the mmeasurement based on when we read it
        _current = ::clamp(v, __min, __max);
    }

    inline CT& target() { return _target; }
    inline const CT& target() const { return _target; }
    void target(T v) {
        auto t = ::clamp(v, __min, __max);
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

    inline void limits(int ___min, int ___max) { __min = ___min; __max = ___max; }

    inline T min() const { return __min; }
    inline void min(T v) { __min = v; }

    inline T max() const { return __max; }
    inline void max(T v) { __max = v; }

    inline T clamp(T __min, T __max) const { return ::clamp(_current, __min, __max); }

private:
    CT _current;
    CT _target;
    T __min;
    T __max;
    bool _targetChanged;
};


