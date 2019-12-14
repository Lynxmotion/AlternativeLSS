//
// Created by guru on 12/13/19.
//

#pragma once

template<class T>
T clamp(T v, T _min, T _max) {
    return (v < _min)
           ? _min
           : (v > _max)
             ? _max
             : v;
}



template<class T>
class Derivitive {
private:
    T _current;
    T _previous;
    T _previousVelocity;
    T _previousAcceleration;
    int filterPoles;

public:
    inline Derivitive<T>() : _current(0), _previous(0), _previousVelocity(0), _previousAcceleration(0), filterPoles(0) {}
    inline Derivitive<T>(T v) : _current(v), _previous(v), _previousVelocity(0), _previousAcceleration(0), filterPoles(0) {}
    inline Derivitive<T>(const Derivitive<T>& copy)
            : _current(copy._current), _previous(copy._previous), _previousVelocity(copy._previousVelocity), _previousAcceleration(copy._previousAcceleration),
              filterPoles(copy.filterPoles)
    {}

    inline Derivitive<T>& operator=(const Derivitive<T>& copy) {
        _current = copy.value;
        _previous = copy._previous;
        _previousVelocity = copy._previousVelocity;
        _previousAcceleration = copy._previousAccelleration;
        filterPoles = copy.filterPoles;
        return *this;
    }

    inline Derivitive<T>& operator=(const T& v) {
        _previousAcceleration = acceleration();
        _previousVelocity = velocity();
        _previous = _current;
        if(filterPoles>0)
            _current = (((_current * (filterPoles-1)) + v) / filterPoles);
        else
            _current = v;
        return *this;
    }

    inline operator const T() const { return _current; }
    inline operator T() { return _current; }

    inline bool operator>(T v) { return _current > v; }
    inline bool operator<(T v) { return _current < v; }
    inline bool operator>=(T v) { return _current >= v; }
    inline bool operator<=(T v) { return _current <= v; }
    inline bool operator==(T v) { return _current == v; }
    inline bool operator!=(T v) { return _current != v; }

    inline T value(T v, T _min, T _max) const { operator=( clamp(v, _min, _max) ); }
    inline T value() const { return _current; }

    inline T previous() const { return _previous; }

    inline void clear() { _current = _previous = _previousVelocity = _previousAcceleration = 0; }

    inline T velocity() const { return _current - _previous; }
    inline T acceleration() const { return velocity() - _previousVelocity; }
    inline T jerk() const { return acceleration() - _previousAcceleration; }

    inline int filter() const { return filterPoles; }
    void filter(int poles) {
        filterPoles = poles;
    }

};


