//
// Created by guru on 11/24/19.
//

#pragma once

#include <platform/posix/LssPosixChannel.h>
#include <LssCommunication.h>
#include <LssCommon.h>
#include <MovingAverage.h>

#include <limits>
#include <functional>

//#include <Eigen/Dense>
#include <map>

//#include "kalman.h"

/* NOTES

 * First steps: get the measurements read
 * Can we generate a sequence that learns the gravity amount?
 * Maybe we should be tracking in 3D with 1-3 servos  (but that means current, position, target have to be arrays too)
 * should we be measuring in torque, or moments?
 * Keep this algorithm microprocessor compatible and we can run on a 32bit LSS bus device that becomes the servo master and does all the realtime servo control with compliance.

   PLAN

 * write a routine to record the limits of a joint system - easy min/max problem
 * write a routine to sequence through the joint space at different speeds (including idle) while recording current to file
 * train a NN to learn the joint space and expected current

 * using the NN, we estimate what the current should be.
 * possibly we may need to use velocity to look-ahead on position before computing delta (?)
 * compute difference between current and estimated movement current, output external force current, turn this into force or torque

 * implement compliance routine

 */

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

class Range {
public:
    int current;
    int target;
    int min;
    int max;

    inline Range() : current(0), target(0), min(std::numeric_limits<int>::min()), max(std::numeric_limits<int>::max()) {}

    int delta() const { return target - current; }

    inline void limits(int _min, int _max) { min = _min; max = _max; }
};

//using namespace Eigen;

class CompliantJoint {
public:
    typedef enum { Disabled, Limp, Holding, Moving, PositiveCompliance, NegativeCompliance, ComplianceLimp  } JointState;

    using StateTransition = std::pair<JointState, JointState>;
    using StateTransitionAction = JointState(JointState from, JointState to);
    using StateTransitionActions = std::map< StateTransition, std::function< StateTransitionAction > > ;


    short joint;                  // joint LSS bus IDs
    std::string name;

    JointState state, prevState;
    unsigned long stateChanged;
    StateTransitionActions transitionActions;

    Parameter<int, Derivitive<int> > current;                  // in mA
    int currentLimit;

    // bias adjusts for gravity or other known external forces and may be +/-
    Derivitive<int> currentPositiveBias, currentNegativeBias;
    int currentBias;

    int cpr;
    int cpr_changed;

    Parameter<int, Derivitive<int> > position;                 // tenths of degrees
    //int lastPosition;
    Derivitive<int> fudge;
    Range velocity;

    //bool targetUpdated;

    int compliantUntil;              // millis timestamp when we switched to compliance

//    MatrixXd gravity;             // vector for gravity
//    MatrixXd externalForce;       // estimated/deduced vector of force we are experiencing

    double mass;                  // mass of the joint and any other joints connected to this joint --- but movable joints mean different moment forces so this should be just the mass of this joint section
    double massPayload;           // a temporary mass the robot is carrying or in any way burdened

    // moving average or IIR on current?

    CompliantJoint(short joint_lssId, std::string _name);

    void enable(bool _enable);

    void update(unsigned long tsnow=0);

    bool isCompliant(unsigned long tsnow=0) const {
        if(state == PositiveCompliance || state == NegativeCompliance|| state == ComplianceLimp)
            return true;
        if(tsnow==0) tsnow = millis();
        return compliantUntil > tsnow;
    }

    inline bool isEnabled() const { return state != Disabled; }

    inline bool isLimp() const { return state == Limp; }

    CompliantJoint& moveTo(int _position, bool _transition=true);

    Parameter<int> mmd;                  // in mA

    inline void CPR(int _cpr) {
        if(cpr != _cpr) {
            cpr = _cpr;
            cpr_changed = true;
        }
    }

    //int measuredVelocity() const { return position.current - lastPosition; }

    void transitionTo(JointState new_state);

    static const char* stateName(JointState state);
};

