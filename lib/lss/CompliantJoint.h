//
// Created by guru on 11/24/19.
//

#pragma once

#include "LssCommunication.h"
#include "LssCommon.h"

#include "analytics/MovingAverage.h"
#include "analytics/numerics.h"
#include "analytics/range.h"

#include <limits>
#include <functional>

//#include <Eigen/Dense>
#include <map>

//#include "kalman.h"
#include "parameter.h"

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
    int gravityBias;

    int cpr;
    int cpr_changed;

    Parameter<int, Derivitive<int> > position;                 // tenths of degrees
    //int lastPosition;
    Derivitive<int> fudge;
    Range velocity;

    //bool targetUpdated;

    unsigned long compliantUntil;              // millis timestamp when we switched to compliance

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

    inline void limp() { state = Limp; }

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

