//
// Created by guru on 11/24/19.
//

#pragma once

#include <platform/posix/LssPosixChannel.h>
#include <LssCommunication.h>
#include <LssCommon.h>

#include <Eigen/Dense>

#include "kalman.h"

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

class Range {
public:
    int current;
    int target;
    int min;
    int max;

    inline Range() : current(0), target(0), min(INT_MIN), max(INT_MAX) {}

    int delta() const { return target - current; }

    inline void limits(int _min, int _max) { min = _min; max = _max; }
};

using namespace Eigen;

class CompliantJoint {
public:
    short joint;                  // joint LSS bus IDs
    std::string name;

    int current;                  // in mA

    Range position;                 // tenths of degrees
    int lastPosition;

    Range velocity;

    bool targetUpdated;

    bool limp;                    // our behavior in limp will have to be a bit different since here all forces are purely external or gravity, there would be 0 current so its all position/target based
    bool active;                  // true if our target should be written to the servo

    MatrixXd gravity;             // vector for gravity
    MatrixXd externalForce;       // estimated/deduced vector of force we are experiencing

    double mass;                  // mass of the joint and any other joints connected to this joint --- but movable joints mean different moment forces so this should be just the mass of this joint section
    double massPayload;           // a temporary mass the robot is carrying or in any way burdened

    // moving average or IIR on current?

    CompliantJoint(short joint_lssId, std::string _name);

    void update();

    void debugPrint();

    CompliantJoint& moveTo(int _position) {
        position.target = _position;
        targetUpdated = true;
        limp = false;   // turn off limp
        return *this;
    }

    void updatePosition(int current) {
        lastPosition = position.current;
        position.current = current;
    }

    int measuredVelocity() const { return position.current - lastPosition; }
};


