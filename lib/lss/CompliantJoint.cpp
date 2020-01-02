//
// Created by guru on 11/24/19.
//

#include "CompliantJoint.h"


CompliantJoint::CompliantJoint(short joint_lssId, std::string _name)
    : joint(joint_lssId), name(_name), state(ComplianceLimp), stateChanged(0), current(0),gravityBias(0), currentLimit(170), cpr(0), cpr_changed(false) //, targetUpdated(false)
{
    //fudge.filter(2);
}

const char* CompliantJoint::stateName(JointState state) {
    switch(state) {
        case Disabled: return "Disabled";
        case Limp: return "Limp";
        case Holding: return "Holding";
        case Moving: return "Moving";
        case PositiveCompliance: return "+Compliance";
        case NegativeCompliance: return "-Compliance";
        case ComplianceLimp: return "ComplianceLimp";
        default:
            return "?state?";
    }
}


void CompliantJoint::enable(bool _enable) {
    if(isEnabled() != _enable) {
        transitionTo(_enable ? Limp : Disabled);
    }
}

void CompliantJoint::transitionTo(JointState new_state) {
    if(new_state == state)
         return; // No change

     prevState = state;
     stateChanged = millis();

    // we get a chance to do something on the transition
    auto action = transitionActions.find( StateTransition(state, new_state) );
    if(action != transitionActions.end()) {
        // call the action
        if(action->second) {
            state = action->second(state, new_state);
            return;
        }
    }
    state = new_state;
}

CompliantJoint& CompliantJoint::moveTo(int _position, bool _transition) {
    position.target(_position);
    //targetUpdated = true;
    if(_transition)
        transitionTo( (position.current().velocity() > 10) ? Moving : Holding );
    return *this;
}

void CompliantJoint::update(unsigned long tsnow)
{
    if(state <= Limp)
        return;
    bool entering = state != prevState;
    int w;

    if(!isLimp() && tsnow > 1200) {
        int dx = position.current().velocity();
        bool pos_polarity = dx > 0;
        bool neg_polarity = dx < 0;

        currentBias = pos_polarity
                ? currentPositiveBias.value()
                : neg_polarity
                    ? currentNegativeBias.value()
                    : 0;

        auto old_currentBias = currentBias;

        if(neg_polarity && gravityBias>0)
            currentBias = gravityBias;
        else if(pos_polarity && gravityBias<0)
            currentBias -= gravityBias;

        if(name == "J14")
            printf("%s  %d |%c| %d %c => %d", old_currentBias, pos_polarity ? '+' : neg_polarity ? '-' : '-', gravityBias, currentBias);

        int unbiasedCurrent = current.current() + currentBias;
        bool limit = unbiasedCurrent > currentLimit;
        //int amps_velocity = -current.current().velocity();

        unsigned long stateTime = millis() - stateChanged;
        bool idle = position.current().velocity()==0 && !limit;


        switch(state) {
            case Holding:
                // check if we are over current limit
                mmd.target(800);
                CPR(3);
                if(limit) {
                    if(pos_polarity)
                        transitionTo(PositiveCompliance);
                    else if (neg_polarity)
                        transitionTo(NegativeCompliance);
                }
                break;

            case Moving:
                mmd.target(1023);
                CPR(3);
                //if(delta.spread()<2 && delta.average()==0 && !limit)
                if(idle)
                        transitionTo(Holding);
                break;


            // seems wierd to split the state, then switch on the same code but we still use discrete polarity state to prevent oscillation
            case NegativeCompliance:
            case PositiveCompliance:
                CPR(1);
                mmd.target(255);

                fudge = clamp(current.current().value() / 23, 0, 50);
                w = (state == NegativeCompliance)
                    ? -fudge.value()
                    : fudge.value();

                // just keep adding to target
                //if (current.current().acceleration() < -25) {
                //if (current.current().velocity() > 1000) {
                //if (fudge.velocity() > 1000) {
                if(fudge > 15 && stateTime > 60) {//} || ((position.current().acceleration() < -50) && stateTime > 200)) {
                    CPR(1); // reduce filter since we cant update FIR in limp mode
                    transitionTo(ComplianceLimp);
                } else {
                    // half way between target and current (set fudge divider to 12 and limp threshold to 30)
                    //moveTo( 0.5* ((position.target() + w) + position.current()), false); // todo: try adding measure velocity here

                    //moveTo(position.target() + w + position.current().velocity()/5, false); // todo: try adding measure velocity here
                    moveTo(position.target() + w, false); // use fudge divider of 20, limp threshold of 15
                }

                if(idle && stateTime>40)
                //if(position.current().acceleration() < -10)
                    transitionTo(Holding);

                break;

            case ComplianceLimp:
                // keep target tracking current position for when we exit compliance
                fudge.clear();
                current.current().clear();
                moveTo( position.current(), false);

                // if our average is low, and the spread of numbers is narrow, then human is holding us in stable position, retake control
                if(idle && stateTime > 200)
                    transitionTo(Holding); // was go to compliance, but what polarity?
                break;
        }

    }
}

