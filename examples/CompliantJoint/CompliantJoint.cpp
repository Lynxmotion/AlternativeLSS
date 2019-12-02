//
// Created by guru on 11/24/19.
//

#include "CompliantJoint.h"


CompliantJoint::CompliantJoint(short joint_lssId, std::string _name)
    : joint(joint_lssId), name(_name), current(0), targetUpdated(false), limp(true)
{

}

void CompliantJoint::update()
{
    if(!limp) {
        int delta = position.delta();
        if(current > 300) {
            int new_pos = 0.5 * (position.current + position.target);
            moveTo(new_pos);
        }
    }
}

void CompliantJoint::debugPrint()
{
}
