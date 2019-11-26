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

}

void CompliantJoint::debugPrint()
{
}
