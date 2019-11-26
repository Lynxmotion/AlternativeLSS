#include <platform/posix/LssPosixChannel.h>
#include <LssCommunication.h>
#include <LssCommon.h>

#include <cstdio>
#include <iostream>

#include <unistd.h>

#include "CompliantJoint.h"


/// Repeat the update 4 times a second
#define UPDATE_DELAY			25

int delay = 0;

bool learn_range = false;
bool learn_model = true;

const int LEARN_MODEL_SPEED[] = {15, 50, 100};

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LssPosixChannel channel;


CompliantJoint joints[] = {
        CompliantJoint(17, "SH.R"),
        CompliantJoint(18, "SH.P"),
        CompliantJoint(19, "ELB"),
};


void updateJoints() {
    bool advance_joint = true;
    bool advance_delay = false;
    for(auto& j : joints) {
        if(learn_range) {
            if(j.position.max == INT_MAX || j.position.current > j.position.max)
                j.position.max = j.position.current;
            else if(j.position.min == INT_MIN || j.position.current < j.position.min)
                j.position.min = j.position.current;
        }

        //joint.debugPrint();
        if(learn_range) {
            // print position extents
            printf("%s[%-4d,%4d]   ",
                   j.name.c_str(), j.position.min, j.position.max
            );
        } else if(learn_model) {
            if(delay >0) {
                delay--;
                advance_joint = false;
            } else if(advance_joint) {
                if(j.position.current + j.velocity.current > j.position.max) {
                    // at end of joint
                    j.moveTo(j.position.min);
                    //printf("[%s=>%d]\n", j.name.c_str(), j.position.min);
                    advance_delay = advance_joint = true;
                } else {
                    // normal move
                    j.moveTo(j.position.current + j.velocity.current);
                    advance_joint = false;
                }
            }

            // print current at
            printf("%d,%d,%d,%d, ",
                   j.position.current, j.position.target, j.measuredVelocity(), j.current
            );
        } else {
            // standard mode
            j.update();
            printf("%s %-4d° Δ%-4d  %-4dmA   ",
                   j.name.c_str(), j.position.current, j.position.delta(), j.current
            );
        }
    }

    if(advance_delay) {
        delay = 100;
    }
    if(learn_model && advance_joint) {
        // end of movement
        for(int i=0; i < 3; i++) {
            joints[i].velocity.current += LEARN_MODEL_SPEED[i];
        }
    }
    printf("\n");
}


int main() {
    unsigned long now=0;
    int success = 0, failed = 0;

    // put your setup code here, to run once:
    channel.begin("/dev/ttyUSB1", 115200);

    // disable motion controller on servos, we are going to burst updates so it only gets in the way
    channel.transmit(LynxPacket(254, LssMotionControl, 0));
    channel.transmit(LynxPacket(254, LssFilterPoleCount, 3));
    channel.transmit(LynxPacket(254, LssLEDColor, 3));
    channel.transmit(LynxPacket(254, LssLimp));

    joints[0].position.limits(-765, 765);
    joints[1].position.limits(-800, 900);
    joints[2].position.limits(-390, 1000);

    if(learn_model) {
        joints[0].velocity.current = LEARN_MODEL_SPEED[0];
        joints[1].velocity.current = LEARN_MODEL_SPEED[1];
        joints[2].velocity.current = LEARN_MODEL_SPEED[2];
        joints[0].moveTo(-50);
        joints[1].moveTo(-800);
        joints[2].moveTo(400);
    } else if(!learn_range) {
        joints[0].moveTo(-165);
        joints[1].moveTo(463);
        joints[2].moveTo(310);
    }

    unsigned long long _quitting_time = millis() + 300000;
    while(millis() < _quitting_time) {

        channel.send({
                         LynxPacket(17, LssCurrent|LssQuery),
                         LynxPacket(17, LssPosition|LssQuery|LssDegrees),
                         LynxPacket(18, LssCurrent|LssQuery),
                         LynxPacket(18, LssPosition|LssQuery|LssDegrees),
                         LynxPacket(19, LssCurrent|LssQuery),
                         LynxPacket(19, LssPosition|LssQuery|LssDegrees)
                 })
                .then( [&success](const LssTransaction& tx) {
                    auto packets = tx.packets();
                    // update joint measurements
                    for(auto p: packets) {
                        if(p.command & LssQuery && p.hasValue) {
                            if (p.command & LssCurrent) {
                                switch (p.id) {
                                    case 17: joints[0].current = p.value; break;
                                    case 18: joints[1].current = p.value; break;
                                    case 19: joints[2].current = p.value; break;
                                }
                            } else if (p.command & LssPosition) {
                                switch (p.id) {
                                    case 17: joints[0].updatePosition(p.value); break;
                                    case 18: joints[1].updatePosition(p.value); break;
                                    case 19: joints[2].updatePosition(p.value); break;
                                }
                            }
                        }
                    }

                    // update joints
                    updateJoints();

                    std::vector<LynxPacket> updates;
                    for(auto& j: joints) {
                        if(!j.limp) {
                            updates.emplace_back(LynxPacket(j.joint, LssPosition|LssAction|LssDegrees, j.position.target));
                            j.targetUpdated = false;
                            j.limp = false;
                        }
                    }
                    channel.send(updates.begin(), updates.end());
                })
                .otherwise([&failed](const LssTransaction& tx) {
                    printf("tx timed out\n");
                });

        usleep(UPDATE_DELAY * 1000);
    }

    printf("sent %d messages (%4.2f msgs/sec)\n", success, success/30.0);
    if(failed>0)
        printf("   %d failures", failed);
    printf("%ld bytes sent (%4.2fbps), %ld bytes received (%4.2fbps)\n",
            channel.bytes_sent, channel.bytes_sent/30.0*9, channel.bytes_received, channel.bytes_received/30.0*9);

#if 0
    // print some detailed stats
    LynxServo::Statistics stats = LynxServo::globalStatistics();
    printf("packet statistics:\n");
    printf("   transmits: %ld    retransmits: %d\n", stats.packet.transmits, stats.packet.retransmits);
    printf("     queries: %d      responses: %d\n", stats.packet.queries, stats.packet.received);

    printf("transaction statistics:\n");
    printf("   complete: %ld    partials: %d    timeouts: %d\n", stats.transaction.complete, stats.transaction.partials, stats.transaction.timeouts);
    printf("   responseTime: %ld | %ld | %ld\n", stats.transaction.responseTime.minimum, stats.transaction.responseTime.average(), stats.transaction.responseTime.maximum);
    printf("   completionTime: %ld | %ld | %ld\n", stats.transaction.completionTime.minimum, stats.transaction.completionTime.average(), stats.transaction.completionTime.maximum);
#endif
    return 0;
}
