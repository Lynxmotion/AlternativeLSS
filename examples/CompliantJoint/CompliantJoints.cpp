#include <platform/posix/LssPosixChannel.h>
#include <LssCommunication.h>
#include <LssCommon.h>

#include <cstdio>
#include <iostream>

#include <unistd.h>
#include <climits>

#include "CompliantJoint.h"



// allow the use of the Exponential Moving Average Filter
#define USE_FPC

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LssPosixChannel channel;


CompliantJoint joints[] = {
        CompliantJoint(17, "SH.R"),
        CompliantJoint(18, "SH.P"),
        CompliantJoint(19, "ELB"),
};


int delay = 0;
MovingAverage<int> qtime(2), utime(2);

bool learn_range = false;       // just position read-back to learn servo safety range
bool learn_model = false;       // beginnings of capturing input/feedback data for training

const int LEARN_MODEL_SPEED[] = {15, 50, 100};


void updateJoints(unsigned long now) {
    bool advance_joint = true;
    bool advance_delay = false;
    for(auto& j : joints) {
        if(learn_range) {
            if(j.position.max() == INT_MAX || j.position.current() > j.position.max())
                j.position.max( j.position.current() );
            else if(j.position.min() == INT_MIN || j.position.current() < j.position.min())
                j.position.min( j.position.current() );
        }

        //joint.debugPrint();
        if(learn_range) {
            // print position extents
            printf("%s[%-4d,%4d]   ",
                   j.name.c_str(), j.position.min(), j.position.max()
            );
        } else if(learn_model) {
            if(delay >0) {
                delay--;
                advance_joint = false;
            } else if(advance_joint) {
                if(j.position.current() + j.velocity.current > j.position.max()) {
                    // at end of joint
                    j.moveTo(j.position.min());
                    //printf("[%s=>%d]\n", j.name.c_str(), j.position.min);
                    advance_delay = advance_joint = true;
                } else {
                    // normal move
                    j.moveTo(j.position.current() + j.velocity.current);
                    advance_joint = false;
                }
            }

            // print current at
            printf("%d,%d,%d,%d, ",
                   (int)j.position.current(), (int)j.position.target(), j.position.current().velocity(), (int)j.current
            );
        } else {
            // standard mode
            printf("%6dms || %s  %-15s", qtime.average()+utime.average(), j.name.c_str(), CompliantJoint::stateName(j.state));
            j.update(now);
            //printf("%4d° %4dΔ%-4d   %4dmA   ",
            //       (int)j.position.current(), j.position.current().velocity(), j.fudge.average(), (int)j.current
            printf("%4d°  %4dF %4dΔ %4dΔ %4dΔ   %4dmA E%-4d Δ%-4dΔ%-4dΔ%-4d  ",
                   (int)j.position.current(), j.fudge.value(),
                   j.position.current().velocity(), j.position.current().acceleration(), j.position.current().jerk(),
                   j.current.current().value(), j.current.current().value() + j.currentBias, j.current.current().velocity(), j.current.current().acceleration(), j.current.current().jerk()
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
    printf("\r");
}

#define UPDATE_DELAY			20

int main() {
    int success = 0, failed = 0;
    bool ready = true;

    // put your setup code here, to run once:
    channel.begin("/dev/ttyUSB0", 115200);

#if 0
    // library doesnt support RESET command yet, so we'll just send as text (and give servos time to complete reset)
    //channel.transmit("#254RS\r");   usleep(1500000);
#endif

    // disable motion controller on servos, we are going to burst updates so it only gets in the way
    channel.transmit(LynxPacket(254, LssMotionControl, 0));
#ifdef USE_FPC
    channel.transmit(LynxPacket(254, LssFilterPoleCount, 3));
#endif
    channel.transmit(LynxPacket(254, LssLEDColor, 3));
    //channel.transmit(LynxPacket(254, LssAngularStiffness, 4));
    //channel.transmit(LynxPacket(254, LssAngularHoldingStiffness, 12));

    /* Some joint setup
     */

    // set servo safety limits
    joints[0].position.limits(-765, 765);
    joints[1].position.limits(-800, 900);
    joints[2].position.limits(-390, 1000);

    // add a little force bias to account for gravity on the shoulder pitch
    joints[1].currentNegativeBias = -60;    // only add bias in the 'negative' direction

    // add a little Exponential Moving Average filtering to the current measurements
    joints[1].current.current().filter(1);
    joints[2].current.current().filter(1);
    joints[3].current.current().filter(1);

    if(learn_model) {
        joints[0].velocity.current = LEARN_MODEL_SPEED[0];
        joints[1].velocity.current = LEARN_MODEL_SPEED[1];
        joints[2].velocity.current = LEARN_MODEL_SPEED[2];
        joints[0].moveTo(-50);
        joints[1].moveTo(-800);
        joints[2].moveTo(400);
    } else if(!learn_range) {
        // initial position of ARM
        joints[0].moveTo(-165);
        joints[1].moveTo(463);
        joints[2].moveTo(310);
    }

    unsigned long long _quitting_time = millis() + 300000;
    while(millis() < _quitting_time) {
        ready = false;
        auto qstart = micros();
        auto _next_update = qstart + 20000;

        channel.send({
                         LynxPacket(joints[0].joint, LssCurrent|LssQuery),
                         LynxPacket(joints[0].joint, LssPosition|LssQuery|LssDegrees),
                         LynxPacket(joints[1].joint, LssCurrent|LssQuery),
                         LynxPacket(joints[1].joint, LssPosition|LssQuery|LssDegrees),
                         LynxPacket(joints[2].joint, LssCurrent|LssQuery),
                         LynxPacket(joints[2].joint, LssPosition|LssQuery|LssDegrees)
                 })
                .then( [&success, &ready, qstart](const LssTransaction& tx) {
                    auto packets = tx.packets();
                    auto now = micros();
                    qtime.add(now - qstart);
                    auto ustart = now;

                    // update joint measurements
                    for(auto p: packets) {
                        if(p.command & LssQuery && p.hasValue) {
                            for(auto& j: joints) {
                                if(j.joint == p.id) {
                                    if (p.command & LssCurrent) {
                                        j.current.current(p.value, p.microstamp); break;
                                    } else if (p.command & LssPosition) {
                                        j.position.current(p.value, p.microstamp); break;
                                    }
                                }
                            }
                        }
                    }

                    // update joints
                    updateJoints(now/1000);

                    std::vector<LynxPacket> updates;
                    for(auto& j: joints) {
                        if(!j.isEnabled())
                            continue;

                        // set color of servo to indicate joint state (Compliance, Limp or Holding/Moving)
                        switch(j.state) {
                            case CompliantJoint::NegativeCompliance:
                            case CompliantJoint::PositiveCompliance:
                                updates.emplace_back(j.joint, LssLEDColor, LssMagenta);
                                break;

                            case CompliantJoint::ComplianceLimp:
                                updates.emplace_back(j.joint, LssLEDColor, LssRed);
                                break;

                            default:
                                updates.emplace_back(j.joint, LssLEDColor, LssBlue);
                                break;
                        }

#ifdef USE_FPC
                        if(j.cpr_changed) {
                            //if(j.cpr ==0) {
                            //    updates.emplace_back(j.joint, LssMotionControl, 1);
                            //    updates.emplace_back(j.joint, LssMotionControl, 0);
                            //} else
                                updates.emplace_back(j.joint, LssFilterPoleCount, j.cpr);
                                updates.emplace_back(j.joint, LssPosition | LssAction | LssDegrees, j.position.target());
                            j.cpr_changed = false;
                            printf("%s CPR %d\n", j.name.c_str(), j.cpr);
                            // fill the CPR
                            //for(int i=1; i<j.cpr; i++)
                            //    updates.emplace_back(j.joint, LssPosition | LssAction | LssDegrees, j.position.target);
                        }
#endif

                        switch (j.state) {
                                break;
                            case CompliantJoint::PositiveCompliance:
                            case CompliantJoint::NegativeCompliance:
                                updates.emplace_back(j.joint, LssPosition | LssAction | LssDegrees, j.position.target());

                            case CompliantJoint::Holding:
                                updates.insert(updates.end(),
                                               LynxPacket(j.joint, LssPosition | LssAction | LssDegrees,
                                                          j.position.target())
                                                       .currentHaltAndLimp(j.currentLimit)
                                );
                                break;

                            case CompliantJoint::Moving:
                                updates.insert(updates.end(),
                                               LynxPacket(j.joint, LssPosition | LssAction | LssDegrees,
                                                          j.position.target())
                                );
                                break;

                            case CompliantJoint::ComplianceLimp:updates.emplace_back(j.joint, LssLimp | LssAction);
                                break;
                        }
                        j.position.changed(false);

                    }
                    channel.send(updates.begin(), updates.end()).regardless([&ready, ustart](const LssTransaction& tx2) {
                        utime.add( micros() - ustart);
                        ready = true;
                    });
                })
                .otherwise([&failed, &ready](const LssTransaction& tx) {
                    printf("tx timed out\n");
                    ready = true;
                });

        while(!ready || micros() < _next_update)
            usleep(500);
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
