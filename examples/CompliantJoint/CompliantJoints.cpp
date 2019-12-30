#include <LssCommunication.h>
#include <LssChannel.h>
#include <LssCommon.h>
#include <CompliantJoint.h>

#include <cstdio>
#include <iostream>

#include <unistd.h>
#include <climits>


#if defined(ARDUINO)
#include <Arduino.h>
#endif


// allow the use of the Exponential Moving Average Filter
#define USE_FPC

// the default LED color when joint is not in compliance mode
const short  LssLedDefault = LssBlue;

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LssChannel channel;


CompliantJoint joints[] = {
#if 0
        CompliantJoint(11, "FL.R"),
        CompliantJoint(12, "FL.P"),
        CompliantJoint(13, "KL"),
        CompliantJoint(14, "HL.P"),
        CompliantJoint(15, "HL.R"),
        CompliantJoint(16, "HL.Y"),
#endif

        CompliantJoint(17, "SL.R"),
        CompliantJoint(18, "SL.P"),
        CompliantJoint(19, "EL"),
#if 0
        CompliantJoint(21, "FR.R"),
        CompliantJoint(22, "FR.P"),
        CompliantJoint(23, "KR"),
        CompliantJoint(24, "HR.P"),
        CompliantJoint(25, "HR.R"),
        CompliantJoint(26, "HR.Y"),
#endif
        CompliantJoint(27, "SH.R"),
        CompliantJoint(28, "SH.P"),
        CompliantJoint(29, "ER")
};


int jdelay = 0;
MovingAverage<int> qtime(2), utime(2);
std::vector< Aggregate<unsigned long> > pkttimings;
std::shared_ptr<LssTransaction> queryTx;
int success = 0, failed = 0, consecutiveFailures=0;
bool ready = true;
unsigned long long _next_update = 0;
unsigned long next_print_stats = 10000;


bool learn_range = false;       // just position read-back to learn servo safety range
bool learn_model = false;       // beginnings of capturing input/feedback data for training

const int LEARN_MODEL_SPEED[] = {15, 50, 100};

#if !defined(ARDUINO)
std::string serial_devname;
#endif

void updateJoints(unsigned long now) {
    printf("%6dms + %dms", qtime.average(), utime.average());

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
            if(jdelay >0) {
                jdelay--;
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
            j.update(now);
            //printf("%4d° %4dΔ%-4d   %4dmA   ",
            //       (int)j.position.current(), j.position.current().velocity(), j.fudge.average(), (int)j.current
#if 0
            printf(" || %s  %-15s", j.name.c_str(), CompliantJoint::stateName(j.state));
            printf("%4d°  %4dF %4dΔ %4dΔ %4dΔ   %4dmA E%-4d Δ%-4dΔ%-4dΔ%-4d  ",
                   (int)j.position.current(), j.fudge.value(),
                   j.position.current().velocity(), j.position.current().acceleration(), j.position.current().jerk(),
                   j.current.current().value(), j.current.current().value() + j.currentBias, j.current.current().velocity(), j.current.current().acceleration(), j.current.current().jerk()
            );
#endif
        }
    }

    if(advance_delay) {
        jdelay = 100;
    }
    if(learn_model && advance_joint) {
        // end of movement
        for(int i=0; i < 3; i++) {
            joints[i].velocity.current += LEARN_MODEL_SPEED[i];
        }
    }
    printf("\n");
}


void print_stats() {
    printf("sent %d messages (%4.2f msgs/sec)\n", success, success/30.0);
    if(failed>0)
        printf("   %d failures\n", failed);
    LssChannelDriver::Statistics statistics = channel.driver().statistics;
    printf("%ld bytes sent (%4.2fbps), %ld bytes received (%4.2fbps)\n",
           statistics.bytes_sent, statistics.bytes_sent/30.0*9,
           statistics.bytes_received, statistics.bytes_received/30.0*9);

    int n=0;
    unsigned long dx=0;
    Aggregate<unsigned long> totals;
    printf("packet timings:\n");
    for(auto& m: pkttimings) {
        printf("  %d: %5ld | %5ld | %5ld\n", n++, m.minimum, m.average(), m.maximum);
	totals.add( m.average() - dx );
	dx = m.average();
    }
    printf("      ---------------------\n");
    printf("      %5ld | %5ld | %5ld\n", totals.minimum, totals.average(), totals.maximum);
}


#if defined(ARDUINO)
HardwareSerial mySerial2(2);
#endif

void setup() {


    // open FTDI channel directly (do lsusb to get ftdi:<vendor>:<product>:<A,B,C,D>
    // where some ftdi chips have multiple serial ports and require and index A,B,C or D.
#if defined(ARDUINO)
    Serial.begin(115200);
    
    // heltec
    Serial.println("Compliant Joint Test\n");
    delay(500);
    mySerial2.begin(230400,SERIAL_8N1,13,5);
    channel.begin(mySerial2, 230400);
#else
    // open a ttyUSB device
    //channel.begin("/dev/ttyUSB0", 115200);
    //
    // or open an FTDI channel directly (unload the ftdi_sio driver)
    //channel.begin("ftdi:0x403:0x6001:*", 115200);
    //
    channel.begin(serial_devname.c_str(), 230400);
#endif

#if 0
    // library doesnt support RESET command yet, so we'll just send as text (and give servos time to complete reset)
    channel.transmit("#254RS\r");   usleep(1500000);
#endif

    // disable motion controller on servos, we are going to burst updates so it only gets in the way
    channel.transmit(LynxPacket(254, LssMotionControl, 0));
#ifdef USE_FPC
    channel.transmit(LynxPacket(254, LssFilterPoleCount, 3));
#endif
    channel.transmit(LynxPacket(254, LssLEDColor, LssLedDefault));
    channel.transmit(LynxPacket(254, LssAngularStiffness, -3));
    channel.transmit(LynxPacket(254, LssAngularHoldingStiffness, -3));
    channel.transmit(LynxPacket(254, LssMaxDuty, 300));

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
        //joints[0].moveTo(-165);
        //joints[1].moveTo(463);
        //joints[2].moveTo(310);
    }
}

void loop()
{
#if defined(ARDUINO)
    channel.update();
#endif

    if(millis() > next_print_stats) {
        print_stats();
        next_print_stats = millis() + 15000;
    }

    // only continue if any current transaction has succeeeded
    // todo: we need a ROS like loop
    if(!ready)
        return;
    ready = false;

    if(!queryTx) {
        std::vector<LynxPacket> queries;
        for (auto j: joints) {
            queries.emplace_back(j.joint, LssCurrent | LssQuery);
            queries.emplace_back(j.joint, LssPosition | LssQuery | LssDegrees);
        }
        queryTx = std::make_shared<LssTransaction>(queries.begin(), queries.end(), 20000);
    }

    auto qstart = micros();
    channel.send(queryTx)
        .then( [qstart](const LssTransaction& tx) {
            auto packets = tx.packets();
            auto ustart = micros();
            qtime.add(ustart - qstart);

            if(pkttimings.size() < packets.size())
                pkttimings.resize(packets.size());

            // update joint measurements
            int n=0;
            unsigned long long ts = 0;
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

                if(ts == 0) ts = p.microstamp;
                pkttimings[n++].add(p.microstamp - ts);
            }

            // update joints
            updateJoints(ustart/1000);

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
                        updates.emplace_back(j.joint, LssLEDColor, LssLedDefault);
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
                    //printf("%s CPR %d\n", j.name.c_str(), j.cpr);
                    // fill the CPR
                    //for(int i=1; i<j.cpr; i++)
                    //    updates.emplace_back(j.joint, LssPosition | LssAction | LssDegrees, j.position.target);
                }
#endif

                if(j.mmd.changed(false)) {
                    updates.emplace_back(j.joint, LssAction | LssMaxDuty, j.mmd.target());
                    j.mmd.current(j.mmd.target());
                }

                switch (j.state) {
                    case CompliantJoint::Limp:
                        updates.emplace_back(j.joint, LssLimp | LssAction);
                        break;
                    case CompliantJoint::PositiveCompliance:
                    case CompliantJoint::NegativeCompliance:
                        updates.emplace_back(j.joint, LssPosition | LssAction | LssDegrees, j.position.target());
                        break;

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

                    case CompliantJoint::ComplianceLimp:
                        updates.emplace_back(j.joint, LssLimp | LssAction);
                        break;
                        
                    default:
                        break;
                }
                j.position.changed(false);

            }
            channel.send(updates.begin(), updates.end()).regardless([qstart, ustart](const LssTransaction& tx2) {
                auto now = micros();
                utime.add(now - ustart);
                ready = true;
                _next_update = now + 5000;
                success++;
                consecutiveFailures=0;
            });
        })
        .otherwise([](const LssTransaction& tx) {
            // print the state of packets in the transaction
            printf("expired: \n");
            char s[32];
            unsigned long long st = 0;
            for(auto& p: tx.packets()) {
                if(st ==0) st = p.microstamp;
                p.serialize(s);
                if(p.command & LssAction)
                    printf("   %5lld: > %s\n", p.microstamp - st, s);
                else if(p.command & LssQuery)
                    printf("   %5lld: %c %s\n", (p.microstamp>0) ? p.microstamp - st : 0, p.hasValue ? '<': '!', s);
            }

            ready = true;
            _next_update = micros() + 20000;
            failed++;
            consecutiveFailures++;
        });
}

#if !defined(ARDUINO)
int main(int argc, char *argv[]) {
    if(argc >1) {
	    serial_devname = argv[1];
	    printf("using port %s\n", serial_devname.c_str());
    }

    setup();

    unsigned long long _quitting_time = millis() + 300000;
    while(millis() < _quitting_time && consecutiveFailures < 25) {
        loop();

        while(!ready || micros() < _next_update)
            usleep(500);
    }

    // turn off the servos
    channel.transmit(LynxPacket(254, LssLEDColor, LssLedOff));
    channel.transmit(LynxPacket(254,LssLimp));

    print_stats();

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
#endif


