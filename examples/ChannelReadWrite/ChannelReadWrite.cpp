#include <LssChannel.h>
#include <LssCommunication.h>
#include <LssCommon.h>
#include <analytics/MovingAverage.h>

#include <cstdio>
#include <iostream>

#include <unistd.h>



/// Repeat the update 4 times a second
#define UPDATE_DELAY			12

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LssChannel channel;

short servos[] = { 17, 18, 19 };

// mirrored servos on humanoid are always 10 IDs higher than cloned servo, so 27, 28, and 29
// but we may need to correct for some offsets in tenths of degrees
short mirror_servo_offsets[] = { -400, -50, 200 };



int main() {
    unsigned long now=0;
    int success = 0, failed = 0;
    MovingAverage<unsigned long long> avgtime(40);

    // put your setup code here, to run once:
    channel.begin("/dev/ttyUSB0", 115200);

    // disable motion controller on servos, we are going to burst updates so it only gets in the way
    channel.transmit(LynxPacket(254, LssMotionControl, 0));
    channel.transmit(LynxPacket(254, LssFilterPoleCount, 3));
    channel.transmit(LynxPacket(254, LssAngularStiffness, 4));
    channel.transmit(LynxPacket(254, LssAngularHoldingStiffness, 4));

    unsigned long long _quitting_time = millis() + 300000;
    while(millis() < _quitting_time) {

        channel.send({
            LynxPacket(servos[2], LssPosition|LssQuery|LssDegrees),
            LynxPacket(servos[1], LssPosition|LssQuery|LssDegrees),
            LynxPacket(servos[0], LssPosition|LssQuery|LssDegrees)
        })
            .then( [&success,&avgtime](const LssTransaction& tx) {
                auto packets = tx.packets();
                std::vector<LynxPacket> updates;
                int n=0;

                success += packets.size();
                avgtime.add(tx.ttc);
                printf("tx-time %ldms  (%lld avg)\n", tx.ttc, avgtime.average());

                // modify packets and resend to mirrored servos
                for(auto p: packets) {
                    if(p.hasValue) {
                        p.id += 10;     // mirrored servos on humanoid are always 10 IDs higher
                        p.value *= -1;  // mirror degree value for other arm
                        p.value += mirror_servo_offsets[n];
                        updates.emplace_back(p.id, LssAction | LssPosition | LssDegrees, p.value);
                    }
                    n++;
                }

                // send pack of updates
                channel.send(updates.begin(), updates.end()).then(
                        [&success,&avgtime](const LssTransaction& tx) {
                            success += tx.packets().size();
                        });
            })
            .otherwise( [&failed](const LssTransaction& tx) {
                failed++;
                switch(tx.state) {
                    case LssTransaction::Expired: printf("expired tx %ld\n", tx.txn); break;
                }
            });

        usleep(UPDATE_DELAY * 1000);
    }

#if 0
    printf("sent %d messages (%4.2f msgs/sec)\n", success, success/30.0);
    if(failed>0)
        printf("   %d failures", failed);
    printf("%ld bytes sent (%4.2fbps), %ld bytes received (%4.2fbps)\n",
            channel.bytes_sent, channel.bytes_sent/30.0*9, channel.bytes_received, channel.bytes_received/30.0*9);

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
