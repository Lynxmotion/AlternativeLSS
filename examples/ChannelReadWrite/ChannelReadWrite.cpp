#include <platform/posix/LssPosixChannel.h>
#include <LssCommunication.h>
#include <LssCommon.h>

#include <cstdio>
#include <iostream>

#include <unistd.h>

/// Repeat the update 4 times a second
#define UPDATE_DELAY			20

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LssPosixChannel channel;


int main() {
    unsigned long now=0;
    int success = 0, failed = 0;
    short servos[] = { 17, 18, 19 };

    // put your setup code here, to run once:
    channel.begin("/dev/ttyUSB0", 115200);

    // disable motion controller on servos, we are going to burst updates so it only gets in the way
    channel.transmit(LynxPacket(254, LssMotionControl, 0));
    channel.transmit(LynxPacket(254, LssFilterPoleCount, 3));

    unsigned long long _quitting_time = millis() + 300000;
    while(millis() < _quitting_time) {

        channel.send({
            LynxPacket(19, LssPosition|LssQuery|LssDegrees),
            LynxPacket(18, LssPosition|LssQuery|LssDegrees),
            LynxPacket(17, LssPosition|LssQuery|LssDegrees)
        })
            .then( [](const LssTransaction& tx) {
                auto packets = tx.packets();
                for(auto p: packets) {
                    //printf("tx returned %d\n", val);
                    if(p.hasValue) {
                        p.id += 10;
                        p.value *= -1;
                        switch (p.id) {
                            case 27: p.value += -400; break;   // shoulder roll
                            case 28: p.value += -50; break;   // shoulder pitch
                            case 29: p.value += 200; break;   // elbow
                        }
                        channel.send({LynxPacket(p.id, LssAction | LssPosition | LssDegrees, p.value)});
                    }
                }
            })
            .otherwise( [](const LssTransaction& tx) {
                switch(tx.state) {
                    case LssTransaction::Expired: printf("expired tx %ld\n", tx.txn); break;
                }
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
