#include <LssServo.h>
#include <LssCommunication.h>
#include <LssCommon.h>

#include <cstdio>
#include <iostream>

#include <unistd.h>

/// Repeat the update 4 times a second
#define UPDATE_DELAY			10

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LynxChannel channel;

// instantiate a servo
LynxServo servos[18];

int main() {
    unsigned long now=0;
    int success = 0, failed = 0;

    // put your setup code here, to run once:
    channel.begin("/dev/ttyUSB0", 115200);

    // right side
    servos[0] = LynxServo(19);
    servos[1] = LynxServo(18);
    servos[2] = LynxServo(17);
    servos[3] = LynxServo(16);
    servos[4] = LynxServo(15);
    servos[5] = LynxServo(14);
    servos[6] = LynxServo(13);
    servos[7] = LynxServo(12);
    servos[8] = LynxServo(11);

#if 1
    // left side
    servos[9] = LynxServo(29);
    servos[10] = LynxServo(28);
    servos[11] = LynxServo(27);
    servos[12] = LynxServo(26);
    servos[13] = LynxServo(25);
    servos[14] = LynxServo(24);
    servos[15] = LynxServo(23);
    servos[16] = LynxServo(22);
    servos[17] = LynxServo(21);
#endif

    // Add one or more servos to the channel
    for(auto& s : servos)
        if(s.id >0)
            channel.add(s);

    unsigned long long _quitting_time = millis() + 30000;
    while(millis() < _quitting_time) {
        // request a number of parameters from the servo
        for(auto& servo: servos) {
            if(servo.id <=1)
                continue;

            servo.ReadAsync(LssPosition | LssVoltage | LssCurrent | LssTemperature)
                    .then([&success, &servo](const MaskSet &set) -> void {
                        // when the async read is complete, the telemetry of the servo will be
                        // uddated, so we can just output those values.
                        std::cout
                                << "Servo " << servo.id << ":  tx:" << set.txn << "  Position " << (servo.position / 10.0) << "deg"
                                << "   Voltage " << (servo.voltage / 10.0) << "mV   Current " << servo.current
                                << "mA   Temperature " << (servo.temperature / 10.0) << "°C" << std::endl;
                        success++;
                    })
                    .otherwise([&failed](const MaskSet &set) -> void {
                        // the request failed (possibly only partiall)
                        std::cout << (set.isUnresponsive() ? "Timeout" : "Partial Response") << std::endl;
                        failed++;
                    });
        }

        // must call channel update in main loop so the Servo sub-system
        // can respond to serial, async read and other events.
        channel.update();

        usleep(UPDATE_DELAY * 1000);
    }

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
    return 0;
}
