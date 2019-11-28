#include <LssServo.h>
#include <LssCommunication.h>
#include <LssCommon.h>

#include <cstdio>
#include <iostream>

#include <unistd.h>

// the ID of the connected servo
#define SERVO_ID			19

/// Repeat the update 4 times a second
#define UPDATE_DELAY			10

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LynxChannel channel;

// instantiate a servo
LynxServo servo(SERVO_ID);

int main() {
    unsigned long now=0;
    int success = 0, failed = 0;

    // put your setup code here, to run once:
    channel.begin("/dev/ttyUSB0", 115200);

    unsigned long long _quitting_time = millis() + 30000;
    while(millis() < _quitting_time) {
        // request a number of parameters from the servo
        channel.read({ servo }, LssPosition | LssVoltage | LssCurrent | LssTemperature)
                .then( [&success](const MaskSet& set) -> void {
                    // when the async read is complete, the telemetry of the servo will be
                    // uddated, so we can just output those values.
                    std::cout
                            << "Servo " << servo.id << ":  tx:" << set.txn  << ":  Position " << (servo.position / 10.0) << "deg"
                            << "   Voltage " << (servo.voltage / 10.0) << "mV   Current " << servo.current
                            << "mA   Temperature " << (servo.temperature / 10.0) << "°C" << std::endl;
                    success++;
                })
                .otherwise( [&failed](const MaskSet& set) -> void {
                    // the request failed (possibly only partiall)
                    std::cout << (set.isUnresponsive() ? "Timeout" : "Partial Response") << std::endl;
                    failed++;
                });

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
    return 0;
}
