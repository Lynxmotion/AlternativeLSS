#include <LynxmotionLSS.h>
#include <LssCommunication.h>
#include <LssCommon.h>

#include <cstdio>
#include <iostream>

// the ID of the connected servo
#define SERVO_ID			19

/// Repeat the update 4 times a second
#define UPDATE_DELAY			250

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LynxChannel channel;

// instantiate a servo
LynxServo servo(SERVO_ID);

unsigned long nextUpdate = 0;

int main() {
    unsigned long now=0;
    int success = 0, failed = 0;

    // put your setup code here, to run once:
    std::cout << "opening servo channel" << std::endl;
    channel.begin("/dev/ttyUSB0", 115200);

    // Add one or more servos to the channel
    channel.add(servo);

    while(now < 30000) {
        now = millis();
        if (now > nextUpdate) {
            // request a number of parameters from the servo
            AsyncToken u = servo.ReadAsync(LssPosition | LssVoltage | LssCurrent | LssTemperature);

            if (u.isValid()) {
                // you can check the status of the AsyncToken using u.isComplete()
                // bool isDone = u.isComplete()

                // or just have the channel do the wait for you synchronously.
                // waitFor() also calls channel.update() which is a must if you are checking
                // AsyncToken.isComplete() yourself.
                if (channel.waitFor(u)) {
                    // when the async read is complete, the telemetry of the servo will be
                    // uddated, so we can just output those values.
                    std::cout
                            << "Servo " << servo.id << ":  Position " << (servo.position / 10.0) << "deg"
                            << "   Voltage " << (servo.voltage / 10.0) << "mV   Current " << servo.current
                            << "mA   Temperature " << (servo.temperature / 10.0) << "°C" << std::endl;
                    success++;
                } else {
                    std::cout << (u.isUnresponsive() ? "Timeout" : "Partial Response") << std::endl;
                    failed++;
                }
            }

            // schedule the next position change
            nextUpdate = now + UPDATE_DELAY;
        }

        // must call channel update in main loop so the Servo sub-system
        // can respond to serial, async read and other events.
        channel.update();
    }

    printf("sent %d messages\n", success);
    if(failed>0)
        printf("   %d failures", failed);
    return 0;
}
