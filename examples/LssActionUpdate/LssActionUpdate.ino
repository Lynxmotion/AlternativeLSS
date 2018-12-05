#include <LynxmotionLSS.h>

// the ID of the connected servo
#define SERVO_MASTER_ID     2
#define SERVO_SLAVE_ID      1

/// The serial port that contains the servo
#define LSS_SERIAL_PORT     Serial1
#define LSS_BAUDRATE		250000

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LynxChannel channel;

// instantiate a servo
LynxServo master(SERVO_MASTER_ID);
LynxServo slave(SERVO_SLAVE_ID);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // open the serial port for the servo channel
  LSS_SERIAL_PORT.begin(LSS_BAUDRATE);
  
  // pass the serial port to the Servo channel
  channel.begin(LSS_SERIAL_PORT);

  // Add one or more servos to the channel
  channel
     .add(master)
     .add(slave);

  master.Write(LssLimp);
}

// will hold the status of our Async Read
// we will be issuing a read on all servos on the channel
AsyncToken u;

unsigned long nextUpdate = 0;
unsigned long nextPrint = 0;

void loop() {
	unsigned long now = millis();
	if (now > nextUpdate) {
		// request a number of parameters from both servos
		// here we use ReadAsyncAll on the channel to read from all servos. The AsyncToken
		// will not be marked complete until all servos have responded (or timed out)
		u = channel.ReadAsyncAll(LssPosition);

		// it would be easier to just call channel.waitFor(u) here but we'll demonstrate
		// Action -> Update loop instead
		// channel.waitFor(u);

		// schedule the next position change
		// we'll reset this anytime we get an update
		nextUpdate = now + 500;
		delay(50);
	}

	if (u.isComplete()) {
		// clear the token so we only get here once for each update
		u.clear();

		// when the async read is complete, the telemetry of the servo will be 
		// uddated, so we can just output those values.
		if (now > nextPrint) {
			nextPrint = now + 1000;
			Serial.print(master.position);
			Serial.print(" => ");
			Serial.println(slave.position);
			master.Write(LssLimp);
    }

    slave.Write(LssPosition, master.position);

		nextUpdate = now;	// will cause a ReadAsyncAll in the next loop
	}

	// must call channel update in main loop so the Servo sub-system 
	// can respond to serial, async read and other events.
	channel.update();
}
