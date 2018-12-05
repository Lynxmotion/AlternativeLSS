#include <LynxmotionLSS.h>

// the ID of the connected servo
#define SERVO_ID			2

/// The serial port that contains the servo
#define LSS_SERIAL_PORT     Serial3
#define LSS_BAUDRATE		250000

/// Repeat the update every X milliseconds
#define SERVO_DELAY			1500

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LynxChannel channel;

// instantiate a servo
LynxServo servo(SERVO_ID);

// our server will bounce between -90.0 degrees and +90.0 degrees
int range = -900;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // open the serial port for the servo channel
  LSS_SERIAL_PORT.begin(LSS_BAUDRATE);
  
  // pass the serial port to the Servo channel
  channel.begin(LSS_SERIAL_PORT);

  // Add one or more servos to the channel
  channel.add(servo);
}

unsigned long nextUpdate = 0;

void loop() {
	unsigned long now = millis();
	if (now > nextUpdate) {
		// move the servo to the opposite position
		range *= -1;
		servo.Write(LssPosition, range);

		// schedule the next position change
		nextUpdate = now + SERVO_DELAY;
	}

	// must call channel update in main loop so the Servo sub-system 
	// can respond to serial, async read and other events.
	channel.update();
}
