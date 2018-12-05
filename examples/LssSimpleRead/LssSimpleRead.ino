#include <LynxmotionLSS.h>

// the ID of the connected servo
#define SERVO_ID			2

/// The serial port that contains the servo
#define LSS_SERIAL_PORT     Serial3
#define LSS_BAUDRATE		250000

/// Repeat the update 4 times a second
#define SERVO_DELAY			500

// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LynxChannel channel;

// instantiate a servo
LynxServo servo(SERVO_ID);

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
		// request a number of parameters from the servo
		AsyncToken u = servo.ReadAsync(LssPosition|LssVoltage|LssCurrent|LssTemperature);

		if (u.isValid()) {
			// you can check the status of the AsyncToken using u.isComplete()
			// bool isDone = u.isComplete()

			// or just have the channel do the wait for you synchronously.
			// waitFor() also calls channel.update() which is a must if you are checking
			// AsyncToken.isComplete() yourself.
			if (channel.waitFor(u)) {
				// when the async read is complete, the telemetry of the servo will be 
				// uddated, so we can just output those values.
				Serial.print("Servo ");
				Serial.print(servo.id);
				Serial.print(":  Position ");
				Serial.print(servo.position/10.0,1);
				Serial.print("deg   Voltage ");
				Serial.print(servo.voltage/10.0,1);
				Serial.print("v   Current ");
				Serial.print(servo.current);
				Serial.print("mA   Temperature ");
				Serial.print(servo.temperature/10.0,1);
				Serial.println("degC");
			}
			else
				Serial.println( u.isUnresponsive() ? "Timeout" : "Partial Response");
		}

		// schedule the next position change
		nextUpdate = now + SERVO_DELAY;
	}

	// must call channel update in main loop so the Servo sub-system 
	// can respond to serial, async read and other events.
	channel.update();
}
