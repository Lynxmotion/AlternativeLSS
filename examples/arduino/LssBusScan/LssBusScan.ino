#include <LynxmotionLSS.h>

/// The serial port that contains the servo
#define LSS_SERIAL_PORT     Serial1
#define LSS_BAUDRATE		250000

#define SCAN_DELAY          5000


// a channel represents a bus of servos and is attached to a Arduino Stream
// (typically a HardwareSerial port)
LynxChannel channel;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // open the serial port for the servo channel
  LSS_SERIAL_PORT.begin(LSS_BAUDRATE);
  
  // pass the serial port to the Servo channel
  channel.begin(LSS_SERIAL_PORT);
}

// will hold the status of our Async Read
// we will be issuing a read on all servos on the channel
AsyncToken u;

unsigned long nextUpdate = 0;

void loop() {
	unsigned long now = millis();
  short discovered;
	if (now > nextUpdate) {
		// request a number of parameters from the servo
		//channel.free();

		// perform the bus scan
    Serial.print("SCANNING: ");
    if ((discovered=channel.scan(1, 32)) > 0) {
      if (discovered > 0) {
        Serial.print(discovered);
        Serial.println(" servos");
      }
			// read details from the servos
			AsyncToken u = channel.ReadAsyncAll(LssPosition | LssVoltage | LssCurrent | LssTemperature);

			// wait for the servos to respond
      channel.waitFor(u);

			// print each servo
			for (int i = 0; i < channel.count; i++) {
				const LynxServo& servo = *channel.servos[i];

        Serial.print("  S");
        Serial.print(servo.id);
        Serial.print(":  ");
        if (servo.isResponsive()) {
          // when the async read is complete, the telemetry of the servo will be 
          // uddated, so we can just output those values.
          Serial.print("Position ");
          Serial.print(servo.position / 10.0, 1);
          Serial.print("deg   Voltage ");
          Serial.print(servo.voltage / 1000.0, 1);
          Serial.print("v   Current ");
          Serial.print(servo.current);
          Serial.print("mA   Temperature ");
          Serial.print(servo.temperature / 10.0, 1);
          Serial.println("degC");
        }
        else if(servo.isUnresponsive()) {
          Serial.println("Unresponsive");
        }
        else {
          Serial.println("No response");
        }
      }
      

			// schedule the next position change
			nextUpdate = now + SCAN_DELAY;
      Serial.println(); // blank line
    }
    else
      Serial.println("no servos");
  }

	// must call channel update in main loop so the Servo sub-system 
	// can respond to serial, async read and other events.
	channel.update();
}
