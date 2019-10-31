
#include <LssCommunication.h>

#include <Servo.h>

// SharpIR sensor library by Giuseppe Masino
// Available in Library Manager as SharpIR or at https://github.com/qub1750ul/Arduino_SharpIR.git
#include <SharpIR.h>


typedef struct _LssDevice {
  short id;           // LSS bus ID
  bool inverted;      // invert servo angle, or invert analog reading
} LssDevice;

typedef struct _Config {
  LssDevice servos[3];
  LssDevice sensors[3];
  
  short LedId;  // LSS ID for 2RC on-board RGB LED
} Config;

const Config default_config PROGMEM = {
  // Servos
  // by default we set the Lss bus ID to the pin label
  {
    { 9, false },
    { 10, false },
    { 11, false }
  },

  // Sensors
  // by default we set the Lss bus ID to the pin label
  {
    { 3, false },
    { 4, false },
    { 5, false }
  },

  // RGB color LED
  7
};


// stores our active configuration
// this is loaded from EEPROM or otherwise uses default_config
Config config;



// the 3 servo pins wired to D9, D10, and D11
// on the 2RC board PCB there are 3 digital pins reserved for servos
Servo servos[3];

// optionally our analog inputs can be a SharpIR sensor
SharpIR sharpIR(SharpIR::GP2Y0A21YK0F, A3);

// the hardware pin for the 3 servos (DO NOT CHANGE)
// these are hard wired pins of the Lss 2RC PCB. To change the LSS Bus ID of servos modify the default config above.
const short hw_pin_servos[] = { 9, 10, 11 };

// the hardware pin for the 3 analog sensor pins (DO NOT CHANGE)
// these are hard wired pins of the Lss 2RC PCB. To change the LSS Bus ID of servos modify the default config above.
const short hw_pin_sensors[] = { A3, A4, A5 };


/* TODO:
 * [done] Implement broadcast ID #254 for all servos
 * [done] Assignable Servo and Analog IDs in code
 * Implement Gyre direction (inverts servo), so must convert to using struct for servo with config
 * Implement Limpness L, no parameter, disabled servo. To reenable send a position update.
 * [done] Fix Angle D in tenths of degree
 * Implement RGB LED control 
 * Read/write config to eeprom
 * Should be listening on SoftwareSerial TX=D16 + Tri=D14, and RX=D15
 * 
 * Brahim:
 *   Please update the LSS 2RC Protocol specification to use one of the 9, 10, 11 as ID example for servo, 3,4,5 as 
 *   analog and 7 as LED. I choose the servo and sensor bus IDs to match the labels on the PCB (easy for customer to 
 *   remember). LED could be anything, maybe we should use a high number like +253 or something?.
 */


short resolve_device(const LssDevice* arr, short arrN,  unsigned short id) {
  for(short i=0; i < arrN; i++) {
    if(config.servos[i].id == id)
      return i;
  }
  return -1;  // no device found
}

#define COUNTOF(arr)  (sizeof(arr)/sizeof(arr[0]))

void process_packet(LynxPacket p) {
  // todo: if we know its a servo command we can get the servo beforehand, or we can get sensor address, whatever
  // todo: we can also automate the response printing
  short n;
  if(p.id == 254) {
    // recursively select all servos
    for(short i = 0; i < COUNTOF(config.servos); i++) {
      p.id = config.servos[i].id;
      process_packet(p);
    }
    return;
    
  } else if((n = resolve_device(config.servos, COUNTOF(config.servos), p.id)) >=0) {
    Servo& servo = servos[n];
    
    // position command 
    if(p.command == (LssDegrees|LssPosition) && p.hasValue) {
      // ensure the servo is on
      if(!servo.attached())
        servo.attach(hw_pin_servos[n]);
        
      // send the command to the servo
      servo.write(p.value / 10);   // unfortunately our RC precision is only in integral degrees
    }

    // position query
    else if(p.command == (LssDegrees|LssPosition|LssQuery) && !p.hasValue) {
      p.set(servo.read() * 10);    // must return in tenths of a degree
    }
  
  } else if((n = resolve_device(config.sensors, COUNTOF(config.sensors), p.id)) >0) {
    int pin = hw_pin_sensors[n];
    
    // read/write pins
    if(p.command == (LssDegrees|LssPosition) && p.hasValue) {
      // send the command to the servo
      digitalWrite(pin, p.value);
    } 

    // position query
    else if(p.command == (LssDegrees|LssPosition|LssQuery) && !p.hasValue) {
      p.set(analogRead(pin));
      //p.set(pin * 3);
    }
  } else if(p.id == config.LedId) {
    // update the color LED
    
  } else
    return; // not a known device, do not reply
  
  // by default we print the response back
  if(p.id >0) {
    Serial.print('*');
    Serial.println(p.toString());
  }
}

void setup() {
  Serial.begin(115200);

  // start with default config
  memset(&config, &default_config, sizeof(default_config));

  // set the ADC resolution
  //analogReadResolution(hw_adc_resolution)

  // enable pullups
  //pinMode(A3, INPUT_PULLUP);
  //pinMode(A4, INPUT_PULLUP);
  //pinMode(A5, INPUT_PULLUP);

  // Define pin as Input
  pinMode (A3, INPUT);
  pinMode (A4, INPUT);
  pinMode (A5, INPUT);

  analogReference(DEFAULT);
}

unsigned long long next_update=0;

char cmdbuffer[32];
char* pcmd = cmdbuffer;

LynxPacket pkt;

void loop() {
  while (Serial.available() > 0) {
    // read the incoming byte
    int c = Serial.read();

    if(c == '\r') {
      *pcmd = 0;  // append null

      // todo: we can probably convert this to a state machine model
      if(cmdbuffer[0] == '#') {
        // parse the LSS command
        if(pkt.parse(&cmdbuffer[1])) {
          process_packet(pkt);
        }
      }

      // clear packet buffer
      pcmd = cmdbuffer;
    } else if(c == '#') {
      // for now, we reset the buffer if we encounter a packet-start character
      pcmd = cmdbuffer;
      *pcmd++ = (char)c;
    } else {
      // add to cmd buffer
      *pcmd++ = (char)c;
    }
  }

  // if we reached the update timer, print data
  if(millis() > next_update) {
    next_update = millis() + 1000;
    Serial.print("A[");
    Serial.print(analogRead(A3));
    Serial.print(", ");
    Serial.print(analogRead(A4));
    Serial.print(", ");
    Serial.print(analogRead(A5));    
    Serial.print(", IR:");
    Serial.print(sharpIR.getDistance());
    Serial.println("cm]");
  }
}
