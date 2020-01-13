
#include <LssCommunication.h>

#include <EEPROM.h>
#include <Servo.h>
#include <SoftwareSerial.h>


// if set, we'll also listen and respond on the USB serial to LSS messages.
// we'll always respond on the software serial bus as it is dedicated to LSS communication.
#define LSS_ON_ARDUINO_SERIAL


// Sensor modes
// These sensors we know about and can perform the conversion to standard units
typedef enum {
  // no conversion, raw value
  Analog,

  // Sharp Infrared
  GP2Y0A41SK0F,   // millimeters
  GP2Y0A21YK0F,   // millimeters
  GP2Y0A02YK0F,   // millimeters
} SensorMode;


typedef struct _LssDevice {
  short id;           // LSS bus ID
  bool inverted;      // invert servo angle, or invert analog reading
  union {
    short mode;       // optional device mode (sensor model, unused for servo)
    short color;
  };
} LssDevice;

typedef struct _Config {
  LssDevice servos[3];
  LssDevice sensors[3];
  LssDevice led;  // LSS ID for 2RC on-board RGB LED
} Config;

const Config default_config PROGMEM = {
  // Servos
  // by default we set the Lss bus ID to the pin label
  {
    { 209, false, 0 },
    { 210, false, 0 },
    { 211, false, 0 }
  },

  // Sensors
  // by default we set the Lss bus ID to the pin label
  {
    { 203, false, GP2Y0A21YK0F },
    { 204, false, 0 },
    { 205, false, 0 }
  },

  // RGB color LED
  { 207, false, LssLedOff }
};


// stores our active configuration
// this is loaded from EEPROM or otherwise uses default_config
Config config;


// the 3 servo pins wired to D9, D10, and D11
// on the 2RC board PCB there are 3 digital pins reserved for servos
Servo servos[3];

// the hardware pin for the 3 servos (DO NOT CHANGE)
// these are hard wired pins of the Lss 2RC PCB. To change the LSS Bus ID of servos modify the default config above.
const short hw_pin_servos[] = { 9, 10, 11 };

// the hardware pin for the 3 analog sensor pins (DO NOT CHANGE)
// these are hard wired pins of the Lss 2RC PCB. To change the LSS Bus ID of servos modify the default config above.
const short hw_pin_sensors[] = { A3, A4, A5 };

// the hardware pins for the LED
const short hw_pin_led[] = { 3, 6, 5 };

// the hardware pins of the LSS Software Serial port
const short hw_pin_lss_rx = 15;
const short hw_pin_lss_tx = 16;
const short hw_pin_lss_tx_enable = 14;
const short hw_pin_usb_tx_enable = 7;

// the address of config within the EEPROM address space
const int eeprom_config_start = 0x16;

// if enabled, a CONFIRM command to LED ID will wipe config settings
bool ready_for_defaults = false;

/* TODO:
 * [done] Implement broadcast ID #254 for all servos
 * [done] Assignable Servo and Analog IDs in code
 * [done] Implement Gyre direction (inverts servo), so must convert to using struct for servo with config
 * [done] Implement Limpness L, no parameter, disabled servo. To reenable send a position update.
 * [done] Fix Angle D in tenths of degree
 * [done] Implement RGB LED control 
 * [done] Implement sensor modes
 * [done] Read/write config to eeprom, including LED color and Gyre direction
 * [done] Should be listening on SoftwareSerial TX=D16 + Tri=D14, and RX=D15
 * [done] Implement register setting of sensor modes (AR, QAR and CAR)
 * [done] Implement DEFAULT and CONFIRM command to reset eeprom
 * Implement change of device ID (using QID and CID command, just ID not supported)
 * Implement change of baud rate (QB and CB)
 * Refactor the query/action parsing, its getting busy. Possibly break Q/C patterns into functions on LssDevice
 * Move serial processing function to LssCommunications library
 * 
 * Brahim:
 *   [1] Please update the LSS 2RC Protocol specification to use one of the 9, 10, 11 as ID example for servo, 3,4,5 as 
 *   analog and 7 as LED. I choose the servo and sensor bus IDs to match the labels on the PCB (easy for customer to 
 *   remember). LED could be anything, maybe we should use a high number like +253 or something?.
 *   [2] I used position commands to read/write the analog sensors. Kind of makes the servo and sensor pins similar. I can change this to the requested 'A' command if needed.
 *   [3] I used AR,QAR,CAR to set sensor mode. Sensor mode can do the conversion for some known sensors like the SharpIR. Default sensor mode reads raw.
 *   [4] Gyre mode G,QG,CG can also set inversion on analog sensor values So 0 becomes 1024 and vice versa.
 */

typedef void (*tx_enable_function)(bool en);

// This struct ties a serial bus to a processing buffer
typedef struct _LssSerialBus {
  // the serial bus we are processing
  Stream* port;
  tx_enable_function tx_enable;
  
  // buffer stores characters as they come in
  char cmdbuffer[16];
  char* pcmd;
} LssSerialBus;


/*
 * LSS Bus
 */
void lss_tx_enable(bool en)
{
    digitalWrite(hw_pin_lss_tx_enable, en ? HIGH : LOW);
}



/*
 * Output a RGB value to the onboard LED
 */
void led_output(short red, short green, short blue) {
  digitalWrite(hw_pin_led[0], 255-red);
  digitalWrite(hw_pin_led[1], 255-green);
  digitalWrite(hw_pin_led[2], 255-blue);
}

/*
 * Output a standard LSS color value
 */
void led_standard_output(short lssColor) {
  switch(lssColor) {
    case LssLedOff:  led_output(0  ,0,0); break;            /* 0 */
    case LssRed:     led_output(255,0,0); break;            /* 1 */
    case LssGreen:   led_output(0,255,0); break;            /* 2 */
    case LssBlue:    led_output(0,0,255); break;            /* 3 */
    case LssYellow:  led_output(255,255,0); break;          /* 4 */
    case LssCyan:    led_output(0,255,255); break;          /* 5 */
    case LssMagenta: led_output(255,0,255); break;          /* 6 */
    default:         led_output(255,255,255); break;        /* white */
  }
}

/*
 * If input value is outside extents, clamp to the min or max value
 */
short clamp(short input, short min_value, short max_value) {
  return (input < min_value)
    ? min_value
    : (input > max_value)
      ? max_value
      : input;
}

/*
 * Convert a raw sensor value based on the sensor mode
 */
short sensor_conversion(short input, short mode, bool inverted) {
  if(inverted)
    input = 1024 - input;

  switch(mode) {
    case GP2Y0A41SK0F: return clamp(20760 / (input - 11), 30, 310);
    case GP2Y0A21YK0F: return clamp(48000 / (input - 20), 90, 810);
    case GP2Y0A02YK0F: return clamp(94620 / (input - 17), 190, 1510);
    default: return input;  // no translation
  }
}


short config_present() {
  char hdr[4];
  EEPROM.get(eeprom_config_start, hdr);
  return (hdr[0]=='2' && hdr[1]=='R' && hdr[2]=='C')
    ? hdr[3]    // config version number in 4th byte
    : 0;
}

void write_config() {
  // write the current config
  char hdr[4] = {'2', 'R', 'C', '1'};
  EEPROM.put(eeprom_config_start, hdr);
  EEPROM.put(eeprom_config_start+4, config);
}

void restore_config() {
  // verify header
  if(config_present()) {
    // read the full config
    EEPROM.get(eeprom_config_start+4, config);
  } else
    write_config(); // first write
}

void write_config(const LssDevice& dev, int offset, int elnum=0) {
  // ensure there is a valid EEPROM config
  // there must be since we write one on startup
  if(config_present()) {
    EEPROM.put(eeprom_config_start + 4 + offset + (sizeof(LssDevice)*elnum), dev);
  }
}


/*
 * Match an LSS bus ID to a device in a collection,
 * returns -1 if no matching device
 */
short resolve_device(const LssDevice* arr, short arrN,  unsigned short id) {
  for(short i=0; i < arrN; i++) {
    if(arr[i].id == id)
      return i;
  }
  return -1;  // no device found
}

#define COUNTOF(arr)  (sizeof(arr)/sizeof(arr[0]))

void process_packet(LssSerialBus& bus, LynxPacket p) {
  // todo: if we know its a servo command we can get the servo beforehand, or we can get sensor address, whatever
  // todo: we can also automate the response printing
  short n;

  bool query = p.command & LssQuery;

  // if flash write is requested
  bool flash = p.command & LssConfig;


  if(p.id == 254) {
    // recursively select all servos
    for(short i = 0; i < COUNTOF(config.servos); i++) {
      p.id = config.servos[i].id;
      process_packet(bus, p);
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
      servo.write(map(p.value, -900, 900, 0, 180));   // unfortunately our RC precision is only in integral degrees
      p.id = 0;   // squelch response
    }

    // position query
    else if(p.command == (LssDegrees|LssPosition|LssQuery) && !p.hasValue) {
      p.set(servo.read() * 10);    // must return in tenths of a degree
    }

    // gyre direction query
    else if(p.matches(LssGyreDirection|LssQuery)) {
      p.set(config.servos[n].inverted ? -1 : +1);
    }

    // set gyre direction
    else if(p.matches(LssGyreDirection) && !p.matches(LssQuery)) {
      if(p.value == -1)
        config.servos[n].inverted = true;
      else if(p.value == 1)
        config.servos[n].inverted = false;
      else
        return; // invalid input, dont response

      if(flash)
        write_config(config.servos[n], offsetof(Config, servos), n);
    }

    // Limp query
    else if(p.command == (LssLimp|LssQuery)) {
      // no value expected
      p.set(servo.attached() ? 1 : 0);
    }

    // set Limp mode (disabled servo)
    else if(p.command == LssLimp) {
      servo.detach();
    }

  } else if((n = resolve_device(config.sensors, COUNTOF(config.sensors), p.id)) >=0) {
    int pin = hw_pin_sensors[n];
    
    // read/write to sensor pins
    if(p.command == (LssDegrees|LssPosition) && p.hasValue) {
      // send the command to the servo
      digitalWrite(pin, p.value);
    } 

    // sensor query
    else if(p.command == (LssDegrees|LssPosition|LssQuery) && !p.hasValue) {
      short input = analogRead(pin);

      // convert the input and set as output value
      p.set(
        sensor_conversion(
          input,
          config.sensors[n].mode,
          config.sensors[n].inverted
        )
      );
      //p.set(pin * 3);
    }

    // gyre direction query
    // todo: this can be refactored since we also do it above
    else if(p.matches(LssGyreDirection|LssQuery)) {
      p.set(config.sensors[n].inverted ? -1 : +1);
    }

    // set gyre direction
    else if(p.matches(LssGyreDirection) && !p.matches(LssQuery)) {
      if(p.value == -1)
        config.sensors[n].inverted = true;
      else if(p.value == 1)
        config.sensors[n].inverted = false;
      else
        return; // invalid input, dont response
    }

    else if (p.matches(LssAngularRange)) {
      if(query)
        p.set(config.sensors[n].mode);
      else if(p.hasValue) {
        config.sensors[n].mode = p.value;
      }
    }

    if(!query && flash)
      write_config(config.sensors[n], offsetof(Config, sensors), n);

  } else if(p.id == config.led.id) {
    // LED query
    if(p.matches(LssLEDColor|LssQuery)) {
      p.set(config.led.color);
    }

    // set LED color
    else if(p.matches(LssLEDColor)) {
      if(!p.hasValue || p.value<0) return; // invalid value, no response
      led_standard_output( config.led.color = p.value );

      if(flash)
        write_config(config.led, offsetof(Config, led));
    }

    else if(p.matches(LssDefault)) {
      ready_for_defaults = true;
    } else if(ready_for_defaults && p.matches(LssConfirm)) {
      memcpy_P(&config, &default_config, sizeof(default_config));
      write_config();
    }
    
  } else
    return; // not a known device, do not reply
  
  // by default we print the response back
  if(p.id >0) {
    if(bus.tx_enable)
      bus.tx_enable(true);
    bus.port->print('*');
    bus.port->print(p.toString());
    bus.port->print('\r');
    if(bus.tx_enable) {
      bus.port->flush();
      bus.tx_enable(false);
    }
  }
}


LssSerialBus arduinoSerial;

void setup() {
  Serial.begin(115200);

  arduinoSerial.port = &Serial;
  arduinoSerial.tx_enable = lss_tx_enable;    // no need to control TX line on arduino serial
  arduinoSerial.pcmd = arduinoSerial.cmdbuffer;

  // start with default config
  // copies config from program memory
  memcpy_P(&config, &default_config, sizeof(default_config));
  restore_config();   // will read from EEPROM, or write to it if it doesnt exist

  // disable USB tx input line so we only receive LSS bus serial data
  pinMode (hw_pin_usb_tx_enable, OUTPUT);
  digitalWrite(hw_pin_usb_tx_enable, LOW);

  // Define pin as Input
  pinMode (A3, INPUT);
  pinMode (A4, INPUT);
  pinMode (A5, INPUT);

  // configure LED pins as outputs
  pinMode (hw_pin_led[0], OUTPUT);
  pinMode (hw_pin_led[1], OUTPUT);
  pinMode (hw_pin_led[2], OUTPUT);
  led_standard_output(config.led.color);

  analogReference(DEFAULT);

  // configure the TX enable tr-state buffer
  pinMode (hw_pin_lss_tx_enable, OUTPUT);
  lss_tx_enable(false);
}


void serialbus_process(LssSerialBus& bus) {
  while (bus.port->available() > 0) {
    // read the incoming byte
    int c = bus.port->read();

    if(c == '\r') {
      *bus.pcmd = 0;  // append null

      // todo: we can probably convert this to a state machine model
      if(bus.cmdbuffer[0] == '#') {
        // parse the LSS command
        LynxPacket pkt;
        if(pkt.parse(&bus.cmdbuffer[1])) {
          process_packet(bus, pkt);
        }
      }

      // clear packet buffer
      bus.pcmd = bus.cmdbuffer;
    } else if(c == '#') {
      // for now, we reset the buffer if we encounter a packet-start character
      bus.pcmd = bus.cmdbuffer;
      *bus.pcmd++ = (char)c;
    } else {
      // add to cmd buffer
      *bus.pcmd++ = (char)c;
    }
  }
}

void loop() {
  serialbus_process(arduinoSerial);
}
