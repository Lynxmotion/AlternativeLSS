
#include <LssCommunication.h>
#include <LssHandlers.h>

#include <EEPROM.h>
#include <AccelStepper.h>
#include <avr/io.h>
#include <avr/wdt.h>

// only define this if you are using the LSS2MC in Arduino mode (Not using the LSS bus) such as testing
#define ARDUINO_DEV_MODE

// Sensor modes
// These sensors we know about and can perform the conversion to standard units
typedef enum {
  // no conversion, raw value
  DualBrushedMode,
  StepperMode
} MotorMode;

typedef enum {
  HoldingMode,
  SteppingMode,
  SpeedMode,
  AngleMode
} MotionMode;


typedef struct _Config2IO {
  long baudrate;
  short id;
  MotorMode motor_mode;
  uint16_t reserved[8];    // future use
} Config2IO;


typedef struct _LssStepper {
  bool reverse;
  float stepsPerRev;
  MotionMode motion_mode;
} LssStepper;

typedef struct _LssBrushedMotor {
  short A;    // pin A
  short B;    // pin B
  bool reverse;
} LssBrushedMotor;


typedef struct _Config {
  Config2IO io;
  LssStepper stepper;
  LssBrushedMotor motor[2];
} Config;

const char EEPROM_CONFIG_VER = '1';   // update this number if eeprom config layout changes

const Config default_config PROGMEM = {
  // 2IO
  {
    115200,               // default baudrate
    215,
    StepperMode,
    {0, 0, 0, 0, 0, 0, 0, 0}
  },

  // Stepper
  {
    false,
    180.0,
    HoldingMode
  },

  // Brushed Motors
  {
    { false },
    { false }
  }
};

// supported baud rates of the 2IO
const long SupportedBaudrates[] = {9600, 19200, 38400, 57600, 115200, 230400, 250000, 460800, 500000 };


// stores our active configuration
// this is loaded from EEPROM or otherwise uses default_config
Config config;

// the following are the pin assignments from the schematic
const unsigned short hw_pin_A1 = 9;
const unsigned short hw_pin_A2 = 10;
const unsigned short hw_pin_B1 = 3;
const unsigned short hw_pin_B2 = 11;


// the 3 servo pins wired to D9, D10, and D11
// on the 2RC board PCB there are 3 digital pins reserved for servos
// Other Stepper libraries: MD_Stepper (int based), uStepper, AsyncStepperLib, FastAccelStepper
AccelStepper stepper(AccelStepper::FULL4WIRE, hw_pin_A1, hw_pin_A2, hw_pin_B1, hw_pin_B2);


// pin to enable/disable the motor control chip
const uint8_t STEPPER_NSLEEP = 2;


// by default the Atmel transmits on USB so software uploads work,
// but in setup() we quickly switch to transmitting on the LSS bus
const unsigned short hw_pin_lss_tx_enable = 14;
const unsigned short hw_pin_usb_tx_enable = 7;

// hardware pin to monitor at startup, if LOW for ~6 seconds will then issue firmware reset
// The LSS RX pin (pin 0) or MISO pin is recommended. The MISO pin is found on the bottom of the PCB as a TEST point pin
const unsigned short hw_firmware_reset_pin = 0;

// the address of config within the EEPROM address space
const int eeprom_config_start = 0x16;

// if enabled, a CONFIRM command to LED ID will wipe config settings
bool ready_for_defaults = false;



// count the number of elements in a C array
#define COUNTOF(arr)  (sizeof(arr)/sizeof(arr[0]))

// todo: put me elsewhere
//void * operator new (size_t size, void * ptr) { return ptr; }


/*
   Enable the transmission of data on the LSS Bus

   The transmission of data is only enabled for the short duration we are actually transmitting, otherwise the
   tri-state buffer is kept in a high impedance state so other bus devices can transmit.
*/
void lss_tx_enable(bool en)
{
  digitalWrite(hw_pin_lss_tx_enable, en ? HIGH : LOW);
}

/*
   Send a string or manual packet reply with a string value.
   Required because LynxPacket for simplicity doesnt support string packet values.
*/
void lss_transmit(String s) {
  // The handler may update the packet for reply transmission
  // if so indicated, print the packet back to master
  lss_tx_enable(true);
  Serial.print('*');
  Serial.print(s);
  Serial.print('\r');
  Serial.flush();
  lss_tx_enable(false);
}

/*
   Send a device model
*/
void transmit_model(short id, String modelMode) {
  String reply;
  reply += id;
  reply += "QMSLSS-2MC-";
  reply += modelMode;
  lss_transmit(reply);
}

void motor_driver_limp() {
  if(config.io.motor_mode == StepperMode)
    stepper.disableOutputs();
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);  
}

void motor_driver_enable() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  if(config.io.motor_mode == StepperMode)
    stepper.enableOutputs();
}


/*
   Force a reset by configuring the watchdog timer to timeout
*/
void reset() {
  wdt_enable(WDTO_30MS);
  while (1) {};
}


// this bytes indicate if the EEPROM has stored a proper config
char eeprom_hdr[4] = {'2', 'M', 'C', EEPROM_CONFIG_VER};

/*
   Returns true if the given baudrate is one of the LSS/2IO supported rates.
*/
bool baudrate_is_supported(long baudrate) {
  for (int i = 0; i < COUNTOF(SupportedBaudrates); i++)
    if (SupportedBaudrates[i] == baudrate)
      return true;
  return false;
}

/*
   Returns the header version of the config stored in EEPROM,
   or 0 if config is not present.
*/
short config_present() {
  char hdr[4];
  EEPROM.get(eeprom_config_start, hdr);
  return (hdr[0] == eeprom_hdr[0] && hdr[1] == eeprom_hdr[1] && hdr[2] == eeprom_hdr[2])
         ? hdr[3]    // config version number in 4th byte
         : 0;
}

/*
   Write the entire config structure to the EEPROM.
*/
void write_config() {
  // write the current config
  EEPROM.put(eeprom_config_start, eeprom_hdr);
  EEPROM.put(eeprom_config_start + 4, config);
}

void write_default_config() {
  memcpy_P(&config, &default_config, sizeof(default_config));        // reset to defaults
  write_config();                                                    // write defaults to EEPROM
}

/*
   Load the entire config from EEPROM into the current config.
*/
void restore_config() {
  // verify header
  if (config_present() == EEPROM_CONFIG_VER) {
    // read the full config
    EEPROM.get(eeprom_config_start + 4, config);
  } else
    write_config(); // first write
}

/*
   Write a sub-structure within our EEPROM config.
   This template is aware of the structure size, and automatically computes the offset based on the
   memory offset from the current config.
*/
template<class T>
void write_config_object(const T& obj) {
  if (config_present() == EEPROM_CONFIG_VER) {
    // write a single device record within the config
    int offset = (unsigned char*)&obj - (unsigned char*)&config;
    if (offset >= 0 && offset < sizeof(config)) {
      EEPROM.put(eeprom_config_start + 4 + offset, obj);
    }
  } else
    write_config();   // write entire config
}

/*
                                           COMMAND HANDLERS

   The following code implements all the command handlers for Servos, Sensors and other 2IO devices
   like the tri-color LED. Each command handler is associated with an LSS command and mode (Action, Query
   or Config). For each packet received the handlers are enumerated and matched against the packet
   command and mode. If a command match is found then that handler is executed.

   Handlers are implemented here as C++ lambda functions but can be a normal function as long as
   you follow the function prototype for that class of handlers. The process_packet() function below
   is the initial packet handler that determines if the packet is targeting a servo, sensor or other
   2IO device and calls the appropriate class of handlers with the matched device details as arguments.

   You can easily add more handlers or devices. Start by cut-and-pasting existing code and modifying
   with the new command match or handler code.

   Each Packet Command Handler has the following syntax:

   { <Command-BitMask> | <Command-Mode = LssQuery|LssAction|LssConfig>,
     <LssNone or LssNoBroadcast|LssMatchAny>,
     <callback-function>
   }

   Command-BitMask
   The command to match such as LssPosition, LssLEDColor, LssBaudRate, LssID or any of the command
   bits as specified in Arduino/library/LynxmotionLSS/src/LssCommunication.h header. In some cases
   such as LssPosition you may also need to add the units such as LssDegrees, LssPulse or LssRPM.
   By default all command bits must match but you can match on any command bits by specifying the
   LssMatchAny flag.


   Command Mode (LssQuery|LssAction|LssConfig)
   Match packets based on if they are queries, action and if they have the config modifier to indicate
   they should write to non-volatile flash. It is ok to specify multiple modes and packets will
   match if any of the mode bits match.

   Match Flags (LssNoBroadcast|LssMatchAny)
   Control how packet matching is performed.
   LssNoBroadcast - By default handlers will answer broadcast messages, you can limit a handler to
                    only it's own LSS bus ID using this flag.
   LssMatchAny    - By default all Handler command bits must match the packet command bits but by
                    using this flag you can indicate you want to match if ANY of the handler command
                    bits are set in the packet command bits. For example, this is used to handle the
                    Config flash writes for multiple commands in one handler. Since multiple handlers
                    may match and be invoked on a packet the first handler updates the config structure,
                    and the second handler does the write to flash.



   SYS handlers
   These handlers answer LED Color and other 2IO global config parameters like Baudrate. These are
   configured on LSS bus ID 207 by default but can be changed via the CID command.

*/



/*  Stepper Motor Mode
 *   
 *   If this mode is enabled then commands in this block become active for controller a stepper, otherwise
 *   the brushed motor handlers would be active.
 */
LssPacketHandlers<> StepperHandlers
({
  /*
     Returns model string

    Ex: #207QMS<cr>
    Returns:  *207QMSLSS-2IO-LED
  */
  { LssModel | LssQuery,                          LssNone,
    [](LynxPacket & p) {
      transmit_model(p.id, "ST");
      return LssNoReply;
    }
  },
    
  /*
     Position in Degrees


    This moves the servo to an angle of 145.6 degrees, where the center (0) position is centered. Negative
    values (ex. -176 representing -17.6 degrees) are used. A full circle would be from -1800 to 1800 degrees.
    A value of 2700 would be the same angle as -900, except the servo would move in a different direction.
    Larger values are permitted and allow for multi-turn functionality using the concept of virtual position.

    Set Position in Degrees (D)
    Ex: #5D132<cr>

    Query Position in Degrees (QD)
    Ex: #5QD<cr> might return *5QD132<cr>

    This above two examples set the target position to 13.2 degrees and then after some time confirms the
    servos position has reached 13.2 degrees.

  */
  { LssWheelMode| LssDegrees | LssAction,           LssNoBroadcast,
    [](LynxPacket & p) {
      motor_driver_enable();
      config.stepper.motion_mode = SpeedMode;
      stepper.setSpeed(p.value / 10);
      return LssReply;
    }
  },

  
#if 0
  { LssPosition | LssDegrees | LssQuery,            LssNoBroadcast,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin, Servo & servo) {
      p.set(map( dev.inverted ? -servo.read() : servo.read(), 0, 180, -900, 900));
      return LssReply;
    }
  },
  { LssPosition | LssDegrees | LssAction,           LssNone,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin, Servo & servo) {
      // ensure the servo is on
      if (!servo.attached())
        servo.attach(pin);

      // send the command to the servo
      servo.write(map(dev.inverted ? -p.value : p.value, -900, 900, 0, 180));   // unfortunately our RC precision is only in integral degrees
      return LssNoReply;
    }
  },
  
  /*
     Position in Pulse Width


    The position in PWM pulses was retained in order to be backward compatible with the SSC-32 / 32U protocol.
    This relates the desired angle with an RC standard PWM pulse and is further explained in the SSC-32 and
    SSC-32U manuals found on Lynxmotion.com. Without any modifications to configuration considered, and a
    ±90.0 degrees standard range where 1500 microseconds is centered, a pulse of 2334 would set the servo to
    165.1 degrees. Valid values for P are [500, 2500]. Values outside this range are corrected / restricted to
    end points.

    Set Position in Pulse Width (QP)
    Ex: #5P2334<cr>

    Query Position in Pulse Width (QP)

    Ex: #5QP<cr> might return *5QP2334

    This command queries the current angular position in PWM "units". The user must take into consideration that
    the response includes any angular range and origin configurations in order to determine the actual angle.
    Valid values for QP are {-500, [500, 2500], -2500}. Values outside the [500, 2500] range are given a negative
    corresponding end point value to indicate they are out of bounds (note that if the servo is physically located
    at one of the endpoints, it may return a negative number if it is a fraction of a degree beyond the position).

  */
  { LssPosition | LssPulse | LssQuery,            LssNoBroadcast,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin, Servo & servo) {
      p.set(servo.readMicroseconds());
      return LssReply;
    }
  },
  { LssPosition | LssPulse | LssAction,           LssNone,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin, Servo & servo) {
      // ensure the servo is on
      if (!servo.attached())
        servo.attach(pin);

      // send the command to the servo
      servo.writeMicroseconds(constrain(p.value, 500, 2500));   // unfortunately our RC precision is only in integral degrees
      return LssNoReply;
    }
  },
#endif


  /*
     Limp


    This action causes the servo to go "limp". The microcontroller will still be powered, but the motor will not.
    As an emergency safety feature, should the robot not be doing what it is supposed to or risks damage, use the
    broadcast ID to set all servos limp #254L<cr>.

    Command the servo to go limp (L)
    Ex: #5L<cr>

  */
  { LssHaltAndHold | LssAction,           LssNoBroadcast,
    [](LynxPacket & p) {
      motor_driver_enable();
      config.stepper.motion_mode = HoldingMode;
      stepper.setSpeed(0);
      return LssReply;
    }
  },
  
  { LssLimp | LssAction,               LssNone,
    [](LynxPacket & p) {
      motor_driver_limp();
      return LssNoReply;
    }
  },
});


/*  Dual Brushed Motor Mode
 *   
 *   If this mode is enabled then commands in this block become active to contol 1 or 2 brushed motors, otherwise
 *   the stepper handlers would be active.
 */
LssPacketHandlers<LssBrushedMotor&> DualBrushedHandlers
({
  /*
     Returns model string

    Ex: #207QMS<cr>
    Returns:  *207QMSLSS-2IO-LED
  */
  { LssModel | LssQuery,                          LssNone,
    [](LynxPacket & p, LssBrushedMotor& m) {
      transmit_model(p.id, "DBR");
      return LssNoReply;
    }
  },
    
  { LssAnalog | LssQuery,            LssNoBroadcast,
    [](LynxPacket & p, LssBrushedMotor& m) {
      return LssNoReply;
    }
  },

  { LssSpeed | LssPulse | LssQuery,               LssNone,
    [](LynxPacket & p, LssBrushedMotor& m) {
      transmit_model(p.id, "RAW");
      return LssNoReply;
    }
  }

  
#if 0
  /*
     Read Value

    We use the Analog command to read or write to GPIO. If the GPIO pin is configured as a known sensor then
    the value will be converted into that sensors output units. Use AngularRange (AR) command to set a sensor's mode.

    Writing to an analog pin uses the Arduino analogWrite() on pins A3 and A5 which will output a PWM signal with appropriate
    duty cycle. You will need a low-pass filter on the pin if you truly want to generate an analog voltage signal. Pin A4 only
    supports digital so outputting a anything other than 0 will set the pin HIGH.

    Read a sensor value
    Ex: #203QD

    Write to a sensor or GPIO pin
    Ex: #203D800

  */
  { LssAnalog | LssQuery,            LssNoBroadcast,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin) {
      if (dev.mode == ReceiverPPM) {
        transmit_ppm(dev.id);
        return LssNoReply;
      } else {
        // convert the input and set as output value
        p.set(
          sensor_conversion(
            analogRead(pin),                                // read input from pin
            p.between(0, LastSensorMode) ? p.value : 0,     // type of sensor
            dev.inverted
          )
        );
      }
      return LssReply;
    }
  },
  { LssAnalog | LssAction,           LssNone,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin) {
      if (dev.mode != ReceiverPPM) {
        pinMode(pin, OUTPUT);
        analogWrite(pin, p.value);
      }
      return LssNoReply;
    }
  },

  // Configure the pins sensor mode
  { LssAngularRange | LssAction,           LssNone,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin) {
      if (dev.mode == ReceiverPPM)
        ppm_stop();
      dev.mode = p.value;
      if (dev.mode == ReceiverPPM)
        ppm_start(pin, dev.inverted);
      return LssNoReply;
    }
  },

  /*
     Returns model string

    Ex: #203QMS<cr>
    Returns:  *203QMSLSS-2IO-A3
  */
  { LssModel | LssQuery,                          LssNone,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin) {
      transmit_model(dev.id, "A", pin - A0); // must remove the An pin offset
      return LssNoReply;
    }
  },

  /*
     Write to flash
     This handler will write the device config to flash when the Config comamnd prefix is given. This handler runs after any of
     the above handlers have already updated the config sctruct.
     We specify LssMatchAny so the command will trigger on ANY of the command names instead of requiring ALL to match. So the
     command list here specifies all commands that support setting the config option in non-volatile flash.
  */
  { LssAngularRange | LssConfig,           LssMatchAny,
    [](LynxPacket & p, LssDevice & dev, unsigned short pin) {
      write_config_object(dev);
      return LssNoReply;
    }
  }
#endif
});


/*
   Command GPIO handlers
   These handlers answer commands for any LssDevice type, which can be a servo, sensor or LED. If none of the above collection
   of handlers process a command then it will fall back to these common handlers to possibly handle the command.

*/
LssPacketHandlers<> CommonDeviceHandlers
({
  /*
     Change Device LSS Bus ID

    A servo's identification number cannot be set "on the fly" and must be configured via the CID command described below. The
    factory default ID is 209, 210 and 211 for servos, 203, 204 and 205 for analog/sensor pins and 207 for LED. These were generally
    chosen to be 200 plus the hardware pin number as written on the PCB solder mask.

    Since smart servos are intended to be daisy chained, in order to respond differently from one another, the user must set different
    identification numbers. Servos with the same ID and baud rate will all receive and react to the same commands (assuming same baud rate).

    Query Identification (QID)
    Ex: #254QID<cr> might return *QID5<cr>

    When using the query ID command, it is best to only have one servo connected and thus receive only one reply. This is useful when
    you are not sure of the servo's ID, but don't want to change it. Using the broadcast command (ID 254) with only one servo will have
    that servo reply with its ID number (assuming the query is sent . Alternatively, pushing the button upon startup and temporarily
    setting the servo ID to 255 will still result in the servo responding with its "real" ID.

    Configure ID (CID)
    Ex: #4CID5<cr>

    Setting a servo's ID in EEPROM is done via the CID command. All servos connected to the same serial bus will be assigned that ID. In
    most situations each servo must be set a unique ID, which means each servo must be connected individually to the serial bus and receive
    a unique CID number. It is best to do this before the servos are added to an assembly. Numbered stickers are provided to distinguish
    each servo after their ID is set, though you are free to use whatever alternative method you like. The servo must be RESET or power
    cycled in order for the new ID to take effect.

  */
  { LssID | LssQuery,       LssNoBroadcast,
    [](LynxPacket & p) {
      p.set(config.io.id);
      return LssReply;
    }
  },
  { LssID | LssConfig,       LssNoBroadcast,
    [](LynxPacket & p) {
      if (p.flash() && p.between(0, LssBroadcastAddress - 1))
        config.io.id = p.value;   // only update object, flash handler will follow-up to write to flash
      return LssNoReply;
    }
  },

  /*
     Baud Rate


    A servo's baud rate cannot be set "on the fly" and must be configured via the CB command described
    below. The factory default baud rate for all servos is 115200. Since smart servos are intended to
    be daisy chained, in order to respond to the same serial bus, all servos in a project should ideally
    be set to the same baud rate. Setting different baud rates will have the servos respond differently
    and may create issues. Available baud rates are: 9600 bps, 19200 bps, 38400 bps, 57600 bps, 115.2 kbps,
    230.4 kbps, 250.0 kbps, 460.8 kbps, 500.0 kbps. The 2IO is configured with default baud rate set to 115200.
    The baud rates are currently restricted to those above.

    Query Baud Rate (QB)
    Ex: #207QB<cr> might return *5QB9600<cr>

    Since the command to query the baud rate must be done at the servo's existing baud rate, it can simply
    be used to confirm the CB configuration command was correctly received before the servo is power cycled
    and the new baud rate takes effect.

    Configure Baud Rate (CB)
    Ex: #207CB9600<cr>

    Important Note: the servo's current session retains the given baud rate and the new baud rate will only
    take effect when the servo is power cycled / RESET.

    Sending this command will change the baud rate associated with servo ID 5 to 9600 bits per second.

  */
  { LssBaudRate | LssQuery,            LssNoBroadcast,
    [](LynxPacket & p) {
      // return the baudrate
      p.set(config.io.baudrate);
      return LssReply;
    }
  },
  { LssBaudRate | LssConfig,           LssNone,
    [](LynxPacket & p) {
      if (baudrate_is_supported(p.value)) {
        // set the baudrate
        config.io.baudrate = p.value;
        if (p.flash())
          write_config_object(config.io);
        Serial.begin(config.io.baudrate);   // reset baud rate now
      }
      return LssNoReply;
    }
  },

  /*
     Reset Module

    Ex: #207RESET<cr> or #5RS<cr>

    This command does a "soft reset" (no power cycle required) and reverts all commands to those stored in EEPROM (i.e. configuration
    commands).
    Note: after a RESET command is received the 2IO module will restart and perform initilization again, making it unavailable on the bus
    for a bit. See Session, note #2 for more details.

  */
  { LssReset | LssAction,                            LssNone,
    [](LynxPacket & p) {
      reset();
      return LssNoReply;
    }
  },

  /*
     Reset Config on Module

    Ex: #207DEFAULT<cr>

    This command clears EEPROM contents to factory default and does a "soft reset" (no power cycle required).
    Note: after a RESET command is received the 2IO module will restart and perform initilization again, making it unavailable on the bus
    for a bit. See Session, note #2 for more details.

  */
  { LssDefault | LssAction,                          LssNone,
    [](LynxPacket & p) {
      write_default_config();
      reset();                                                           // soft reset
      return LssNoReply;
    }
  },


  { LssWheelMode | LssAction,                          LssNone,
    [](LynxPacket & p) {
      motor_driver_limp();
      if(p.value == 0) {
        // stepper mode
        config.io.motor_mode = StepperMode;
      } else if(p.value == 1) {
        config.io.motor_mode = DualBrushedMode;
      }
      return LssReply;
    }
  },
  
  /*
     Gyre Direction

    "Gyre" is defined as a circular course or motion. The effect of changing the gyre direction is as if you were to use a mirror image of
    a circle. CW = 1; CCW = -1. The factory default is clockwise (CW).

    Ex: #5G-1<cr>
    This command will cause servo #5's positions to be inverted, effectively causing the servo to rotate in the opposite direction given
    the same command. For example in a 2WD robot, servos are often physically installed back to back, therefore setting one of the servos
    to a negative gyration, the same wheel command (ex WR30) to both servos will cause the robot to move forward or backward rather than
    rotate.

    Query Gyre Direction (QG)
    Ex: #5QG<cr> might return *5QG-1<cr>

    The value returned above means the servo is in a counter-clockwise gyration.

    Configure Gyre (CG)
    Ex: #5CG-1<cr>

    This changes the gyre direction as described above and also writes to EEPROM.

  */
  #if 0
  { LssGyreDirection | LssQuery,                     LssNoBroadcast,
    [](LynxPacket & p, LssDevice & dev) {
      p.set(dev.inverted ? -1 : +1);
      return LssReply;
    }
  },
  { LssGyreDirection | LssAction | LssConfig,          LssNone,
    [](LynxPacket & p, LssDevice & dev) {
      if (p.value == -1)
        dev.inverted = true;
      else if (p.value == 1)
        dev.inverted = false;
      else
        return LssNoReply; // invalid input, dont response
      return LssNoReply;
    }
  },
  #endif

  /*
     Write to flash
     This handler will write the device config to flash when the Config comamnd prefix is given. This handler runs after any of
     the above handlers have already updated the config sctruct.
     We specify LssMatchAny so the command will trigger on ANY of the command names instead of requiring ALL to match. So the
     command list here specifies all commands that support setting the config option in non-volatile flash.
  */
  { LssID | LssGyreDirection | LssConfig,           LssMatchAny,
    [](LynxPacket & p) {
      write_config_object(config.io);
      return LssNoReply;
    }
  }
});



/*
   Given a new packet, determine what device it is intended for and call the packet handlers above with appropriate arguments.
*/
void process_packet(LynxPacket p) {
  short n;
  short r = LssNoReply;

  switch(config.io.motor_mode) {
    case StepperMode:
      if (p.id != config.io.id)
        return;
      r = StepperHandlers(p);
      break;
    case DualBrushedMode:
      if (p.id == config.io.id)
        r = DualBrushedHandlers(p, config.motor[0]);
      else if (p.id == config.io.id + 1)
        r = DualBrushedHandlers(p, config.motor[1]);
      else
        return;
      break;
    default:
      // nothing
      return;
  }
  
  // if we didnt match any handler then see if our command device handlers can
  if (r == LssNoHandler) {
    r = CommonDeviceHandlers(p);
  }

  // The handler may update the packet for reply transmission
  // if so indicated, print the packet back to master
  if (r == LssReply && p.id > 0)
    lss_transmit(p.toString());
}

/*
   Process a packet that was sent with the LSS Broadcast ID.
   for broadcasts we must recurse with each device we know about.

*/
void process_broadcast_packet(LynxPacket p) {
  short n;
  short r = LssNoReply;

  // todo:  loop through all steppers or motors and set
}

/*
   test if user is requesting a factory reset
*/
void test_factory_reset() {
  pinMode(hw_firmware_reset_pin, INPUT_PULLUP);
  delay(15);    // time for pullup to bring the line HIGH

  while ( digitalRead(hw_firmware_reset_pin) == LOW ) {
    unsigned long now = millis();
    if (now > 1000) {
      if (now > 6000) {
        // enough warning, we write defaults and reset!
        write_default_config();
        reset();
      }
    }

    wdt_reset();    // dont let the watchdog timer stop us
  }
  pinMode(hw_firmware_reset_pin, INPUT);
}

void drive_brushed() {
  // todo: update PWM, etc
  
}

void setup() {
  motor_driver_limp();
  
  // configure the TX enable tr-state buffer
  pinMode (hw_pin_lss_tx_enable, OUTPUT);
  lss_tx_enable(false);

  // disable USB tx input line so we only receive LSS bus serial data
#if !defined(ARDUINO_DEV_MODE)
  pinMode (hw_pin_usb_tx_enable, OUTPUT);
  digitalWrite(hw_pin_usb_tx_enable, LOW);
#endif

  // Define motor control pins as output
  pinMode (hw_pin_A1, OUTPUT);
  pinMode (hw_pin_A2, OUTPUT);
  pinMode (hw_pin_B1, OUTPUT);
  pinMode (hw_pin_B2, OUTPUT);

  // hold a servo pin LOW to indicate wish for a factory reset
  test_factory_reset();

  // start with default config
  // copies config from program memory
  memcpy_P(&config, &default_config, sizeof(default_config));
  restore_config();   // will read from EEPROM, or write to it if it doesnt exist

  // configure the hardware serial port
  Serial.begin( baudrate_is_supported(config.io.baudrate)
                ? config.io.baudrate      // eeprom contained valid baudrate
                : 115200L                 // baudrate was invalid, use default
              );

  stepper.setMaxSpeed(1000);
}

void loop() {
  // buffer stores characters as they come in
  static char cmdbuffer[16];
  static char* pcmd = cmdbuffer;

  if(config.io.motor_mode == StepperMode) {
    switch(config.stepper.motion_mode) {
      case HoldingMode: break;
      case SpeedMode: stepper.runSpeed(); break;
      case SteppingMode: stepper.run(); break;
    }
  } else {
    // brushed motor mode
    drive_brushed();
  }
  
  
  while (Serial.available() > 0) {
    // read the incoming byte
    int c = Serial.read();

    if (c == '\r') {
      *pcmd = 0;  // append null

      // todo: we can probably convert this to a state machine model
      if (cmdbuffer[0] == '#') {
        // parse the LSS command
        LynxPacket pkt;
        if (pkt.parse(&cmdbuffer[1])) {
          if (pkt.id == LssBroadcastAddress)
            process_broadcast_packet(pkt);
          else
            process_packet(pkt);
        }
      }

      // clear packet buffer
      pcmd = cmdbuffer;
    } else if (c == '#') {
      // for now, we reset the buffer if we encounter a packet-start character
      pcmd = cmdbuffer;
      *pcmd++ = (char)c;
    } else {
      // add to cmd buffer
      *pcmd++ = (char)c;
    }
  }
}
