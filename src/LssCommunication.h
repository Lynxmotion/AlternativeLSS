
#pragma once

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif
#include <WString.h>

#include "LynxmotionLSS-Config.h"

#if defined(LSS_STATS)
#include "aggregate.h"
#endif

#define LssBroadcastAddress   (254)

#define BIT(n) (((unsigned long)1)<<n)

typedef unsigned long LssCommands;

#define  LssInvalid          0
#define  LssAction           BIT(1)
#define  LssQuery            BIT(2)
#define  LssConfig           BIT(3)
#define  LssCommandModes     (LssAction|LssQuery|LssConfig)

#define  LssDegrees          BIT(4)
#define  LssRPM              BIT(5)
#define  LssPulse            BIT(6)
#define  LssUnits            (LssDegrees|LssRPM|LssPulse)

  // modifiers
#define  LssTimedMove        BIT(11)
#define  LssSpeed            BIT(15)
#define  LssModifiers        (LssTimedMove|LssSpeed)

#define  LssID               BIT(7)
#define  LssBaudRate         BIT(8)
#define  LssLimp             BIT(9)
#define  LssHaltAndHold      BIT(10)
#define  LssMove             BIT(12)
#define  LssPosition         BIT(13)
#define  LssTarget           BIT(14)
#define  LssWheelMode        BIT(16)
#define  LssMaxSpeed         BIT(17)
#define  LssAngularRange     BIT(18)
#define  LssAngularStiffness BIT(19)
#define  LssOriginOffset     BIT(20)
#define  LssGyreDirection    BIT(21)
#define  LssLEDColor         BIT(22)
#define  LssCurrent          BIT(23)
#define  LssVoltage          BIT(24)
#define  LssTemperature      BIT(25)
#define  LssFirstPosition    BIT(26)
#define  LssDefault          BIT(27)
#define  LssConfirm          BIT(28)
#define  LssAnalog           BIT(29)
#define  LssReset            BIT(30)
#define  LssModel            BIT(31)
#define  LssCommandSet       ((0xffffffff & ~(LssCommandModes|LssUnits|LssModifiers)) | LssQuery)


// commands that are part of servo configuration
// you probably dont need these for normal control operations
#define  LssConfigCommandSet  ( \
       LssWheelMode \
      |LssMaxSpeed \
      |LssAngularRange \
      |LssAngularStiffness \
      |LssLEDColor \
      |LssBaudRate \
      |LssGyreDirection \
      |LssOriginOffset \
      |LssID )

  // commands that support asynchronous queries
#define  LssAsyncCommandSet  ( \
       LssQuery \
      |LssPosition \
      |LssTarget \
      |LssMaxSpeed \
      |LssVoltage \
      |LssCurrent \
      |LssConfigCommandSet )


typedef enum {
  LssLedOff  = 0,
  LssRed     = 1,
  LssGreen   = 2,
  LssBlue    = 3,
  LssYellow  = 4,
  LssCyan    = 5,
  LssMagenta = 6,
  LssWhite   = 7
} LssColors;


class LynxPacket {
  public:
    short id;
    LssCommands command;
    bool hasValue;
    long value;

    inline LynxPacket() : id(0), command(LssInvalid), hasValue(false), value(0) {}
    inline LynxPacket(short _id, LssCommands _command) : id(_id), command(_command), hasValue(false), value(0) {}
    inline LynxPacket(short _id, LssCommands _command, long _value) : id(_id), command(_command), hasValue(true), value(_value) {}

    LynxPacket(const char* pkt);

    bool operator==(const LynxPacket& rhs) const;

    inline void set(long _value) { value=_value; hasValue=true; }

    bool parse(const char* pkt);

    char* serialize(char* out) const;

	inline bool matches(LssCommands bits) const { return (command & bits) == bits; }

	inline bool between(long min, long max) const { return hasValue && value >= min && value <= max; }

	inline bool broadcast() const { return id == 254; }

	// true if command is a query command
	inline bool query() const { return (command & LssQuery) >0; }

	// true if command requests value be written to flash (Config prefix)
	inline bool flash() const { return (command & LssConfig) >0; }

    static LssCommands parseCommand(const char*& pkt);
    
    // converts the cmd into a string command code and places the result in 'out'
    // returns the end of the command code string within the 'out' memory, or NULL if an error
    static char* commandCode(LssCommands cmd, char* out);

#if defined(HAVE_STRING)
    String toString() const;
#endif
};
