
#pragma once

#include "LynxmotionLSS-Config.h"
#include "LssCommon.h"

#if defined(LSS_STATS)
#include "aggregate.h"
#endif


#define BIT(n) (((unsigned long)1)<<n)

typedef unsigned long LssCommands;
typedef unsigned long LssModifiers;

#define  LssInvalid          0
#define  LssAction           BIT(1)
#define  LssQuery            BIT(2)
#define  LssConfig           BIT(3)
#define  LssCommandModes     (LssAction|LssQuery|LssConfig)

#define  LssDegrees          BIT(4)
#define  LssRPM              BIT(5)
#define  LssPulse            BIT(6)
#define  LssUnits            (LssDegrees|LssRPM|LssPulse)

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
#define  LssAngularHoldingStiffness BIT(31)
#define  LssOriginOffset     BIT(20)
#define  LssGyreDirection    BIT(21)
#define  LssLEDColor         BIT(22)
#define  LssCurrent          BIT(23)
#define  LssVoltage          BIT(24)
#define  LssTemperature      BIT(25)
#define  LssFirstPosition    BIT(26)
#define  LssMotionControl    BIT(27)
#define  LssFilterPoleCount  BIT(28)
#define  LssDefault          BIT(29)
#define  LssConfirm          BIT(30)
#define  LssCommandSet       ((0xffffffffffff & ~(LssCommandModes|LssUnits)) | LssQuery)

// modifiers
#define  LssModTimedMove            BIT(2)
#define  LssModSpeed                BIT(3)
#define  LssModCurrentHaltAndHold   BIT(4)
#define  LssModCurrentHaltAndLimp   BIT(5)
#define  LssModifiersSet        (LssTimedMove|LssSpeed|LssModCurrentHaltAndHold)

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
    unsigned long long microstamp;  // timestamp in microseconds the packet was last transmitted or received
    LssCommands command;
    LssModifiers modifiers;
    bool hasValue;
    int value;

    // modifier values
    int current;//, speed, timedMove;

    inline LynxPacket() : id(0), microstamp(0), command(LssInvalid), modifiers(0), hasValue(false), value(0) {}
    inline LynxPacket(short _id, LssCommands _command) : id(_id), microstamp(0), command(_command), modifiers(0), hasValue(false), value(0) {}
    inline LynxPacket(short _id, LssCommands _command, int _value) : id(_id), microstamp(0), command(_command), modifiers(0), hasValue(true), value(_value) {}

    LynxPacket(const char* pkt);

    bool operator==(const LynxPacket& rhs) const;

    inline void set(int _value) { value=_value; hasValue=true; }

    inline LynxPacket& currentHaltAndHold(int _current) { modifiers |= LssModCurrentHaltAndHold; current = _current; return *this; }
    inline LynxPacket& currentHaltAndLimp(int _current) { modifiers |= LssModCurrentHaltAndLimp; current = _current; return *this; }

    bool parse(const char* pkt);

    char* serialize(char* out) const;

	inline bool matches(LssCommands bits) const { return (command & bits) == bits; }

    static LssCommands parseCommand(const char*& pkt);
    
    // converts the cmd into a string command code and places the result in 'out'
    // returns the end of the command code string within the 'out' memory, or NULL if an error
    static char* commandCode(LssCommands cmd, char* out);

    // converts
    static char* modifierCode(LssModifiers mods, char* out);


#if defined(HAVE_STRING)
    String toString() const;
#endif

private:
    int readValue(const char*& pkt, bool& _hasValue);
};
