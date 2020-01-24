
#include "LssCommunication.h"

#define ACCEPT(cmdid) return (LssCommands)(cmdid);
#define SWITCH(cmdid)  if(*pkt==0 || !isalpha(*pkt)) ACCEPT(cmdid) else switch (toupper(*pkt++))

LssCommands LynxPacket::parseCommand(const char*& pkt) 
{
  /*  This code might be a bit confusing but it is much faster than a bunch of string
   *  comparisons. This is technically called a "Trie" or prefix tree.
   *  https://en.wikipedia.org/wiki/Trie
   *  
   *  To  simplify the code I use the above two macros SWITCH and ACCEPT:
   *  ACCEPT(cmdid) -- immediately return with a parsed command value of cmdid.
   *  SWITCH(cmdid) -- test if next character is a stop char (null char or non-alpha 
   *                   char) and if so accept it by immediately returning cmdid. If
   *                   not, then test the character as the next character in the Trie.
   *                   I.e. the next character in the reduced set of commands. 
   *                   
   *  NOTE The SWITCH(cmdid) macro uses standard switch() logic for testing characters, 
   *  but don't confuse the macro argument for the argument of a standard switch(), the 
   *  argument to SWITCH is the cmdid if we find a stop char and SWITCH is internally 
   *  aware the char to test is *pkt.
   */
  const char* keep_ptr = pkt;
  SWITCH(LssInvalid) {
    case 'L': SWITCH(LssLimp) {
      case 'E': SWITCH(LssInvalid) {
        case 'D': ACCEPT(LssLEDColor);
      }
    }
    case 'H': ACCEPT(LssHaltAndHold);
    case 'O': ACCEPT(LssOriginOffset);
    case 'P': ACCEPT(LssPosition|LssPulse);
    case 'D': SWITCH(LssPosition|LssDegrees) {
      case 'E': SWITCH(LssInvalid) {
      case 'F': SWITCH(LssInvalid) {
      case 'A': SWITCH(LssInvalid) {
      case 'U': SWITCH(LssInvalid) {
      case 'L': SWITCH(LssInvalid) {
      case 'T': ACCEPT(LssDefault);
      }}}}}
    }
    case 'A': SWITCH(LssInvalid) {
      case 'R': ACCEPT(LssAngularRange);
      case 'S': ACCEPT(LssAngularStiffness);
    }
    case 'M': SWITCH(LssInvalid) {
      case 'D': ACCEPT(LssMove|LssDegrees);
    }
    case 'W': SWITCH(LssInvalid) {
      case 'D': ACCEPT(LssWheelMode|LssDegrees);
      case 'R': ACCEPT(LssWheelMode|LssRPM);
    }
    case 'B': ACCEPT(LssBaudRate);
    case 'G': ACCEPT(LssGyreDirection);
    case 'S': SWITCH(LssInvalid) {
      case 'D': ACCEPT(LssMaxSpeed|LssDegrees);
      case 'R': ACCEPT(LssMaxSpeed|LssRPM);
    }      
    
    case 'Q': SWITCH(LssQuery) {
      case 'O': ACCEPT(LssQuery|LssOriginOffset);
      case 'A': SWITCH(LssQuery|LssAnalog) {
        case 'R': ACCEPT(LssQuery|LssAngularRange);
        case 'S': ACCEPT(LssQuery|LssAngularStiffness);
      }
      case 'M': SWITCH(LssInvalid) {
        case 'S': ACCEPT(LssQuery|LssModel);
	  }
      case 'P': ACCEPT(LssQuery|LssPosition|LssPulse);
      case 'D': SWITCH(LssQuery|LssPosition|LssDegrees) {
        case 'T': ACCEPT(LssQuery|LssTarget);
      }
      case 'W': SWITCH(LssInvalid) {
        case 'D': ACCEPT(LssQuery|LssWheelMode|LssDegrees);
        case 'R': ACCEPT(LssQuery|LssWheelMode|LssRPM);
      }
      case 'S': SWITCH(LssInvalid) {
        case 'D': ACCEPT(LssQuery|LssMaxSpeed|LssDegrees);
        case 'R': ACCEPT(LssQuery|LssMaxSpeed|LssRPM);
      }
      case 'L': SWITCH(LssInvalid) {
        case 'E': SWITCH(LssInvalid) {
          case 'D': ACCEPT(LssQuery|LssLEDColor);
        }
      }
      case 'I': SWITCH(LssInvalid) {
        case 'D': ACCEPT(LssQuery|LssID);
      }
      case 'B': ACCEPT(LssQuery|LssBaudRate);
      case 'G': ACCEPT(LssQuery|LssGyreDirection);
      // FirstPOsition Pulse/Degrees
      // Midel, SerialNumber, FirmwareVersion
      case 'V': ACCEPT(LssQuery|LssVoltage);
      case 'T': ACCEPT(LssQuery|LssTemperature);
      case 'C': ACCEPT(LssQuery|LssCurrent);
    }

    case 'R': SWITCH(LssInvalid) {
      case 'S': ACCEPT(LssReset);
      case 'E': SWITCH(LssInvalid) {
      case 'S': SWITCH(LssInvalid) {
      case 'E': SWITCH(LssInvalid) {
      case 'T': ACCEPT(LssReset) {
      }}}}
    }

    case 'C': SWITCH(LssInvalid) {
      case 'O': SWITCH(LssConfig|LssOriginOffset) {
        case 'N': SWITCH(LssInvalid) {
        case 'F': SWITCH(LssInvalid) {
        case 'I': SWITCH(LssInvalid) {
        case 'R': SWITCH(LssInvalid) {
        case 'M': ACCEPT(LssConfirm);
        }}}}
      }
      case 'A': SWITCH(LssInvalid) {
        case 'R': ACCEPT(LssConfig|LssAngularRange);
        case 'S': ACCEPT(LssConfig|LssAngularStiffness);
      }
      case 'S': SWITCH(LssInvalid) {
        case 'D': ACCEPT(LssConfig|LssMaxSpeed|LssDegrees);
        case 'R': ACCEPT(LssConfig|LssMaxSpeed|LssRPM);
      }
      case 'L': SWITCH(LssInvalid) {
        case 'E': SWITCH(LssInvalid) {
          case 'D': ACCEPT(LssConfig|LssLEDColor);
        }
      }
      case 'I': SWITCH(LssInvalid) {
        case 'D': ACCEPT(LssConfig|LssID);
      }
      case 'B': ACCEPT(LssConfig|LssBaudRate);
      case 'G': ACCEPT(LssConfig|LssGyreDirection);
      // FirstPOsition Pulse/Degrees
    }
  }
  //OSCOPE_TRIGGER
  Serial.print("INVCMD ");
  Serial.println(keep_ptr);
  return LssInvalid;
}

char* LynxPacket::commandCode(LssCommands cmd, char* out) 
{
  char* pout = out;
  if((cmd & LssQuery) >0)
    *pout++ = 'Q';
  else if((cmd & LssConfig) >0)
    *pout++ = 'C';

  LssCommands unit = cmd & LssUnits;

  // filter out the member flag
  LssCommands member = cmd & (LssCommandSet & ~LssQuery); // LssQuery, that special command
  if(member != 0) {
      switch(member) {
        case 0: // LssQuery command
          break;
        case LssID:
          *pout++ = 'I';
          *pout++ = 'D';
          break;
        case LssLimp:
          *pout++ = 'L';
          break;
        case LssHaltAndHold:
          *pout++ = 'H';
          break;
        case LssPosition:
          *pout++ = (unit == LssPulse) ? 'P' : 'D';
          break;
        case LssTarget:
          *pout++ = 'D';
          *pout++ = 'T';
          break;
        case LssFirstPosition:
          *pout++ = 'F';
          *pout++ = (unit == LssPulse) ? 'P' : 'D';
          break;
        case LssWheelMode:
          *pout++ = 'W';
          *pout++ = (unit == LssRPM) ? 'R' : 'D';
          break;
        case LssMaxSpeed:
          *pout++ = 'S';
          *pout++ = (unit == LssRPM) ? 'R' : 'D';
          break;
        case LssVoltage:
          *pout++ = 'V';
          break;
        case LssCurrent:
          *pout++ = 'C';
          break;
        case LssTemperature:
          *pout++ = 'T';
          break;
        case LssAnalog:
          *pout++ = 'A';
          break;
        case LssAngularRange:
          *pout++ = 'A';
          *pout++ = 'R';
          break;
        case LssAngularStiffness:
          *pout++ = 'A';
          *pout++ = 'S';
          break;
        case LssLEDColor:
          *pout++ = 'L';
          *pout++ = 'E';
          *pout++ = 'D';
          break;
        case LssBaudRate:
          *pout++ = 'B';
          break;
        case LssGyreDirection:
          *pout++ = 'G';
          break;
        case LssOriginOffset:
          *pout++ = 'O';
          break;
        case LssDefault:
          *pout++ = 'D';
          *pout++ = 'E';
          *pout++ = 'F';
          *pout++ = 'A';
          *pout++ = 'U';
          *pout++ = 'L';
          *pout++ = 'T';
          break;
        case LssConfirm:
          *pout++ = 'C';
          *pout++ = 'O';
          *pout++ = 'N';
          *pout++ = 'F';
          *pout++ = 'I';
          *pout++ = 'R';
          *pout++ = 'M';
          break;
        case LssModel:
          *pout++ = 'M';
          *pout++ = 'S';
          break;
        default:
          // cannot serialize, unknown command code
          return NULL;
      }
  }
  
  *pout =0;
  return pout;
}

#if defined(HAVE_STRING)
String LynxPacket::toString() const {
  char buf[32];
  if(serialize(buf) !=NULL)
    return String(buf);
  return String();
}
#endif

char* LynxPacket::serialize(char* out) const
{
  // print ID efficiently
  unsigned char x = id;
  if(x>=100) {
      *out++ = '0'+(x/100);
      x %= 100;
      if(x<10)
        *out++ = '0';   // number is 2 digits, with a zero in the middle
  }
  if(x>=10) {
    *out++ = '0'+(x/10);
    *out++ = '0'+(x%10);
  } else {
    *out++ = '0'+x;
  }

  // print command code
  out = commandCode(command, out);
  if(out==NULL)
    return NULL;

  // use platform to convert value
  if(hasValue) {
    if(NULL == ltoa(value, out, 10))
      return NULL;
    while(*out) out++;  // skip to end
  } else
    *out=0;
  return out;
}

LynxPacket::LynxPacket(const char* pkt)
  : id(0), command(LssInvalid), hasValue(false), value(0)
{
  parse(pkt);
}

bool LynxPacket::parse(const char* pkt)
{
  // we parse into local variables and then set instance members
  // when we are sure we've successfully parsed.
  short _id=0;
  LssCommands _command=LssInvalid;
  bool _hasValue=false;
  long _value=0;
#if defined(LSS_LOGGING)
  const char* begin = pkt;
#endif

  if(!isdigit(*pkt))
    goto bad_read;
    
  // read ID
  while (*pkt && isdigit(*pkt)) {
      _id *= 10;
      _id += (short)(*pkt++ - '0');
  }

  _command = parseCommand(pkt);
  if(_command == LssInvalid)
    goto bad_read;

  if(isdigit(*pkt) || *pkt=='-') {
    bool isNegative = false;
    if(*pkt=='-') {
      isNegative=true;
      pkt++;
    }
    
    while (*pkt && isdigit(*pkt)) {
        _value *= 10;
        _value += (int)(*pkt++ - '0');
    }
    if(isNegative)
      _value *= -1;
    _hasValue = true;
  }

  id=_id;
  command = _command;
  hasValue = _hasValue;
  value = _value;
  
  return true;
    
bad_read:
#if defined(LSS_LOGGING)
  LSS_LOGGING.print("E@");
  LSS_LOGGING.print(pkt - begin);
  LSS_LOGGING.print(" ");
  while(begin <= pkt ) {
    if(isprint(*begin))
      LSS_LOGGING.print(*begin++);
    else
      LSS_LOGGING.print('[');
      LSS_LOGGING.print((short)*begin++, HEX);
      LSS_LOGGING.print(']');
  }
  LSS_LOGGING.println();
#endif
  return false;
}
