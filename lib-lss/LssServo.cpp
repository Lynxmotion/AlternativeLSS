
#include "LssServo.h"

#include <limits.h>


LynxServo::Statistics::Statistics()
{
  memset(&packet, 0, sizeof(packet));
  memset(&transaction, 0, sizeof(transaction));
}


#ifdef LSS_STATS
// our shared aggregated servo stats
LynxServo::Statistics global_servo_stats;
const LynxServo::Statistics& LynxServo::globalStatistics() { return global_servo_stats; }
#endif

#ifdef LSS_STATS
LynxServo::LynxServo(short _ID, int _units, Statistics* _stats)
#else
LynxServo::LynxServo(short _ID, int _units)
#endif
  : id(_ID), channel_owned(false), channel(NULL), units(LssDegrees), timeout_usec(TRANSACTION_TIMEOUT), timeouts(0), enableAfter_millis(0),
    state(0), position(1800), target(1800), speed(0), current(120), voltage(124), temperature(235), config(NULL)
#ifdef LSS_STATS
    , stats( _stats ? _stats : &global_servo_stats)
#endif
{
    memset(&mask, 0, sizeof(MaskSet));
}

bool LynxServo::isEnabled() const
{
  if (!channel) return false;
  if (enableAfter_millis > 0) {
    if (enableAfter_millis < millis()) {
      // give this servo another chance!
      timeouts = channel->unresponsive_request_limit -1;
      enableAfter_millis = 0;
      return true;  // indicate enabled
    }
    else
      return false; // servo is considered unresponsive
  }
  return true;  // all good, servo is enabled for comms
}

void LynxServo::WritePosition(short p)
{
  Write(LssPosition|units, p);
}

void LynxServo::Write(LssCommands cmd)
{
  if(isEnabled()) {
#if defined(LSS_STATS)
    if(stats) {
      stats->packet.transmits++;
      if(cmd & LssQuery)
        stats->packet.queries++;
    }
#endif

    if((cmd & (LssPosition|LssWheelMode|LssMaxSpeed|LssFirstPosition))>0)
      cmd |= units;

    // ensure it is our turn to send
    channel->send(LynxPacket(id, cmd));
  }
}

void LynxServo::Write(LssCommands cmd, int value)
{
  if (isEnabled()) {
#if defined(LSS_STATS)
    if(stats) {
      stats->packet.transmits++;
      if(cmd & LssQuery)
        stats->packet.queries++;
    }
#endif
    channel->send(LynxPacket(id, cmd, value));
  }
}

void LynxServo::ClearAsync(LssCommands commands)
{
    memset(&mask, 0, sizeof(MaskSet));
}

// initiate an asynchronous read of one or more servo registers
MaskSet::Promise LynxServo::ReadAsync(LssCommands commands)
{
  if (!isEnabled()) {
      MaskSet::Promise p;
      p.reject();
      return p;  // must be attached to a channel
  }

  // ensure we only have supported async commands
  commands &= LssAsyncCommandSet;
  
  //if (mask.txn == 0 || mask.read == mask.completed) {   // old way would halt servo if TO occured since mask.txn!=0 and mask.read!=completed
  //if (mask.txn != channel->txn_current || mask.read==mask.completed) {
  if (mask.txn ==0) {
    // setup a new read transaction
    memset(&mask, 0, sizeof(MaskSet));
    mask.txn = channel->txn_next++;
  }

  mask.read |= commands;

  // call update to send if we have the clear-to-send txn token
  update();

  return channel->on(mask);
}

#define SENDIF(lssbit) if((unsent & lssbit)>0) { /*LSS_LOGGING.print("Send " #lssbit); LSS_LOGGING.print("  "); LSS_LOGGING.print(channel->txn_current);  LSS_LOGGING.print("  "); LSS_LOGGING.println(isAsyncComplete() ? "Complete":"Pending");*/ mask.requested |= lssbit; Write(LssQuery|lssbit); }
void LynxServo::update()
{
  if(!isEnabled()) return;
  if(channel->txn_current == mask.txn) {
    // set the commands to read (or add to the list)
    unsigned long now = micros();
    
    if(mask.requested ==0) {
      // this is the first request we are sending
      mask.timestamp = now;
      mask.expire = now + timeout_usec;
    } else if(now > mask.expire && mask.completed!=mask.read) {
      // this transaction is expired (timed out)
      OSCOPE_TRIGGER(3, "PKT-TO")
      channel->txn_current++;
      mask.txn = 0;
      timeouts++;
      if (timeouts >= channel->unresponsive_request_limit) {
        // consider servo unresponsive and disable
        enableAfter_millis = millis() + channel->unresponsive_disable_interval;
      }
      //Serial.println("expired");
#if defined(LSS_STATS)
      if(stats) {
        if(mask.completed>0)
          stats->transaction.partials++;
        else  
          stats->transaction.timeouts++;
      }
#endif
#if defined(LSS_LOGGING)
      LSS_LOGGING.print('S');
      LSS_LOGGING.print(id);
      LSS_LOGGING.print(' ');
      LSS_LOGGING.print(channel->txn_current);
      LSS_LOGGING.print(' ');
      LSS_LOGGING.print(channel->txn_next);
      LSS_LOGGING.println(" TO");
#endif
      return;
    }
    
    // send any command queries that weren't already sent
    unsigned long unsent = mask.read & ~mask.requested;
    if(unsent>0) {
      // todo: maybe we should send these one at a time by prepending an 'else' before each SENDIF? However, it will cause unittests to fail that depend on channel.update()
      SENDIF(LssQuery)
      SENDIF(LssPosition)
      SENDIF(LssTarget)
      SENDIF(LssWheelMode)
      SENDIF(LssMaxSpeed)
      SENDIF(LssVoltage)
      SENDIF(LssCurrent)
      SENDIF(LssTemperature)
      SENDIF(LssAngularRange)
      SENDIF(LssAngularStiffness)
      SENDIF(LssLEDColor)
      SENDIF(LssBaudRate)
      SENDIF(LssGyreDirection)
      SENDIF(LssOriginOffset)  
    } 
    
    if(mask.txn>0 && mask.completed && (mask.completed == mask.read) && (mask.txn == channel->txn_current)) {
      // todo: this code never seems to run because the completion is detected in dispatch, possibly remove but keep for now for safety
      // we finished, signal the channel we are done if required
#if defined(LSS_STATS)
      if(stats) {
        unsigned long elapsed = (mask.timestamp < now) 
          ? (now - mask.timestamp) 
          : (ULONG_MAX - (mask.timestamp-now)); // micros overflowed
        stats->transaction.complete++;
        stats->transaction.completionTime.add(elapsed);     // elapsed is total transaction time
      }
#endif      
      channel->txn_current++;
      mask.txn = 0;
    } //else LSS_LOGGING.println(isAsyncComplete() ? "  Complete":"  Pending");    
  }
}

#if defined(LSS_LOGGING) && defined(LSS_LOG_SERVO_DISPATCH)
#define DISPATCH(bit, member) if ((cmd & bit)>0) { LSS_LOGGING.print("  " #bit " "); LSS_LOGGING.print(pkt.value); LSS_LOGGING.print(" => " #member); member = pkt.value; mask.completed |= bit; }
#else
#define DISPATCH(bit, member) if ((cmd & bit)>0) { member = pkt.value; mask.completed |= bit; }
#define DISPATCH_CAST(bit, type, member) if ((cmd & bit)>0) { member = (type)pkt.value; mask.completed |= bit; }
#endif

void LynxServo::dispatch(LynxPacket pkt)
{
  timeouts = 0; // we got a response, reset the timeout counter

#if defined(LSS_LOGGING) && defined(LSS_LOG_SERVO_DISPATCH) // channel already logs incoming packets
  char s[12];
	LSS_LOGGING.print('S');
  LSS_LOGGING.print(id);
  LSS_LOGGING.print(' ');
  LynxPacket::commandCode(pkt.command, s);
  LSS_LOGGING.print(s);
  if (pkt.hasValue) {
    LSS_LOGGING.print(" V");
    LSS_LOGGING.print(pkt.value);
  }
#endif

#if defined(LSS_STATS)
  unsigned long now = micros();
  unsigned long elapsed = (mask.timestamp < now) 
    ? (now - mask.timestamp) 
    : (ULONG_MAX - (mask.timestamp-now)); // micros overflowed
  if(stats) {
    stats->packet.received++;
    if(mask.read>0 && mask.completed==0) {
      // first packet received 
      stats->transaction.responseTime.add(elapsed);   // elapsed is time-to-first-packet
    }
  }
#endif
  
  if (pkt.command == LssQuery)
  {
    // handle LssQuery on its own
    LssCommands cmd = pkt.command;
    DISPATCH(LssQuery, state);
  }
  else if ((pkt.command && LssQuery) > 0) {
    // handle when LssQuery is paired with another specific command
    LssCommands cmd = pkt.command & (LssCommandSet & ~LssQuery);
    DISPATCH(LssPosition, position)
    else DISPATCH(LssTarget, target)
    else DISPATCH(LssVoltage, voltage)
    else DISPATCH(LssCurrent, current)
    else DISPATCH(LssTemperature, temperature)
    else if ((cmd & LssConfigCommandSet) > 0) {
      if (config == NULL) {
        config = (LssServoConfig*)calloc(1, sizeof(LssServoConfig));
      }
      DISPATCH(LssFirstPosition, config->firstPosition)
      else DISPATCH(LssGyreDirection, config->gyreDirection)
      else DISPATCH(LssBaudRate, config->baudrate)
      else DISPATCH_CAST(LssLEDColor, LssColors, config->ledColor)
      else DISPATCH(LssAngularStiffness, config->angularStiffness)
      else DISPATCH(LssMaxSpeed, config->maxSpeed)
      else DISPATCH(LssAngularRange, config->angularRange)
      else DISPATCH(LssWheelMode, config->wheelMode)
      //else DISPATCH(LssID, id);		// what to do here? We should update ID member which could fubar erverything?
    }
  }
#if defined(LSS_LOGGING) && defined(LSS_LOG_SERVO_DISPATCH)
  LSS_LOGGING.println();
#endif

  /*Serial.print('>');
  Serial.print(mask.read);
  Serial.print('|');
  Serial.print(mask.completed);
  Serial.print('=');
  Serial.print(mask.read & ~mask.completed);
  Serial.print("  (");
  Serial.print(mask.txn);
  Serial.print('/');
  Serial.print(channel->txn_current);
  Serial.println(')');*/
  if(mask.txn>0 && mask.completed && (mask.completed == mask.read) && (mask.txn == channel->txn_current)) {
#if defined(LSS_STATS)
    if(stats) {
      stats->transaction.complete++;
      stats->transaction.completionTime.add(elapsed);     // elapsed is total transaction time
    }
#endif
    // we finished, signal the channel we are done if required
    channel->txn_current++;
    mask.txn = 0;
  }
}
