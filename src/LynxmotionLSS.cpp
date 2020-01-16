
#include "LynxmotionLSS.h"

#include <limits.h>

#if defined(LSS_OSCOPE_TRIGGER_PIN)
#define OSCOPE_TRIGGER(ex) oscope_trigger(ex);
void oscope_trigger(short exception_code)
{
  // flip pin state to trigger oscope
  digitalWrite(LSS_OSCOPE_TRIGGER_PIN, HIGH);
  delayMicroseconds(500);
  digitalWrite(LSS_OSCOPE_TRIGGER_PIN, LOW);
  delayMicroseconds(250);
  Serial.print("TRG@");
  Serial.println(exception_code);
}
#else
#define OSCOPE_TRIGGER(ex)
inline void oscope_trigger(short exception_code) {} // nop
#endif


LynxChannel::LynxChannel(const char* channel_name) 
  : name(channel_name), serial(NULL), pbuffer(NULL), timeout_usec(TRANSACTION_TIMEOUT), unresponsive_request_limit(unresponsive_request_limit), unresponsive_disable_interval(UNRESPONSIVE_DISABLE_INTERVAL),
    size(0), count(0), servos(NULL),
    txn_current(1), txn_next(1)
{}

void LynxChannel::begin(Stream& _stream)
{
  serial = &_stream;
  pbuffer = buffer;
}

void LynxChannel::update()
{
  if(serial==NULL || pbuffer==NULL) return;

  // allow servos to update
  //short lowest_txn = 0;
  unsigned long _txn_current = txn_current;
  for(int i=0; i<count; i++) {
    servos[i]->update();
    //if (servos[i]->mask.txn < lowest_txn)
    //  lowest_txn++;
  }

  // if nothing pending, then we can equalize txn number
  //if (pending == 0)
  //  txn_current = txn_next;

  // if txn_current changed, then we can queue up the next servo writes
  // todo: maybe there is a better way to detect this, perhaps by detecting in the above loop (for now KISS)
  if(_txn_current != txn_current) {
    for(int i=0; i<count; i++)
      if(servos[i]->mask.txn == txn_current)
        servos[i]->update();
  }

  // process input from stream (serial)
  while(serial->available() >0) {
    char c = serial->read();
    if (c == '*') {
      // start of packet, we shouldnt have anything in the packet buffer
      // if we do, then it is junk, probably bus corruption, and we will have to discard it
#if defined(LSS_LOGGING)
      if (pbuffer != buffer) {
        *pbuffer = 0;
        LSS_LOGGING.print("JUNK ");
        LSS_LOGGING.println(buffer);
      }
#endif
      pbuffer = buffer;
      *pbuffer++ = '*';
    }
    else if (c=='\r') {
      // process packet
      *pbuffer = 0;
#if defined(LSS_LOGGING) && defined(LSS_LOG_PACKETS)
      LSS_LOGGING.print("<< ");
      LSS_LOGGING.println(buffer);
#endif
      if (buffer[0] == '*') {
        // dispatch packet to destination servo (if we have it)
        LynxPacket packet(&buffer[1]);
        for(int i=0; i<count; i++) {
          if(servos[i]->id == packet.id) {
            servos[i]->dispatch(packet);
            break;
          }
        }    
      }
      pbuffer = buffer; // reset buffer insert position
    } else {
      // add to buffer
      *pbuffer++ = c;
    }
  }
}

LynxChannel& LynxChannel::add(LynxServo& servo)
{
  if(count >= size)
    alloc(size + 5);
  servo.channel = this;
  servos[count++] = &servo;
  return *this;    
}

bool LynxChannel::contains(short servoId) const 
{
  for(int i=0; i<count; i++)
    if(servos[i]->id == servoId) return true;
  return false;
}

LynxServo& LynxChannel::operator[](short servoId) {
  for(int i=0; i<count; i++)
    if(servos[i]->id == servoId) 
      return *servos[i];
  // we halt here because there is no appropriate response for a failed lookup! (returning a ref)
#if defined(LSS_LOGGING)
  LSS_LOGGING.print("LynxChannel::operator[");
  LSS_LOGGING.print(servoId);
  LSS_LOGGING.print("] called on non-existent servo");
#endif
  while(1);
}

const LynxServo& LynxChannel::operator[](short servoId) const {
  for(int i=0; i<count; i++)
    if(servos[i]->id == servoId) 
      return *servos[i];
  // we halt here because there is no appropriate response for a failed lookup! (returning a ref)
#if defined(LSS_LOGGING)
  LSS_LOGGING.print("LynxChannel::operator[");
  LSS_LOGGING.print(servoId);
  LSS_LOGGING.print("] called on non-existent servo");
#endif
  while(1);
}

AsyncToken LynxChannel::ReadAsyncAll(LssCommands commands)
{
  if(count>0) {
    AsyncToken rt;
    for (int i = 0; i < count; i++) {
      AsyncToken t = servos[i]->ReadAsync(commands);
      if (t.isActive())
        rt = t; // only accept if we got a valid active token, otherwise the servo is ignored and AsyncAll does its best
    }
    return rt;  // return the last active token, which will be the last token to finish and thus finishing our AsyncAll
  } else
    return AsyncToken();
}

bool LynxChannel::waitFor(const AsyncToken& token)
{
  unsigned long timeout = micros() + timeout_usec;
  unsigned long _txn = txn_current;
  while( micros() < timeout && token.isActive()) {
    if(_txn != txn_current) {
      // a servo responded, reset timeout
      timeout = micros() + timeout_usec;
      _txn = txn_current;
    }

    update(); // this call will process incoming data and update the servos
  }
  return token.isComplete();
}

void LynxChannel::send(const LynxPacket& p)
{
  char buf[64];
  char* pend = buf;
  *pend++ = '#';
  if((pend=p.serialize(pend)) !=NULL) {
    char* pbegin = buf;
#if defined(LSS_LOGGING) && defined(LSS_LOG_PACKETS)
      LSS_LOGGING.print(">> ");
      LSS_LOGGING.println(pbegin);
#endif
    *pend++ = '\r';
    *pend=0;
    serial->print(pbegin);
    
#if INTERPACKET_DELAY>0
    delay(INTERPACKET_DELAY);
#endif
  }
}

short LynxChannel::scan(short beginId, short endId)
{
  // array to keep track of discovered devices
  short N = 0;
  short discovered[endId - beginId + 1];
  memset(discovered, 0, sizeof(discovered));

  // build a channel attached to same stream as current, but we will only add 1 servo
  LynxChannel sub_channel(this->name);
  LynxServo servo(beginId);

  // set timeout much lower
  sub_channel.timeout_usec = 2000000UL;

  // add a single servo and we will keep incrementing the ID
  servo.stats = NULL;       // dont bother collecting servo stats
  sub_channel.add(servo);   // add our scan servo to the scan channel

  // iterate each ID on our single scan servo
  sub_channel.begin(*this->serial);
  while (servo.id <= endId) {
    AsyncToken t = servo.ReadAsync(LssQuery);
    if (sub_channel.waitFor(t)) {
      // found a servo
      discovered[N++] = servo.id;
    }

    servo.id++; // scan next ID
    servo.timeouts = 0;
    servo.enableAfter_millis = 0;
  }

  // clear all servos (remove the servo we temporarily used to enumerate)
  sub_channel.free();

  // create servos of all discovered
  // for scans, the channel will own and must free the created LynxServo's
  if(N>0)
    create(discovered, N);
  return N;
}

LynxChannel::~LynxChannel() 
{ 
  free();
}

void LynxChannel::alloc(short n)
{
  if (servos != NULL) {
    servos = (LynxServo**)realloc(servos, n*sizeof(LynxServo*));
  }
  else {
    servos = (LynxServo**)calloc(n, sizeof(LynxServo*));
    count = 0;
  }
  size = n;
}

void LynxChannel::free()
{
  if(servos) {
    for (int i = 0; i < count; i++) {
      if (servos[i] && servos[i]->channel_owned)
        delete servos[i];
    }
    ::free(servos);
    servos = NULL;
  }
  count=0;
}

//void * operator new (size_t size, void * ptr) { return ptr; }

void LynxChannel::create(const short* ids, short N)
{
  short* shadowed_ids = (short*)calloc(N, sizeof(short));
  short shadowed_N = N;

  memcpy(shadowed_ids, ids, N*sizeof(short));

  // clear out any servos that already exist
  if (count > 0) {
    // todo: this could be more efficient if continuous servo addition or bus-scanning really matters
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < count; j++) {
        if (servos[j] && ids[i] && ids[i] == servos[j]->id) {
          // squelch adding this existing servo ID
          shadowed_ids[i] = 0;
          shadowed_N--;
        }
      }
    }
  }

  if (shadowed_N > 0) {
    // allocate new servos
    alloc(count + shadowed_N);
    for (int i = 0; i < N; i++) {
      if (shadowed_ids[i] > 0) {
        //LynxServo* servo = new (&_servos[i]) LynxServo(shadowed_ids[i]);
        LynxServo* servo = new LynxServo(shadowed_ids[i]);
        servo->channel = this;
        servo->channel_owned = true;  // mark this object for ::free when channel deallocates
        servos[count++] = servo;
      }
    }
  }

  // free shadow mem
  ::free(shadowed_ids);
}

bool LynxPacket::operator==(const LynxPacket& rhs) const
{
  return id==rhs.id && command==rhs.command && hasValue==rhs.hasValue && (!hasValue || (value==rhs.value));
}

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
AsyncToken LynxServo::ReadAsync(LssCommands commands)
{
  if (!isEnabled()) 
    return (enableAfter_millis>0)
      ? AsyncToken::Unresponsive()  // in unresponsive period
      : AsyncToken();               // must be attached to a channel

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

  return AsyncToken(mask);
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
      OSCOPE_TRIGGER(3)
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
      else DISPATCH(LssLEDColor, config->ledColor)
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
