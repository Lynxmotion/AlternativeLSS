
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

#include "LssCommunication.h"
class LynxServo;



class MaskSet 
  // TODO: rename to AsyncTransaction
{
  public:
    unsigned long txn;  // current transmission number for this request
    LssCommands read;
    LssCommands requested;
    LssCommands completed;

    unsigned long timestamp;      // millis when first packet was sent
    unsigned long expire;         // timestamp when the read request will be considered expired

    inline MaskSet(unsigned long _txn = 0, LssCommands _read = LssInvalid) : txn(_txn), read(_read), requested(0), completed(0), timestamp(0), expire(0) {}

    inline bool isActive() const { return txn!=0; }
    inline bool isComplete() const { return read == completed; }
	  inline bool isUnresponsive() const { return completed==0; }
};

class AsyncToken
{
  public:
    AsyncToken() : set(NULL) {}
	  AsyncToken(const AsyncToken& copy) : set(copy.set) {}
	  AsyncToken(const MaskSet& _set) : set(&_set) {}

	  inline AsyncToken& operator=(const AsyncToken& t) { set = t.set; return *this; }
	  inline AsyncToken& operator=(const MaskSet& s) { set = &s;  return *this; }

	  // clear the async token
	  inline void clear() { set = NULL;  }

    inline bool isValid() const { return set!=NULL; }
    inline bool isActive() const { return set && set->isActive(); }
    inline bool isComplete() const { return set && set->isComplete(); }
	  inline bool isUnresponsive() const { return set && set->isUnresponsive(); }
  
    // returns an async token that indicates an unresponsive device/servo
    static AsyncToken Unresponsive() {
      static MaskSet um(0, LssQuery);
      return AsyncToken(um);
    }

protected:
    const MaskSet* set;
};

class LynxChannel {
  // They say classes shouldnt be friends, but then why bother have the friend feature? Besides, I
  // will take performance and simplicity over they-say for embedded systems.
  friend class LynxServo;
  
  public: // todo: should be private
    const char* name;
    Stream* serial;
    char buffer[32];
    char* pbuffer;

    // parameters
    unsigned long timeout_usec;     // time we will wait for a servo to respond (in microseconds)
    short unresponsive_request_limit;       // if a servo fails to respond to this number of packets in a row, we consider it unresponsive
    unsigned long unresponsive_disable_interval;  // amount of milliseconds a servo gets disabled when found unresponsive

    // servo collection
    short size;
    short count;
    LynxServo** servos;

    // packet queue
    unsigned long txn_current;  // transmission number we are sending now
    unsigned long txn_next;     // next transmission number we will assign
    
  public:
    LynxChannel(const char* channel_name=NULL);
    virtual ~LynxChannel();

    void free();

    void begin(Stream& serial);
    void update();

    LynxChannel& add(LynxServo& servo);

    bool contains(short servoId) const;
    const LynxServo& operator[](short servoId) const;
    LynxServo& operator[](short servoId);

    short scan(short beginId, short endId);

    void create(const short* ids, short N);

    template<size_t N>
    inline void create(const short (&ids)[N]) { create(ids, N); }

    // perform a read 
    AsyncToken ReadAsyncAll(LssCommands commands);

    // wait for an async operation to finish
    bool waitFor(const AsyncToken& token);
    
    void send(const LynxPacket& p);

  protected:
    
    void alloc(short n);

};

typedef struct _LssServoConfig {
	short firstPosition;
	byte gyreDirection;
	int baudrate;
	LssColors ledColor;
	short angularStiffness;
	short angularRange;
	short maxSpeed;
	byte wheelMode;
  char firmware[16];
} LssServoConfig;


class LynxServo {
  /* TODO
   *  * Implement improved servo timeouts
   *      . Track last N time-to-first-packet response-times (TTFR) as a rolling average (say N=5)
   *      . Each transaction has it's timeout set to TTFR plus a factor  (so we quickly detect timeouts)
   *      . On X timeouts we disable servo for Y millis, and/or
   *      . On timeout we reset timeout threshold (TOTH) to high value, but as TTFR drops we decay the timeout threshold based on a decay factor of TOTH-TTFR
   */
  public:
    short id;
    LynxChannel* channel;
    
  public:
    bool channel_owned; // if true, the servo object memory is owned by the channel (ex. discovered via bus scan)

    // default units for commands
    LssCommands units;
    unsigned long timeout_usec;

    // if a servo becomes unresponse, we mark if as disabled
    mutable short timeouts;
    mutable unsigned long enableAfter_millis; // at what millis() time we will allow a retry

    short state;
    short position;
    short target;
    short speed;
    short current;
    short voltage;
    short temperature;

    // warning: this 'config' variable is typically NULL.
    // config settings are not typically meddled with during normal servo use,
    // so lets store these settings in an external object (that isnt created unless necessary)
	  LssServoConfig* config;

    // variables used during async reads
    MaskSet mask;

  public:
#if defined(LSS_STATS)
    // supplying NULL to _stats argument will use a global shared Servo stats object (aggregated servo stats)
    class Statistics;
    LynxServo(short _ID, int _units=LssDegrees, Statistics* _stats=NULL);
#else
    LynxServo(short _ID, int _units=LssDegrees);
#endif

    bool isEnabled() const;

    inline bool isResponsive() const { return timeouts==0; }

    inline bool isUnresponsive() const { return enableAfter_millis > 0; }

    void Write(LssCommands cmd);
    void Write(LssCommands cmd, int value);

    void WritePosition(short p);

    // initiate an asynchronous read of one or more servo registers
    AsyncToken ReadAsync(LssCommands commands);
    void ClearAsync(LssCommands commands);

    void dispatch(LynxPacket pkt);

    void update();


#if defined(LSS_STATS)
  class Statistics {
    public:
      struct {
        unsigned long transmits;
        unsigned int queries;        // transmits that expect a response
        unsigned int retransmits;    // if we detect a missing response we can attempt a retransmit before a timeout occurs
        unsigned int received;
      } packet;

      // a transaction is a group of packets sent at once, and we expect the replies from the servo in the same order
      struct {
        unsigned long complete;       // total number of transactions that were completed without error
        unsigned int partials;       // number of transactions that ended in a partial reply but with missing packets and eventually hit timeout
        unsigned int timeouts;       // servo didnt respond at all (does not include partial replies)
        Aggregate<unsigned long, unsigned long long> responseTime;
        Aggregate<unsigned long, unsigned long long> completionTime;
      } transaction;

      Statistics();
  };
  Statistics* stats;
  static const Statistics& globalStatistics();
#endif    

};
