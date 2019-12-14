
#pragma once

#include "LynxmotionLSS-Config.h"
#include "LssCommunication.h"
#include "LssChannel.h"
#include "AsyncToken.h"
#include "MaskSet.h"

#if defined(LSS_STATS)
#include "analytics/aggregate.h"
#endif


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
    LssChannel* channel;
    
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
    LynxServo(short _ID=0, int _units=LssDegrees, Statistics* _stats=NULL);
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
//    MaskSet::Promise  ReadAsync(LssCommands commands);
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
