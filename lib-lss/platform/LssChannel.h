#pragma once

#include "../LynxmotionLSS-Config.h"
#include "../LssCommunication.h"
#include "../AsyncToken.h"

class LynxServo;



class LssChannelBase {
    // They say classes shouldnt be friends, but then why bother have the friend feature? Besides, I
    // will take performance and simplicity over they-say for embedded systems.
    friend class LynxServo;

public: // todo: should be private
    const char* name;

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
    LssChannelBase(const char* channel_name=nullptr);
    virtual ~LssChannelBase();

    virtual void free();

    virtual void update() =0;
    virtual void send(const LynxPacket& p);
    virtual short scan(short beginId, short endId);

    LssChannelBase& add(LynxServo& servo);

    bool contains(short servoId) const;
    const LynxServo& operator[](short servoId) const;
    LynxServo& operator[](short servoId);

    void create(const short* ids, short N);

    template<size_t N>
    inline void create(const short (&ids)[N]) { create(ids, N); }

    // perform a read
    AsyncToken ReadAsyncAll(LssCommands commands);

    // wait for an async operation to finish
    bool waitFor(const AsyncToken& token);

protected:
    void alloc(short n);

    // transmit a serialized packet through the channel
    virtual void transmit(const char* pkt_bytes, int count)=0;
};