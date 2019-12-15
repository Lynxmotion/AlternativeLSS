#pragma once

#include "LynxmotionLSS-Config.h"
#include "LssCommunication.h"
#include "LssTransaction.h"
#include "LssPromise.h"

#if defined(LSS_STATS)
#include "analytics/aggregate.h"
#endif

#include <list>

class LynxServo;

#if defined(ARDUINO)
#include "LssArduinoChannel.h"
#else
#include "platform/posix/LssPosixChannel.h"
#endif

#if defined(HAS_LIBFTDI)
#include "platform/libftdi/LssFtdiChannel.h"
#endif


class LssChannel {
public: // todo: should be private
    // parameters
    unsigned long timeout_usec;     // time we will wait for a servo to respond (in microseconds)
    short unresponsive_request_limit;       // if a servo fails to respond to this number of packets in a row, we consider it unresponsive
    unsigned long unresponsive_disable_interval;  // amount of milliseconds a servo gets disabled when found unresponsive

    // packet queue
    unsigned long txn_current;  // transmission number we are sending now
    unsigned long txn_next;     // next transmission number we will assign

    pthread_mutex_t txlock;
    std::list<LssTransaction> transactions;

public:
class Statistics {
    public:
        struct _tx {
            Aggregate<unsigned long> timeToComplete;
        } tx;
    };

public:
    LssChannel();
    virtual ~LssChannel();

#if defined(ARDUINO)
    // open port using Arduino stream interface
    ChannelDriverError begin(Stream& dev, int baudrate);
#else
    // open port using standard linux /dev name or ftdi:<vendor>:<product>:<A,B,C,D>
    ChannelDriverError begin(const char* devname, int baudrate);
#endif

    inline void close() {
        if(_driver) {
            delete _driver;
            _driver = nullptr;
        }
    }

#if defined(ARDUINO)
    // arduino typically uses polling since we are single threaded
    inline void update() { if(_driver) _driver->signal(UpdateSignal, 0, NULL); }
#endif

    virtual short scan(short beginId, short endId);

    LssTransaction::Promise send(std::initializer_list<LynxPacket> p);

    template<class It>
    LssTransaction::Promise send(It first, It last)
    {
        pthread_mutex_lock(&txlock);
        //LssTransaction tx(txn_next++, packets);
        transactions.emplace_back(txn_next++, first, last);
        auto& promise = transactions.back().promise;
        bool sendSignal = transactions.size() ==1;
        pthread_mutex_unlock(&txlock);
        if(_driver && sendSignal)
            _driver->signal(TransactionSignal, 0, nullptr);
        return promise;
    }

    // transmit a serialized packet through the channel
    //virtual void transmit(const char* pkt_bytes, int count)=0;
    virtual void transmit(const LynxPacket& pkt);

    void create(const short* ids, short N);

    template<size_t N>
    inline void create(const short (&ids)[N]) { create(ids, N); }

    // perform a read
    //AsyncToken ReadAsyncAll(LssCommands commands);

    const LssChannelDriver& driver() const { return *_driver; }
    LssChannelDriver& driver() { return *_driver; }

    Statistics statistics;

protected:
    LssChannelDriver* _driver;

    void alloc(short n);

    void completeTransaction();

    // driver callbacks
    // maybe move these functions both driver and callback to a single function with msg and args, makes for better extendability
public: // todo: should not be public
    void driverIdle();
    void driverDispatch(LynxPacket& p);
};