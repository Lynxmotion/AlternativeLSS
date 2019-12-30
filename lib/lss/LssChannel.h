#pragma once

#include "LynxmotionLSS-Config.h"
#include "LssCommunication.h"
#include "LssTransaction.h"
#include "LssPromise.h"

#if defined(LSS_STATS)
#include "analytics/aggregate.h"
#endif

#include <list>
#include <utility>

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

    using Promise = LssPromise< LssTransaction >;

    class QueuedTransaction {
    public:
        std::shared_ptr<LssTransaction> tx;
        Promise promise;

        inline QueuedTransaction() {}
        inline QueuedTransaction(std::shared_ptr<LssTransaction> _tx) : tx(std::move(_tx)) {}
    };

    pthread_mutex_t txlock;
    std::list< QueuedTransaction > transactions;

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

    inline bool ready() const {
        return _driver != nullptr;
    }

    inline void close() {
        if(_driver) {
            delete _driver;
            _driver = nullptr;
        }
    }

    void clear();

    short scan(short beginId, short endId);

    Promise send(std::shared_ptr<LssTransaction> tx);

    inline Promise send(std::initializer_list<LynxPacket> packets)
    {
        return send(std::make_shared<LssTransaction>(txn_next++, packets));
    }

    template<class It>
    Promise send(It first, It last)
    {
        return send(std::make_shared<LssTransaction>(txn_next++, first, last));
    }

    // transmit a serialized packet through the channel
    void transmit(const LynxPacket& pkt);

    /// transmit data directly onto the LSS bus
    /// Use at your own risk, this bypasses performance features of this library and is not recommended.
    void transmit(const char* text, int text_len=-1);

    const LssChannelDriver& driver() const { return *_driver; }
    LssChannelDriver& driver() { return *_driver; }

#if defined(ARDUINO)
    // arduino typically uses polling since we are single threaded
    inline void update() { if(_driver) _driver->signal(UpdateSignal, 0, NULL); }
#endif

    Statistics statistics;

protected:
    LssChannelDriver* _driver;

    void completeTransaction();

    // driver callbacks
    // maybe move these functions both driver and callback to a single function with msg and args, makes for better extendability
public: // todo: should not be public
    void driverIdle();
    void driverDispatch(LynxPacket& p);
};