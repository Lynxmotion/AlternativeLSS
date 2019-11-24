//
// Created by guru on 11/22/19.
//

#ifndef LYNXMOTIONLSS_LSSTRANSACTION_H
#define LYNXMOTIONLSS_LSSTRANSACTION_H


#include "MaskSet.h"
#include <vector>

class LssTransaction {
public:
    using Promise = LssPromise<const LssTransaction>;

    typedef enum {
        Pending,
        Active,
        Completed,
        Expired
    } State;

    unsigned long txn;          // current transmission number for this request
    unsigned long timestamp;    // millis when first packet was sent
    unsigned long expireAt;     // timestamp when the read request will be considered expired
    State state;

    // statistics
    unsigned long txt, ttfr, ttc;

    // this promise will be called when the transaction completes or expires
    LssPromise<const LssTransaction> promise;

    // construct a new transaction with the given packets to  transmit
    LssTransaction(unsigned long _txn, std::initializer_list<LynxPacket> packets, unsigned long _expire_uSec=20000);

    // we cannot copy LssTransactions
    LssTransaction(const LssTransaction& copy) = delete;
    LssTransaction& operator=(const LssTransaction& copy) = delete;

    void expire();

    // returns true if the transaction should be expired
    bool expired(unsigned long long tsnow=0) const;

    inline const std::vector<LynxPacket>& packets() const { return _packets; }

    const LynxPacket next();
    void dispatch(const LynxPacket& p);

protected:
    unsigned long expireInterval;

    // list of packets in this transaction
    std::vector<LynxPacket> _packets;

    // pointer to the next packet to be transmitted
    std::vector<LynxPacket>::iterator _tx;

    // pointer to the next query response we expect to receive
    std::vector<LynxPacket>::iterator _rx;

    void checkCompleteStatus();
};


#endif //LYNXMOTIONLSS_LSSTRANSACTION_H
