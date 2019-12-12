//
// Created by guru on 11/22/19.
//

#include "LssTransaction.h"



LssTransaction::LssTransaction(unsigned long _txn, std::initializer_list<LynxPacket> packets, unsigned long _expire_uSec)
    : txn(_txn), timestamp(micros()), expireAt(0), nextQ(0), expireInterval(_expire_uSec), txt(0), ttfr(0), ttc(0), state(Pending),
      _packets(packets)
{
    std::sort(_packets.begin(), _packets.end(), _packet_order_by_busid::sorter);
    _tx = _packets.begin();
    _rx = _packets.begin();
}

#if 0
LssTransaction::LssTransaction(const LssTransaction& copy)
    : LssPromise(copy), txn(copy.txn), timestamp(copy.timestamp), expireAt(copy.expireAt),
      txt(copy.txt), ttfr(copy.ttfr), ttc(copy.ttc), completed(copy.completed),
      _packets(copy._packets)
{
    unsigned long _tx_i = copy._tx - copy._packets;
    unsigned long _rx_i = copy._rx - copy._packets;
    _tx = _packets + _tx_i;
    _rx = _packets + _rx_i;
}

LssTransaction& LssTransaction::operator=(const LssTransaction& copy)
{
    LssPromise::operator=(copy);
    txn = copy.txn;
    timestamp = copy.timestamp;
    expireAt = copy.expireAt;
    txt = copy.txt;
    ttfr = copy.ttfr;
    ttc = copy.ttc;
    completed = copy.completed;
    _packets = copy._packets;
    return *this;
}
#endif

bool LssTransaction::expired(unsigned long long tsnow) const {
    if(state == Expired)
        return true;    // already marked as expired
    if(expireAt ==0)
        return false;   // no packets transmitted yet, so expire timer not started
    if(tsnow ==0)
        tsnow = micros();
    return tsnow > expireAt;
}

void LssTransaction::expire() {
    state = Expired;
}


const LynxPacket LssTransaction::next()
{
    if(state < Completed) {
        if (txt == 0) {
            // first transmission
            txt = micros();
            expireAt = txt + expireInterval;
        }

        // we can transmit a packet as long as any packet we are waiting to receive has the same bus ID
        if (_tx != _packets.end() && _tx->id == _rx->id) {
            LynxPacket p = *_tx;
            _tx++;

            while (_rx != _packets.end() && _rx < _tx && (_rx->command & LssQuery) == 0)
                _rx++;

            checkCompleteStatus();
            return p;
        }
    }
    return LynxPacket();
}

void LssTransaction::dispatch(const LynxPacket& p)
{
    if(_rx->id == p.id && _rx->command==p.command) {
        _rx->set(p.value);
        _rx++;

        while(_rx!=_packets.end() && (_rx->command & LssQuery)==0)
            _rx++;

        checkCompleteStatus();
    }
}

void LssTransaction::checkCompleteStatus()
{
    if(state >= Completed)
        return;
    if(_rx == _packets.end() && _tx == _packets.end()) {
        state = Completed;
        ttc = micros() - txt;
        //printf("ttc %ld\n", ttc);
    }
}
