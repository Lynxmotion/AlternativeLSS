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
    //Qwait *= 1.025;
    //printf("Q now %ld\n", Qwait);
}

long early_xmissions = 0;
long nots = 0;

unsigned long LssTransaction::Qwait = 1800;

const LynxPacket LssTransaction::next()
{
    unsigned long long now = micros();
    LynxPacket p;

    if(state < Completed) {
        if (txt == 0) {
            // first transmission
            txt = now;
            expireAt = txt + expireInterval;
        }

#if 1
        // we can transmit a packet as long as any packet we are waiting to receive has the same bus ID
        if (_tx != _packets.end() && _tx->id == _rx->id) {
            LynxPacket p = *_tx;
            _tx++;

            // advance over any non-query packets since we wont receive anything back from them
            while (_rx != _packets.end() && _rx < _tx && (_rx->command & LssQuery) == 0)
                _rx++;

            checkCompleteStatus();

            // store timestamp of transmission and schedule next transmission
            p.microstamp = now;
            nextQ = (p.command & LssQuery)
                ? now + 1600         // query packet: wait x microseconds before sending next command
                : now;              // action/config packet: next packet can be sent immediately
            return p;
        }
#else
        if(_tx != _packets.end()) {
            bool is_query = (_tx->command & LssQuery) > 0;

            //  always transmit an action
            if (!is_query) {
                p = *_tx;
                _tx++;
            } else if(now > nextQ || _rx == _tx) {
                // transmit a query if possible, we may send before collecting a response from last one
                // as long as we give enough time for servo to finish transmitting on the bus.
                if(_rx < _tx) {
                    if(ttfr ==0)
                        return p;   // abort, we must at least have received 1 response before blasting queries
                    early_xmissions++;
                    if((early_xmissions % 100)==0)
                        printf("sent %ld earlies\n", early_xmissions);
                }
                p = *_tx;
                _tx++;
            } else {
                nots++;
            }

            if(p.id) {
                // store timestamp of transmission and schedule next transmission
                p.microstamp = now;

                if(_tx == _packets.end()) {
                    // no more packets to send
                    tt_tx_c = now - txt;
                    nextQ = now;
                } else {
                    bool next_is_query = (_tx->command & LssQuery) > 0;
                    bool next_is_same_dev = _tx->id == p.id;

                    // we must add a Qwait if next packet is a query and not the same servo
                    nextQ = (is_query && next_is_query && !next_is_same_dev)
                            ? now + Qwait         // query packet: wait x microseconds before sending next command
                            : now;              // action/config packet: next packet can be sent immediately
                }
            }

        }

        // advance over any non-query packets since we wont receive anything back from them
        while (_rx != _packets.end() && _rx < _tx && (_rx->command & LssQuery) == 0)
            _rx++;

        checkCompleteStatus();

#endif
    }
    return p;
}

void LssTransaction::dispatch(const LynxPacket& p)
{
    if(_rx!=_packets.end() && _rx->id == p.id && _rx->command==p.command) {
        auto now = micros();
        _rx->set(p.value);
        _rx->microstamp = now;
        _rx++;

        expireAt = now + expireInterval;

        if(ttfr ==0)
            ttfr = now;

        // advance over any non-query packets
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
