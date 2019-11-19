
#include "LssArduinoChannel.h"

LssArduinoChannel::LssArduinoChannel(const char* channel_name)
        : LynxChannel(channel_name)
{}

void LssArduinoChannel::begin(Stream& _stream)
{
    serial = &_stream;
    pbuffer = buffer;
}

void LssArduinoChannel::update()
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
                // todo: remove me....this was to capture a particular occurance of a corrupt packet due to servo issue
                if (buffer[1] == '2' && buffer[2] == 'Q' && buffer[2] == 'D' && buffer[2] == 'L') {
                    char corrupt[40];
                    sprintf(corrupt, "BADPKT/%s", buffer);
                    OSCOPE_TRIGGER(1, corrupt)
                }

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

void LssArduinoChannel::transmit(const char* pkt_bytes)=0
{
    serial->print(pkt_bytes);

#if defined(ARDUINO) && INTERPACKET_DELAY>0
    delay(INTERPACKET_DELAY);
#endif
}

LssArduinoChannel::~LssArduinoChannel()
{
    LynxChannel::free();
}
