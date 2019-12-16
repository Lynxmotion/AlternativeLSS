#include "LssArduinoChannel.h"
#include "LssChannel.h"
#include "Arduino.h"

LssArduinoChannel::LssArduinoChannel(LssChannel* channel)
        : LssChannelDriver(channel)
{}

void LssArduinoChannel::begin(Stream* _stream)
{
    serial = _stream;
    pbuffer = buffer;
}

void LssArduinoChannel::update()
{
    if(serial==NULL || pbuffer==NULL) return;

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
                if(packet.id)
                channel->driverDispatch(packet);
            }
            pbuffer = buffer; // reset buffer insert position
        } else {
            // add to buffer
            *pbuffer++ = c;
        }
    }
}

LssArduinoChannel::~LssArduinoChannel()
{
}

intptr_t LssArduinoChannel::signal(ChannelDriverSignal signal, unsigned long a, const void* ptr) {
        // we send a simple character to awaken the processing thread
    switch(signal) {
        case OpenSignal:
            begin( (Stream*)ptr);
            Serial.println("Arduino serial channel");
            break;

        case UpdateSignal:
            channel->driverIdle();
            update();
            break;
            
        case TransactionSignal:
        case DataSignal:
            //write(priv->notify.client, "*", 1);
            break;

        case TransmitSignal:
            if(ptr && a>0) {
                //Serial.print('['); Serial.print(a); Serial.println(']');
                serial->write( (const char*)ptr, a );

#if defined(ARDUINO) && INTERPACKET_DELAY>0
                delay(INTERPACKET_DELAY);
#endif
            }
            break;
    }
    return 0;
} 
