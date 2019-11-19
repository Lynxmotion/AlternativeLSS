#pragma once

#include "../LssChannel.h"

class LssArduinoChannel : public LssChannelBase {
public: // todo: should be private
    const char* name;
    Stream* serial;
    char buffer[32];
    char* pbuffer;

public:
    LssArduinoChannel(const char* channel_name=NULL);
    virtual ~LssArduinoChannel();

    void begin(Stream& serial);
    void update();

    void send(const LynxPacket& p);
};