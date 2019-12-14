#pragma once

#include "LssChannelDriver.h"

class Stream;

class LssArduinoChannel : public LssChannelDriver {
public: // todo: should be private
    Stream* serial;
    char buffer[32];
    char* pbuffer;

public:
    LssArduinoChannel(LssChannel* channel);
    ~LssArduinoChannel() override;

    intptr_t signal(ChannelDriverSignal signal, unsigned long a, const void* ptr) override;

private:
    void begin(Stream* serial);
    void update();
};

