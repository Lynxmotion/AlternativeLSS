#pragma once

#include "../../LssChannelDriver.h"

class ftdi_serial_private;


class LssFtdiChannel : public LssChannelDriver {
private:
    int baudrate;
    ftdi_serial_private* priv;

public:
    explicit LssFtdiChannel(LssChannel* channel);
    ~LssFtdiChannel() override;

    intptr_t signal(ChannelDriverSignal signal, unsigned long a, const void* ptr) override;

private:
    void* run();
    static void* s_run(void*);

    ChannelDriverError begin(const char* devname, int baudrate);
    void transmit(const char* pkt_bytes, int count=-1);
};