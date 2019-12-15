#pragma once

#include "../../LssChannelDriver.h"

class posix_serial_private;


class LssPosixChannel : public LssChannelDriver {
private:
    const char* devname;
    int baudrate;

    posix_serial_private* priv;

public:
    explicit LssPosixChannel(LssChannel* channel);
    ~LssPosixChannel() override;

    intptr_t signal(ChannelDriverSignal signal, unsigned long a, const void* ptr) override;

private:
    void* run();
    static void* s_run(void*);

    ChannelDriverError begin(const char* devname, int baudrate);
    void transmit(const char* pkt_bytes, int count=-1);
};