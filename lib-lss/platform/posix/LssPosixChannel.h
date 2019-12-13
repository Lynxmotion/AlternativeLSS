#pragma once

#include "../LssChannelDriver.h"

class posix_serial_private;


class LssPosixChannel : public LssChannelDriver {
public: // todo: should be private
    const char* devname;
    int baudrate;

    posix_serial_private* priv;

public:
    explicit LssPosixChannel(LssChannel* channel);
    ~LssPosixChannel();

    intptr_t signal(ChannelDriverSignal signal, unsigned long a, const void* ptr) override;

    ChannelDriverError begin(const char* devname, int baudrate);

    void transmit(const char* pkt_bytes, int count=-1);

private:
    void* run();
    static void* s_run(void*);
};