#pragma once

#include "../LssChannel.h"

class posix_serial_private;

class LssPosixChannel : public LssChannelBase {
public: // todo: should be private
    char buffer[256];
    char* pbuffer;

    posix_serial_private* priv;

public:
    LssPosixChannel(const char* channel_name=nullptr);
    virtual ~LssPosixChannel();

    virtual void free();

    virtual void begin(const char* devname, int baudrate);
    virtual void update();

    virtual void transmit(const char* pkt_bytes, int count);

private:
    // todo: must convert signal_handler_IO to instance method and trampoline
    static void signal_handler_IO(int status);
};