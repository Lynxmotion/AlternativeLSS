#pragma once

#include "../LssChannel.h"

class posix_serial_private;

enum ChannelState {
    ChannelStopped,
    ChannelError,
    ChannelStarting,
    ChannelStopping,
    ChannelIdle,
    ChannelProcessing
};

class LssPosixChannel : public LssChannelBase {
public: // todo: should be private
    const char* devname;
    int baudrate;
    char buffer[256];
    char* pbuffer;

    posix_serial_private* priv;

    unsigned long bytes_sent, bytes_received;

public:
    LssPosixChannel(const char* channel_name=nullptr);
    virtual ~LssPosixChannel();

    virtual void free();

    virtual void begin(const char* devname, int baudrate);
    virtual void update();

    virtual void transmit(const char* pkt_bytes, int count);

private:
    void* run();
    static void* s_run(void*);

    void open();
};