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

    posix_serial_private* priv;

    unsigned long bytes_sent, bytes_received;

public:
    explicit LssPosixChannel(const char* channel_name=nullptr);
    ~LssPosixChannel() override;

    void free() override;

    bool begin(const char* devname, int baudrate);

    void update() override;

    using LssChannelBase::transmit;
    void transmit(const char* pkt_bytes, int count=-1) override;
    void driverSignal() override;

private:
    void* run();
    static void* s_run(void*);
};