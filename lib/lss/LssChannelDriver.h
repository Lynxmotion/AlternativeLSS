//
// Created by guru on 12/13/19.
//

#pragma once

#include <cstdint>

typedef enum ChannelState {
    ChannelStopped,
    ChannelError,
    ChannelStarting,
    ChannelStopping,
    ChannelIdle,
    ChannelProcessing
} ChannelState;

typedef enum {
    DriverSuccess,
    DriverUnknownError,
    DriverOpenFailed,
    DriverAlreadyInitialized,
    DriverNotFound,
    DriverAccessDenied,
    DriverInvalidParameter
} ChannelDriverError;


typedef enum {
    OpenSignal,
    UpdateSignal,
    DataSignal,
    TransmitSignal,
    TransactionSignal
} ChannelDriverSignal;

class LssChannel;

class LssChannelDriver
{
protected:
    LssChannel* channel;

public:
    class Statistics {
    public:
        unsigned long bytes_sent, bytes_received;

        inline Statistics() : bytes_sent(0), bytes_received(0) {}
    };

public:
    virtual ~LssChannelDriver() {};

    explicit inline LssChannelDriver(LssChannel* _channel) : channel(_channel) {}

    ///@brief signal the driver of a new event
    virtual intptr_t signal(ChannelDriverSignal signal, unsigned long a, const void* ptr)=0;

    Statistics statistics;
};