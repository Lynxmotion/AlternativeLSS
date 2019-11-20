
#pragma once

#include "LynxmotionLSS-Config.h"
#include "LssCommunication.h"
#include "LssPromise.h"

class MaskSet
    // TODO: rename to AsyncTransaction
{
public:
    using Promise = LssPromise<const MaskSet>;

    unsigned long txn;  // current transmission number for this request
    LssCommands read;
    LssCommands requested;
    LssCommands completed;

    unsigned long timestamp;      // millis when first packet was sent
    unsigned long expire;         // timestamp when the read request will be considered expired

    inline MaskSet(unsigned long _txn = 0, LssCommands _read = LssInvalid) : txn(_txn), read(_read), requested(0), completed(0), timestamp(0), expire(0) {}

    inline bool isActive() const { return txn!=0; }
    inline bool isComplete() const { return read == completed; }
    inline bool isUnresponsive() const { return completed==0; }
};
