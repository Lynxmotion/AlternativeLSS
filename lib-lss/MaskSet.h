
#pragma once

#include "LynxmotionLSS-Config.h"
#include "LssCommunication.h"
#include "LssPromise.h"

class MaskSet
    // TODO: rename to AsyncTransaction
{
public:
    short busid;
    LssCommands read;
    LssCommands requested;
    LssCommands completed;

    inline MaskSet(short _busid = 0, LssCommands _read = LssInvalid) : busid(_busid), read(_read), requested(0), completed(0) {}

    inline bool isComplete() const { return read == completed; }
    //inline bool isUnresponsive() const { return completed==0; }
};
