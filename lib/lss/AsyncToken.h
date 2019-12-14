
#pragma once

#include "LynxmotionLSS-Config.h"
#include "LssCommunication.h"
#include "MaskSet.h"

#include <functional>

#if 0
class AsyncToken
{
public:
    AsyncToken() : set(NULL) {}
    AsyncToken(const AsyncToken& copy) : set(copy.set) {}
    AsyncToken(AsyncToken&& mv) : set(mv.set) { mv.set = nullptr; }
    AsyncToken(const MaskSet& _set) : set(&_set) {}

    inline AsyncToken& operator=(const AsyncToken& t) { set = t.set; return *this; }
    inline AsyncToken& operator=(const MaskSet& s) { set = &s;  return *this; }

    // clear the async token
    inline void clear() { set = NULL;  }

    inline bool isValid() const { return set!=NULL; }
    inline bool isActive() const { return set && set->isActive(); }
    inline bool isComplete() const { return set && set->isComplete(); }
    inline bool isUnresponsive() const { return set && set->isUnresponsive(); }

    // returns an async token that indicates an unresponsive device/servo
    static AsyncToken Unresponsive() {
        static MaskSet um(0, LssQuery);
        return AsyncToken(um);
    }

protected:
    const MaskSet* set;
};

#endif