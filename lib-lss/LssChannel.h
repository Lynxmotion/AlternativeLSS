
#if defined(ARDUINO)
#include "platform/arduino/LssArduinoChannel.h"
using LssChannel = LssArduinoChannel;
#elif defined(HAS_LIBFTDI)
#include "platform/libftdi/LssFtdiChannel.h"
using LssChannel = LssFtdiChannel;
#else
#include "platform/posix/LssPosixChannel.h"
using LssChannel = LssFtdiChannel;
#endif