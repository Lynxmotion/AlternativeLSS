
#if defined(ARDUINO)
#include "platform/arduino/LssArduinoChannel.h"
using LynxChannel = LssArduinoChannel;
#elif defined(HAS_LIBFTDI)
#include "platform/libftdi/LssFtdiChannel.h"
using LynxChannel = LssFtdiChannel;
#else
#include "platform/posix/LssPosixChannel.h"
using LynxChannel = LssFtdiChannel;
#endif