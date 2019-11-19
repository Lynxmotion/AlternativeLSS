
#if defined(ARDUINO)
#include "platform/arduino/LssArduinoChannel.h"
using LynxChannel = LssArduinoChannel;
#else
#include "platform/posix/LssPosixChannel.h"
using LynxChannel = LssPosixChannel;
#endif