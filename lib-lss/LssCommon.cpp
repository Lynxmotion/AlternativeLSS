#include "LssCommon.h"



#if defined(ARDUINO)

#define ConsolePrint(s) Serial.print(s);
#define ConsolePrintln() Serial.println();

#else
#include <sys/time.h>

#define ConsolePrint(s) printf(s);
#define ConsolePrintln() printf("\n");

// our epoch will be from program start like in Arduino
bool timeInitialized = false;
time_t programStart = {0};

void initTime() {
    if(!timeInitialized) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        programStart = tv.tv_sec;
        timeInitialized = true;
    }
}

unsigned long long micros() {
    struct timeval tv;

    initTime();
    gettimeofday(&tv, NULL);
    tv.tv_sec -= programStart;
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

unsigned long long millis() {
    struct timeval tv;

    initTime();
    gettimeofday(&tv, NULL);
    tv.tv_sec -= programStart;
    return tv.tv_sec * 1000 + tv.tv_usec/1000;
}


/*Stream::Stream(const char* port)
{

}

void Stream::print(const char* s)
{

}

void Stream::println()
{

}

void Stream::println(const char* s)
{

}*/

#endif




#if defined(LSS_OSCOPE_TRIGGER_PIN)
void oscope_trigger(short exception_code, const char* msg)
{
  // flip pin state to trigger oscope
  digitalWrite(LSS_OSCOPE_TRIGGER_PIN, HIGH);
  delayMicroseconds(500);
  digitalWrite(LSS_OSCOPE_TRIGGER_PIN, LOW);
  delayMicroseconds(250);
  ConsolePrint("TRG@");
  ConsolePrint(exception_code);
  ConsolePrint(" => ");
  ConsolePrint(msg);
  ConsolePrintln();
}
#endif

