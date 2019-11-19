
#if (ARDUINO >= 100)
#include <Arduino.h>
#elif defined(ARDUINO)
#include <WProgram.h>
 #include <WString.h>
 #include <pins_arduino.h>
#else
// Linux targets?
#include <string>
#include <cstring>
#include <stdlib.h>

typedef std::string String;
typedef unsigned char byte;

unsigned long long micros();
unsigned long long millis();

/*class Stream {
public:
    Stream(const char* port);



    void print(const char* s);

    void println();
    void println(const char* s);
};*/

#endif


#if defined(LSS_OSCOPE_TRIGGER_PIN)
#define OSCOPE_TRIGGER(ex, msg) oscope_trigger(ex);
void oscope_trigger(short exception_code, const char* msg);
#else
#define OSCOPE_TRIGGER(ex, msg)
void oscope_trigger(short exception_code, const char* msg);// nop
#endif
