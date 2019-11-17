
#pragma once

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif
#include <WString.h>

#include <LynxmotionLSS.h>

class CharStream : public Stream
{
  protected:
    const short rewriteLimit = 8;
    
    // we will allocate buffer as two pages
    char buffer[128];
    char* pread;
    char* pwrite;
    
  public:
    void clear();

    CharStream();

    // override Stream methods
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();

    // override Print methods
    virtual size_t write(uint8_t c);
};

/*
 * Simple Simulated Servo for testing
 * 
 */
class SimServo
{
  public:
    Stream* channel;
    short id;
    short position;
    short target;
    short speed;
    short voltage;
    short current;
    short temperature;

    SimServo(short _id=0);

    void assign(short _id, Stream* _channel);

    LynxPacket receive(LynxPacket p);
};

// we must declare our placement-new operator so we can instantiate SimServo in-place
void * operator new (size_t size, void * ptr);


class SimChannel : public CharStream
{
  public:
    char writebuffer[128];
    char* wbwrite;
    SimServo* servos;
    short count;
    
  public:
    SimChannel();
    ~SimChannel();

    bool contains(short servoId) const;
    SimServo& operator[](short servoId);
    const SimServo& operator[](short servoId) const;

    virtual size_t write(uint8_t c);

    template<size_t N>
    void begin(const int (&ids)[N])
    {
      count=N;
      servos = (SimServo*)calloc(count, sizeof(SimServo));
      for(int i=0; i<count; i++) {
        // call the placement new operator here supplying the memory the constructor operates on
        // "placement new" operator doesnt allocate memory, we supply the memory it just runs the constructor
        SimServo* s = new (&servos[i]) SimServo();
        s->assign(ids[i], this);
      }
    }
};

class LssTestRig {
  public:
    SimChannel sim;
    LynxChannel channel;
    LynxServo hip;
    LynxServo ankle;
    LynxServo foot;

    LssTestRig();
    bool scan();
    void update();
};

class LssPhysicalRig {
  public:
    LynxChannel channel;

    LssPhysicalRig(HardwareSerial& serial, short* ids, short count);
    
    void update();
};


extern bool _neg_test;
#define test_neg(test_func) { _neg_test=true; test_func;  _neg_test=false; }

void test_report(const __FlashStringHelper* result, bool passed);
void test_report(String result, bool passed);
void test_head(const __FlashStringHelper* header);
void test_report_finish();

void test_parse_command(const char* cmd, unsigned long parsed);
void test_parse_packet(const char* pkt, LynxPacket packet);
void test_serialize_command(const char* pkt, LssCommands cmd);
void test_serialize_packet(const char* pkt, LynxPacket packet); 

//void test_packet_class();
void lynx_charstream_tests();
void lynx_servo_tests();
