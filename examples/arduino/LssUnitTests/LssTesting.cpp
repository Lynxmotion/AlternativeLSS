
#include "LssTesting.h"

void * operator new (size_t size, void * ptr) { return ptr; }

CharStream::CharStream()
    : pread(buffer), pwrite(buffer) 
{
}

void CharStream::clear() { 
  pread=buffer; 
  pwrite=buffer; 
}

int CharStream::available() { return pwrite - pread; }

int CharStream::read() { 
  if(pread >= pwrite) 
    return -1; 
  else {
    if(pread - buffer > rewriteLimit)
      flush();
    return *pread++;
  }
}

int CharStream::peek() { 
  if(pread >= pwrite) 
    return -1; 
  else {
    return *pread;
  }
}

void CharStream::flush() {
  short n = pread - buffer;
  if(n>0) {
    memmove(buffer, pread, pwrite-pread);
    pread -= n;
    pwrite -= n;
  }
}

size_t CharStream::write(uint8_t c) {
  *pwrite++ = c;
  return 1;
}




/*
 * Simple Simulated Servo for testing
 * 
 */

SimServo::SimServo(short _id) 
    : id(_id), position(1800), target(1800), speed(105), voltage(1240), current(100), temperature(230) 
{
}

void SimServo::assign(short _id, Stream* _channel) 
{
  id=_id; 
  channel=_channel; 
}

LynxPacket SimServo::receive(LynxPacket p) 
{
  bool query = (p.command & LssQuery)>0;
  LynxPacket response(id, p.command);
  if((p.command & LssPosition)>0) {
    if(query) {
      response.set(position);
      return response;
    } else if(p.hasValue)
      position = p.value;
  }
  if((p.command & LssTarget)>0) {
    if(query) {
      response.set(target);
      return response;
    } else if(p.hasValue)
      target = p.value;
  }
  
  return LynxPacket();
}


SimChannel::SimChannel() 
    : wbwrite(writebuffer), servos(NULL), count(0)
{
}

SimChannel::~SimChannel()
{
  if(servos)
    free(servos);
}

bool SimChannel::contains(short servoId) const 
{
  for(int i=0; i<count; i++)
    if(servos[i].id == servoId) return true;
  return false;
}

SimServo& SimChannel::operator[](short servoId) 
{
  for(int i=0; i<count; i++)
    if(servos[i].id == servoId) return servos[i];
  Serial.println("SimServo::operator[] called on non-existent servo");
  while(1);
}

const SimServo& SimChannel::operator[](short servoId) const 
{
  for(int i=0; i<count; i++)
    if(servos[i].id == servoId) return servos[i];
  Serial.println("SimServo::operator[] called on non-existent servo");
  while(1);
}

size_t SimChannel::write(uint8_t c) 
{
  if(c==13) {
    *wbwrite=0;              // null terminate the write buffer
    wbwrite = writebuffer;   // go back to beginning
    if(*wbwrite!='#')
      return 1;
    wbwrite++; // skip #

	// parse the data into a packet
	LynxPacket p;
    if(!p.parse(wbwrite))
      return 0;

	// find the servo associated with this packet ID
    for(int i=0; i<count; i++) {
      if(servos[i].id == p.id) {
        LynxPacket response = servos[i].receive(p);
        if(response.id!=0 && response.command!=LssInvalid) {
          char buf[64];
          char* pend = buf;
          *pend++ = '*';
          if((pend=response.serialize(pend)) !=NULL) {
            char* pbegin = buf;
            *pend++ = '\r';
            while(pbegin < pend)
              CharStream::write(*pbegin++);  // must use base::write() or it will kick back to this virtual method
          }
        }
        break;
      }
    }
    wbwrite=writebuffer;  // clear write buffer
  } else if(c!=10) {
    //return CharStream::write(c);
    *wbwrite++ = c;
  }
  return 1;
}



LssTestRig::LssTestRig() : channel("rig"), hip(1), ankle(5), foot(6) {
  // instantiate our client side Channel/Servos
  channel
    .add(hip)
    .add(ankle)
    .add(foot);
  channel.begin(sim);
  sim.begin((int[]){hip.id, ankle.id, foot.id});
}

bool LssTestRig::scan()
{
  AsyncToken t = channel.ReadAsyncAll(LssPosition);
  return channel.waitFor(t);
}

void LssTestRig::update() {
  // allow the channel to read incoming data
  channel.update();
}

LssPhysicalRig::LssPhysicalRig(HardwareSerial& serial, short* ids, short count)
{
  if(ids && count>0)
    channel.create(ids, count);
  channel.begin(serial);
}

void LssPhysicalRig::update()
{
  // allow the channel to read incoming data
  channel.update();
}



/* UNIT TESTS
 *  
 */
long _passed=0, _failed=0;
bool _neg_test = false;

void test_report(const __FlashStringHelper* test, bool passed) {
  if(_neg_test) passed = !passed;
  Serial.print(F("   "));
  Serial.print(passed ? " PASS ":"*FAIL*" );
  Serial.print(F("   "));
  Serial.println(test);
  if(passed) _passed++; else _failed++;
}

void test_report(String test, bool passed) {
  if(_neg_test) passed = !passed;
  Serial.print(F("   "));
  Serial.print(passed ? " PASS ":"*FAIL*" );
  Serial.print(F("   "));
  Serial.println(test);
  if(passed) _passed++; else _failed++;
}

void test_head(const char* header) {
  Serial.println();
  Serial.print(header);
  Serial.println(F(":"));
}

void test_head(const __FlashStringHelper* header) {
  Serial.println();
  Serial.print(header);
  Serial.println(F(":"));
}

void test_report_finish() {
  Serial.print(_passed); Serial.print("\r tests passed");
  if(_failed>0) { Serial.print(", "); Serial.print(_failed); Serial.println(" failed."); } else Serial.println(".");
}

#define test_neg(test_func) { _neg_test=true; test_func;  _neg_test=false; }
