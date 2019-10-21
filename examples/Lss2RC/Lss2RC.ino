
#include <LssCommunication.h>
#include <Servo.h>

Servo servos[3];



/* todo
 *  (1) Add analog sensors
 *  (2) Read/write config to eeprom
 *  
 */


Servo* resolve_servo(unsigned short id) {
    switch(id) {
      case  9: return &servos[0]; break;
      case 10: return &servos[1]; break;
      case 11: return &servos[2]; break;
      default: return nullptr;
    }
}

int resolve_analog(unsigned short id) {
    switch(id) {
      case 3: return A3; break;
      case 4: return A4; break;
      case 5: return A5; break;
      default: return 0;
    }
}

void process_packet(LynxPacket p) {
  // todo: if we know its a servo command we can get the servo beforehand, or we can get sensor address, whatever
  // todo: we can also automate the response printing
  int w;
  Servo* servo = resolve_servo(p.id);
  if( servo != nullptr) {
    // position command 
    if(p.command == (LssDegrees|LssPosition) && p.hasValue) {
      // send the command to the servo
      servo->write(p.value);
    } 

    // position query
    else if(p.command == (LssDegrees|LssPosition|LssQuery) && !p.hasValue) {
      p.set(servo->read());
    }

      // print the response
      Serial.print('*');
      Serial.println(p.toString());
  } else if((w = resolve_analog(p.id)) >0) {
    // read/write pins
    if(p.command == (LssDegrees|LssPosition) && p.hasValue) {
      // send the command to the servo
      digitalWrite(w, p.value);
    } 

    // position query
    else if(p.command == (LssDegrees|LssPosition|LssQuery) && !p.hasValue) {
      p.set(digitalRead(w));
    }

    // print the response
    Serial.print('*');
    Serial.println(p.toString());
  }
}

void setup() {
  Serial.begin(115200);
  
  servos[0].attach(9);
  servos[1].attach(10);
  servos[2].attach(11);

  //analogReadResolution(16)

  // enable pullups
  //pinMode(A3, INPUT_PULLUP);
  //pinMode(A4, INPUT_PULLUP);
  //pinMode(A5, INPUT_PULLUP);
}

unsigned long long next_update=0;

char cmdbuffer[32];
char* pcmd = cmdbuffer;

LynxPacket pkt;

void loop() {
  while (Serial.available() > 0) {
    // read the incoming byte
    int c = Serial.read();

    if(c == '\r') {
      *pcmd = 0;  // append null

      // todo: we can probably convert this to a state machine model
      if(cmdbuffer[0] == '#') {
        // parse the LSS command
        if(pkt.parse(&cmdbuffer[1])) {
          process_packet(pkt);
        }
      }

      // clear packet buffer
      pcmd = cmdbuffer;
    } else if(c == '#') {
      // for now, we reset the buffer if we encounter a packet-start character
      pcmd = cmdbuffer;
      *pcmd++ = (char)c;
    } else {
      // add to cmd buffer
      *pcmd++ = (char)c;
    }
  }

  // if we reached the update timer, print data
  if(millis() > next_update) {
    next_update = millis() + 1000;
    Serial.print("A 3:");
    Serial.print(digitalRead(A3));
    Serial.print("  4:");
    Serial.print(digitalRead(A4));
    Serial.print("  5:");
    Serial.println(digitalRead(A5));    
  }
}
