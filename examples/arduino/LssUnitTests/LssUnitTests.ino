// define to true if you installed the MemoryFree Arduino library 
// and you want to get free mem information.
// #define HAVE_MEMORYFREE

#include "LssTesting.h"


#ifdef HAVE_MEMORYFREE
#include <MemoryFree.h>
#endif

/// Enable to include unit tests.
/// Unit tests do not require physical devices, the unit tests use a test rig and simulated 
/// hardware to respond to LssServo and LssChannel bus transactions.
#define UNIT_TESTS

/// Enable to include physical tests.
/// If enabled, these tests will scan the bus for devices at the configured baud rate and
/// then perform a series of validation and stress testing.
//#define PHYSICAL_SERIAL      Serial
#define PHYSICAL_BAUDRATE    250000

/// Repeat the physical tests every X milliseconds
#define PHYSICAL_RETEST_DELAY   2000


// this variable gets set to true if at least one bus device is detected
bool physical_rig_present = false;

// the number of loops we completed
int loops = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Lynxmotion Servo Tests:");

#if defined(LSS_OSCOPE_TRIGGER_PIN)
  // this pin will be toggled anytime a read error occurs
  // great for diagnosing electrical issues on the serial line using an oscilloscope
  pinMode(LSS_OSCOPE_TRIGGER_PIN, OUTPUT);
  digitalWrite(LSS_OSCOPE_TRIGGER_PIN, LOW);
#endif

#if defined(UNIT_TESTS)
  test_command_parsing();
  test_serialize_commands();
  test_packet_class();    
  lynx_charstream_tests();
  test_rig();
  test_servos();
  test_async();
#endif

#if defined(PHYSICAL_SERIAL)
  // only show physical operational tests if we are also outputing library unit tests
#if defined(UNIT_TESTS)
  test_head("Physical Servo Operational Tests");
#endif

  // setup serial port attached to servos
  PHYSICAL_SERIAL.begin(PHYSICAL_BAUDRATE);
  physical_rig_present = test_phy_servos();
  test_report_finish();

  if(!physical_rig_present) {
    Serial.print("\r\nNo physical servos detected.\r\nI will try again in ");
    Serial.print(PHYSICAL_RETEST_DELAY/1000.0, 1);
    Serial.println(" seconds.");
  }
#endif
}

void printAggregate(const Aggregate<unsigned long, unsigned long long>& a) {
  Serial.print(a.minimum);
  Serial.print('<');
  Serial.print(a.average());
  Serial.print('<');
  Serial.print(a.maximum);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(PHYSICAL_RETEST_DELAY);
  
#if defined(PHYSICAL_SERIAL)
  test_phy_servos();
  //test_report_finish();

  if((loops%10) ==0) {
    // report stats
    const LynxServo::Statistics& stats = LynxServo::globalStatistics();
    
    Serial.print(F("Packets Q"));
    Serial.print(stats.packet.queries);
    //Serial.print(F(":T"));
    //Serial.print(stats.packet.transmits);
    Serial.print(F(":R"));
    Serial.print(stats.packet.received);
    
    Serial.print(F("   Tx C"));
    unsigned long total = stats.transaction.complete+stats.transaction.partials+stats.transaction.timeouts;
    Serial.print(stats.transaction.complete);
    Serial.print("/P");
    Serial.print(stats.transaction.partials);
    Serial.print("/T");
    Serial.print(stats.transaction.timeouts);
    Serial.print('/');
    Serial.print((stats.transaction.partials+stats.transaction.timeouts)*100.0 / total, 1);
    Serial.print('%');
    
    Serial.print(F("   TTFR "));
    printAggregate(stats.transaction.responseTime);
    Serial.print(F("   TT "));
    printAggregate(stats.transaction.completionTime);

#ifdef HAVE_MEMORYFREE
    Serial.print(F("   MemFree "));
    Serial.println(freeMemory());
#endif

    // rescan for physical devices
    test_physical_scan();
  }
#endif

  loops++;
}
