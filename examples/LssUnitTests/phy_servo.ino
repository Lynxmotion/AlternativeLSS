#if defined(PHYSICAL_SERIAL)


short physical_servos[32] = {0};
short physical_servos_count = 0;

bool test_rig_servo_read(short id, LssCommands cmd)
{
  LssPhysicalRig rig(PHYSICAL_SERIAL, physical_servos, physical_servos_count);
  String test = "test_phy_rig_servo";
  char s[32];

  LynxPacket::commandCode(cmd, s);
  test += id;
  test += "_";
  test += s;
  
  AsyncToken w = rig.channel[id].ReadAsync(cmd);    // send out position queries
  bool passed = rig.channel.waitFor(w);
  if(passed) {
    test += "  (";
    switch(cmd) {
      case LssQuery: test += rig.channel[id].state; break;
      case LssPosition: test += rig.channel[id].position; break;
      case LssTarget: test += rig.channel[id].target; break;
      case LssSpeed: test += rig.channel[id].speed; break;
      case LssCurrent: test += rig.channel[id].current; break;
      case LssVoltage: test += rig.channel[id].voltage; break;
      case LssTemperature: test += rig.channel[id].temperature; break;
    }
    test += ')';
  }
  test_report(test, passed);
  rig.channel.free();
  return passed;
}

void test_async_phy_rig_COMMAND_times_N(const char* testprefix, LssCommands cmd, int N)
{
  bool passed = true;
  String test(testprefix);
  test += "_N";
  test += N;
  LssPhysicalRig rig(PHYSICAL_SERIAL, physical_servos, physical_servos_count);
  for(int i=0; i<N && passed; i++) {
    delay(30);
    AsyncToken w = rig.channel.ReadAsyncAll(cmd);
    passed = rig.channel.waitFor(w);
    if(!passed) {
      test += "  (PASS";
      test += i;
      test += ')';
    }
  }

  if(rig.channel.txn_current!=rig.channel.txn_next) {
    test += "  (txn)";
    passed = false;
  }
  test_report(test, passed);
}

bool test_rig_servo_present(short id)
{
  return test_rig_servo_read(id, LssQuery);
}

void test_async_phy_rig_allservos_query()
{
  test_async_phy_rig_COMMAND_times_N("test_async_phy_rig_allservos_query", LssQuery, 1);
}

void test_async_phy_rig_allservos_query_N10()
{
  test_async_phy_rig_COMMAND_times_N("test_async_phy_rig_allservos_query", LssQuery, 10);
}

void test_async_phy_rig_allservos_position_target()
{
  test_async_phy_rig_COMMAND_times_N("test_async_phy_rig_allservos_read_position_target", LssPosition|LssTarget, 1);
}

void test_async_phy_rig_allservos_position_target_N10()
{
  test_async_phy_rig_COMMAND_times_N("test_async_phy_rig_allservos_read_position_target", LssPosition|LssTarget, 10);
}

bool test_physical_scan()
{
  String test = "test_physical_scan";
  LynxChannel channel("busscan");
  channel.begin(PHYSICAL_SERIAL);

  int discovered = channel.scan(1, (sizeof(physical_servos)/sizeof(physical_servos[0]) ));    // send out position queries, up to as many as our array can hold
  bool passed = discovered > 0;
  if(passed) {
    test += "  discovered ";
    test += channel.count;
    test += " device(s)   (IDs ";
    physical_servos_count = channel.count;
    //memset(physical_servos, 0, sizeof(physical_servos));
    for(int i=0; i<channel.count; i++) {
      physical_servos[i] = channel.servos[i]->id;
      if(i>0)
        test += ',';
      test += physical_servos[i];
    }
    test += ')';
  }
  if(physical_servos_count<=0) {
    passed = false;
    test += "  (no devices)";
  }
  test_report(test, passed);
  //if (physical_servos_count) physical_servos_count = 1;
  return passed;
}

bool test_phy_servos()
{
  #if false
  // just do some basic position/target test on physical servos
  test_async_phy_rig_allservos_position_target_N10();
  return true;
  #else
  // check to see if all our needed physical servos are present
  bool ready = (physical_servos_count>0) 
    ? true
    : test_physical_scan();

  if(ready) {
#if 0
    for(int i=0; i<physical_servos_count; i++) {
        short id = physical_servos[i];
        test_rig_servo_present(id);
        test_rig_servo_read(id, LssPosition);
        test_rig_servo_read(id, LssTarget);
    }
    test_async_phy_rig_allservos_query();
    test_async_phy_rig_allservos_position_target();
    
    test_head(F("Physical Servo Stress Tests"));
    test_async_phy_rig_allservos_query_N10();
#endif
    test_async_phy_rig_allservos_position_target_N10();
  } else {
    Serial.println("   SKIPPED  One or more physical servos are missing");
  }
  return ready;
  #endif
}

#endif
