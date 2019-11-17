
void test_async_req_increments_txn()
{
  LssTestRig rig;
  rig.ankle.ReadAsync(LssPosition);
  test_report(F("test_async_req_increments_txn"), rig.channel.txn_next==1);
}

void test_async_only_token_holder_sends()
{
  LssTestRig rig;
  short id = rig.ankle.id;
  rig.channel.txn_next = 2;
  rig.sim[id].position = 670;  // directly setting sim position of servo5
  rig.ankle.ReadAsync(LssPosition);
  rig.update();
  test_report(F("test_async_only_token_holder_sends"), rig.channel[id].position!=670);
}

void test_async_servo_sends_when_holding_token()
{
  LssTestRig rig;
  String test(F("test_async_servo_sends_when_holding_token"));
  bool passed = false;
  short id = rig.ankle.id;
  rig.channel.txn_next = 2;
  rig.sim[id].position = 670;  // directly setting sim position of servo5
  rig.ankle.ReadAsync(LssPosition);
  rig.update();
  if(rig.channel[id].position!=670) {
    rig.channel.txn_current++;
    rig.update();
    if(rig.channel[id].position!=670) {
      rig.channel.txn_current++;
      rig.update();
      if(rig.channel[id].position==670) {
        passed = true;
	  }
	  else test += "  (3rd)";
	}
	else test += "  (2nd)";
  }
  else test += "  (1st)";
  test_report(test, passed);
}

void test_async_txn_increments_when_async_done()
{
  LssTestRig rig;
  bool passed = false;
  short id = rig.ankle.id;
  rig.channel.txn_next = 2;
  rig.sim[id].position = 670;  // directly setting sim position of servo5
  rig.ankle.ReadAsync(LssPosition);
  rig.update();
  if(rig.channel[id].position!=670) {
    rig.channel.txn_current++;
    rig.update();
    if(rig.channel[id].position!=670) {
      rig.channel.txn_current++;
      rig.update();
      if(rig.channel[id].position==670) {
        rig.update();
        passed = rig.channel.txn_current==3;  // received now that we have the token
      }
    }
  }
  test_report(F("test_async_txn_increments_when_async_done"), passed);
}

void test_async_position()
{
  LssTestRig rig;
  short id = rig.ankle.id;
  rig.sim[id].position = 670;  // directly setting sim position of servo5
  rig.ankle.ReadAsync(LssPosition);
  rig.update();
  test_report(F("test_async_position"), rig.ankle.position == 670);
}

void test_async_position_target()
{
  LssTestRig rig;
  short id = rig.ankle.id;
  rig.sim[id].position = 670;  // directly setting sim position of servo5
  rig.sim[id].target = 150;  // directly setting sim position of servo5
  rig.ankle.ReadAsync(LssPosition|LssTarget);
  rig.update();
  test_report(F("test_async_position_target"), rig.ankle.position==670 && rig.ankle.target==150);
}

void test_async_position_target_with_2_servos()
{
  bool passed = false;
  String test(F("test_async_position_target_with_2_servos"));
  LssTestRig rig;
  short hip = rig.hip.id;
  short ankle = rig.ankle.id;
  rig.sim[hip].position = 420;  // directly setting sim position of servo5
  rig.sim[hip].target = 50;  // directly setting sim position of servo5
  rig.sim[ankle].position = 670;  // directly setting sim position of servo5
  rig.sim[ankle].target = 150;  // directly setting sim position of servo5
  rig.hip.ReadAsync(LssPosition|LssTarget);
  rig.ankle.ReadAsync(LssPosition|LssTarget);
  rig.update();
  if(rig.ankle.position!=670 && rig.ankle.target!=150 && rig.hip.position==420 && rig.hip.target==50) {
    // second update should read the next servo
    rig.update();
    if(rig.ankle.position==670 && rig.ankle.target==150 && rig.hip.position==420 && rig.hip.target==50) {
      passed = true;
    } else {
      test += " (2nd pass)";
    }
  } else test += " (1st pass)";
  test_report(test, passed && rig.channel.txn_current==rig.channel.txn_next);
}

void test_async_position_target_with_2_servos_rev_order()
{
  bool passed = false;
  String test(F("test_async_position_target_with_2_servos_rev_order"));
  LssTestRig rig;
  short hip = rig.hip.id;
  short ankle = rig.ankle.id;
  rig.sim[hip].position = 420;  // directly setting sim position of servo5
  rig.sim[hip].target = 50;  // directly setting sim position of servo5
  rig.sim[ankle].position = 670;  // directly setting sim position of servo5
  rig.sim[ankle].target = 150;  // directly setting sim position of servo5
  rig.ankle.ReadAsync(LssPosition|LssTarget);
  rig.hip.ReadAsync(LssPosition|LssTarget);
  rig.update();
  if(rig.ankle.position==670 && rig.ankle.target==150 && rig.hip.position!=420 && rig.hip.target!=50) {
    // second update should read the next servo
    rig.update();
    if(rig.ankle.position==670 && rig.ankle.target==150 && rig.hip.position==420 && rig.hip.target==50) {
      passed = true;
    } else {
      test += " (2nd pass)";
    }
  } else test += " (1st pass)";
  test_report(test, passed && rig.channel.txn_current==rig.channel.txn_next);
}

void test_async_position_target_read_all_3servos()
{
  bool passed = false;
  String test(F("test_async_position_target_read_all_3servos"));
  LssTestRig rig;
  short hip = rig.hip.id;
  short ankle = rig.ankle.id;
  short foot = rig.foot.id;
  rig.sim[hip].position = 420;  // directly setting sim position of servo5
  rig.sim[hip].target = 50;  // directly setting sim position of servo5
  rig.sim[ankle].position = 670;  // directly setting sim position of servo5
  rig.sim[ankle].target = 150;  // directly setting sim position of servo5
  rig.sim[foot].position = 700;  // directly setting sim position of servo5
  rig.sim[foot].target = 550;  // directly setting sim position of servo5
  rig.channel.ReadAsyncAll(LssPosition|LssTarget);
  rig.update();
  if(rig.ankle.position!=670 && rig.ankle.target!=150 && rig.hip.position==420 && rig.hip.target==50 && rig.foot.position!=700 && rig.foot.target!=550) {
    // second update should read the next servo
    rig.update();
    if(rig.ankle.position==670 && rig.ankle.target==150 && rig.hip.position==420 && rig.hip.target==50 && rig.foot.position!=700 && rig.foot.target!=550) {
      rig.update();
      if(rig.ankle.position==670 && rig.ankle.target==150 && rig.hip.position==420 && rig.hip.target==50 && rig.foot.position==700 && rig.foot.target==550) {
        passed = true;
      } else
        test += " (3rd pass)";
    } else
      test += " (2nd pass)";
  } else 
    test += " (1st pass)";
  test_report(test, passed && rig.channel.txn_current==rig.channel.txn_next);
}

void test_async_multiple_read_loop()
{
  bool passed = true;
  String test(F("test_async_multiple_read_loop"));
  LssTestRig rig;
  short hip = rig.hip.id;
  short ankle = rig.ankle.id;
  short foot = rig.foot.id;
  for(int i=0; i<10 && passed; i++) {
    rig.sim[ankle].position = i*3;  // directly setting sim position of servo5
    rig.sim[ankle].target = i*6;  // directly setting sim position of servo5
    rig.sim[hip].position = i*17;  // directly setting sim position of servo5
    rig.sim[hip].target = i*5;  // directly setting sim position of servo5
    rig.sim[foot].position = i*11;  // directly setting sim position of servo5
    rig.sim[foot].target = i*14;  // directly setting sim position of servo5
    
    AsyncToken token = rig.channel.ReadAsyncAll(LssPosition|LssTarget);
    if(!rig.channel.waitFor(token)) {
      test += "  (timeout)";
      passed = false;
      break;
    }

    if(rig.ankle.position!=i*3 || rig.ankle.target!=i*6 || 
      rig.hip.position!=i*17 || rig.hip.target!=i*5 ||
      rig.foot.position!=i*11 || rig.foot.target!=i*14) {
        test += "  (mismatch)";
        passed = false;
    }
    if(rig.channel.txn_current!=rig.channel.txn_next) {
      test += "  (txn)";
      passed = false;
    }
    if(!passed) {
      // print the iteration#
      test += "(#";
      test += i;
      test += ')';
    }
  }
  test_report(test, passed);
}

void test_async()
{
  test_head(F("Asynchronous Reads"));
  test_async_req_increments_txn();
  test_async_only_token_holder_sends();
  test_async_servo_sends_when_holding_token();
  test_async_txn_increments_when_async_done();
  test_async_position();
  test_async_position_target();
  test_async_position_target_with_2_servos();
  test_async_position_target_with_2_servos_rev_order();
  test_async_position_target_read_all_3servos();
  test_async_multiple_read_loop();
}
