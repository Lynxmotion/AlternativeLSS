
void test_servo_set_position_s5_125()
{
  LssTestRig rig;
  rig.ankle.WritePosition(125);
  rig.update();             // allow the channel to read incoming data
  test_report(F("servo_set_position_s5_125"), rig.sim.servos[1].position==125);
}

void test_servo_query_read_async()
{
  String test_name = F("test_servo_set_position_read_async");
  SimChannel sim;
  LynxChannel channel("unittests");
  LynxServo s1(1);
  LynxServo s5(5);
  channel.add(s1);
  channel.add(s5);
  channel.begin(sim);
  sim.begin({1,5,6});
  sim[5].position = 670;  // directly setting sim position of servo5
  //s5.SetPosition(670);
  AsyncToken w = s5.ReadAsync(LssQuery);    // send out position queries
  channel.update();             // allow the channel to read incoming data
  if(!w.isComplete())
    test_name += F("   (async didnt complete)");
  test_report(test_name, s5.position==670);
}

void test_servo_set_position_read_async()
{
  String test_name = F("test_servo_set_position_read_async");
  SimChannel sim;
  LynxChannel channel("unittests");
  LynxServo s1(1);
  LynxServo s5(5);
  channel.add(s1);
  channel.add(s5);
  channel.begin(sim);
  sim.begin({1,5,6});
  sim[5].position = 670;  // directly setting sim position of servo5
  //s5.SetPosition(670);
  AsyncToken w = s5.ReadAsync(LssPosition);    // send out position queries
  channel.update();             // allow the channel to read incoming data
  if(!w.isComplete())
    test_name += F("   (async didnt complete)");
  test_report(test_name, s5.position==670);
}

void test_servo_set_position_read_async2()
{
  String test_name = F("test_servo_set_position_read_async2");
  LssTestRig rig;
  rig.sim[5].position = 670;  // directly setting sim position of servo5
  //rig.ankle.SetPosition(670);
  AsyncToken w = rig.ankle.ReadAsync(LssPosition);    // send out position queries
  rig.update();             // allow the channel to read incoming data
  if(!w.isComplete())
    test_name += F("   (async didnt complete)");
  test_report(test_name, rig.ankle.position==670);
}

void test_servos()
{
  test_head(F("Servo and Channel class"));
  test_servo_set_position_s5_125();
  test_servo_set_position_read_async();
  test_servo_set_position_read_async2();
}
