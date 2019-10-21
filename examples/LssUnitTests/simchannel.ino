

void test_simchannel_action()
{
  SimChannel sim;
  sim.begin({1,5,6});
  sim.print("#1D450\r");
  test_report(F("test_simchannel_action"), sim.servos[0].position==450);
}

void test_simchannel_query()
{
  SimChannel sim;
  sim.begin({1,5,6});
  sim.print("#1QD\r");
  String r = sim.readStringUntil('\r');
  test_report(F("test_simchannel_query"), r.equals("*1QD1800"));
}

void test_simchannel()
{
  test_head(F("SimServo and SimChannel class"));
  test_simchannel_action();
  test_simchannel_query();
}
