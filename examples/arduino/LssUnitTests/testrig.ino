
void test_rig_scan()
{
  LssTestRig rig;
  bool passed = rig.scan();    // send out position queries
  test_report(F("test_rig_scan"), passed);
}

void test_rig_channel_has_hip()
{
  LssTestRig rig;
  test_report(F("test_rig_channel_has_hip"), rig.sim.contains(1));
}

void test_rig_channel_hip_is_id1()
{
  LssTestRig rig;
  test_report(F("test_rig_channel_hip_is_id1"), rig.hip.id==1);
}

void test_rig_sim_hip_is_id1()
{
  LssTestRig rig;
  test_report(F("test_rig_sim_hip_is_id1"), rig.sim.contains(1));
}

void test_rig_channel_has_ankle()
{
  LssTestRig rig;
  test_report(F("test_rig_channel_has_ankle"), rig.sim.contains(5));
}

void test_rig_channel_ankle_is_id5()
{
  LssTestRig rig;
  test_report(F("test_rig_channel_ankle_is_id5"), rig.ankle.id==5);
}

void test_rig_sim_ankle_is_id5()
{
  LssTestRig rig;
  test_report(F("test_rig_sim_ankle_is_id5"), rig.sim.contains(5));
}

void test_rig_channel_has_foot()
{
  LssTestRig rig;
  test_report(F("test_rig_channel_has_foot"), rig.sim.contains(6));
}

void test_rig_channel_foot_is_id6()
{
  LssTestRig rig;
  test_report(F("test_rig_channel_foot_is_id6"), rig.foot.id==6);
}

void test_rig_sim_foot_is_id6()
{
  LssTestRig rig;
  test_report(F("test_rig_sim_foot_is_id6"), rig.sim.contains(6));
}

void test_rig_channel_1()
{
  LssTestRig rig;
  test_report(F("test_rig_channel[1]"), rig.channel[1].id==1);
}

void test_rig_channel_5()
{
  LssTestRig rig;
  test_report(F("test_rig_channel[5]"), rig.channel[5].id==5);
}

void test_rig_channel_6()
{
  LssTestRig rig;
  test_report(F("test_rig_channel[6]"), rig.channel[6].id==6);
}

void test_rig_sim_1()
{
  LssTestRig rig;
  test_report(F("test_rig_sim[1]"), rig.sim[1].id==1);
}

void test_rig_sim_5()
{
  LssTestRig rig;
  test_report(F("test_rig_sim[5]"), rig.sim[5].id==5);
}

void test_rig_sim_6()
{
  LssTestRig rig;
  test_report(F("test_rig_sim[6]"), rig.sim[6].id==6);
}

void test_rig()
{
  test_head(F("Test Rig"));
  test_rig_scan();
  test_rig_channel_has_hip();
  test_rig_channel_hip_is_id1();
  test_rig_sim_hip_is_id1();

  test_rig_channel_has_ankle();
  test_rig_channel_ankle_is_id5();
  test_rig_sim_ankle_is_id5();

  test_rig_channel_has_foot();
  test_rig_channel_foot_is_id6();
  test_rig_sim_foot_is_id6();

  test_rig_channel_1();
  test_rig_channel_5();
  test_rig_channel_6();

  test_rig_sim_1();
  test_rig_sim_5();
  test_rig_sim_6();

}
