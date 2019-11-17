
void test_parse_command(const char* cmd, unsigned long parsed) 
{
  String test = "parseCommand:";
  test += cmd;
  
  LssCommands val = LynxPacket::parseCommand(cmd);
  test += ':';
  test += String((unsigned long)val, HEX);
  bool result = val==(LssCommands)parsed;
  if(*cmd!=0) {
    // unparsed chars left
    test += " (unparsed chars)";
    result = false;
  }
  test_report(test, result);
}

void test_serialize_command(const char* pkt, LssCommands cmd) 
{
  char buf[128];
  String test = "serializeCommand:";
  test += String((unsigned long)cmd, HEX);
  test += ':';
  test += pkt;
  char* serialized = LynxPacket::commandCode(cmd, buf);
  bool passed = serialized!=NULL && strcmp(pkt, buf)==0;
  if(!passed) {
    test += "(returned ";
    test += (serialized==NULL) ? "null" : *buf ? buf : "blank";
    test += ")";
  }
  test_report(test, passed);
}

void test_command_parsing()
{
  test_head(F("parse commands using fast string Trie"));

  // action commands
  test_parse_command("L", LssLimp);
  test_parse_command("H", LssHaltAndHold);
  //test_parse_command("T", LssTimedMove);
  //test_parse_command("S", LssSpeed);
  //test_parse_command("MD", LssMove|LssDegrees);
  test_parse_command("O", LssOriginOffset);
  test_parse_command("AR", LssAngularRange);
  test_parse_command("P", LssPosition|LssPulse);
  test_parse_command("D", LssPosition|LssDegrees);
  test_parse_command("WD", LssWheelMode|LssDegrees);
  test_parse_command("WR", LssWheelMode|LssRPM);
  test_parse_command("SD", LssMaxSpeed|LssDegrees);
  test_parse_command("SR", LssMaxSpeed|LssRPM);
  test_parse_command("AS", LssAngularStiffness);
  test_parse_command("LED", LssLEDColor);
  test_parse_command("B", LssBaudRate);
  test_parse_command("G", LssGyreDirection);

  // query commands
  test_parse_command("QO", LssQuery|LssOriginOffset);
  test_parse_command("QAR", LssQuery|LssAngularRange);
  test_parse_command("QP", LssQuery|LssPosition|LssPulse);
  test_parse_command("QD", LssQuery|LssPosition|LssDegrees);
  test_parse_command("QWD", LssQuery|LssWheelMode|LssDegrees);
  test_parse_command("QWR", LssQuery|LssWheelMode|LssRPM);
  test_parse_command("QSD", LssQuery|LssMaxSpeed|LssDegrees);
  test_parse_command("QSR", LssQuery|LssMaxSpeed|LssRPM);
  test_parse_command("QAS", LssQuery|LssAngularStiffness);
  test_parse_command("QLED", LssQuery|LssLEDColor);
  test_parse_command("QID", LssQuery|LssID);
  test_parse_command("QB", LssQuery|LssBaudRate);
  test_parse_command("QG", LssQuery|LssGyreDirection);
  //test_parse_command("QFP", LssQuery|LssPowerUpPosition|LssPulse);
  //test_parse_command("QFD", LssQuery|LssPowerUpPosition|LssDegrees);
  test_parse_command("QDT", LssQuery|LssTarget);
  //test_parse_command("QM", LssQuery|LssModel);
  //test_parse_command("QN", LssQuery|LssSerial);
  //test_parse_command("QF", LssQuery|LssFirmware);
  test_parse_command("Q", LssQuery);
  test_parse_command("QV", LssQuery|LssVoltage);
  test_parse_command("QT", LssQuery|LssTemperature);
  test_parse_command("QC", LssQuery|LssCurrent);

  // config commands
  test_parse_command("CO", LssConfig|LssOriginOffset);
  test_parse_command("CAR", LssConfig|LssAngularRange);
  test_parse_command("CSD", LssConfig|LssMaxSpeed|LssDegrees);
  test_parse_command("CSR", LssConfig|LssMaxSpeed|LssRPM);
  test_parse_command("CAS", LssConfig|LssAngularStiffness);
  test_parse_command("CLED", LssConfig|LssLEDColor);
  test_parse_command("CID", LssConfig|LssID);
  test_parse_command("CB", LssConfig|LssBaudRate);
  test_parse_command("CG", LssConfig|LssGyreDirection);
  //test_parse_command("CFP", LssQuery|LssPowerUpPosition|LssPulse);
  //test_parse_command("CFD", LssQuery|LssPowerUpPosition|LssDegrees);

  // test some commands with unparsed characters left over
  test_head(F("parse commands that have unmatched characters using fast string Trie"));
  test_neg( test_parse_command("PX", LssPosition|LssPulse) );
  test_neg( test_parse_command("QBy", LssQuery|LssBaudRate) );
  test_neg( test_parse_command("QLEDx", LssQuery|LssLEDColor) );
  test_neg( test_parse_command("CLED-", LssConfig|LssLEDColor) );
}

void test_serialize_commands()
{
  test_head(F("generate command codes from bitmasks"));

  // action commands
  test_serialize_command("L", LssLimp);
  test_serialize_command("H", LssHaltAndHold);
  //test_serialize_command("T", LssTimedMove);
  //test_serialize_command("S", LssSpeed);
  //test_serialize_command("MD", LssMove|LssDegrees);
  test_serialize_command("O", LssOriginOffset);
  test_serialize_command("AR", LssAngularRange);
  test_serialize_command("P", LssPosition|LssPulse);
  test_serialize_command("D", LssPosition|LssDegrees);
  test_serialize_command("WD", LssWheelMode|LssDegrees);
  test_serialize_command("WR", LssWheelMode|LssRPM);
  test_serialize_command("SD", LssMaxSpeed|LssDegrees);
  test_serialize_command("SR", LssMaxSpeed|LssRPM);
  test_serialize_command("AS", LssAngularStiffness);
  test_serialize_command("LED", LssLEDColor);
  test_serialize_command("B", LssBaudRate);
  test_serialize_command("G", LssGyreDirection);

  // query commands
  test_serialize_command("QO", LssQuery|LssOriginOffset);
  test_serialize_command("QAR", LssQuery|LssAngularRange);
  test_serialize_command("QP", LssQuery|LssPosition|LssPulse);
  test_serialize_command("QD", LssQuery|LssPosition|LssDegrees);
  test_serialize_command("QWD", LssQuery|LssWheelMode|LssDegrees);
  test_serialize_command("QWR", LssQuery|LssWheelMode|LssRPM);
  test_serialize_command("QSD", LssQuery|LssMaxSpeed|LssDegrees);
  test_serialize_command("QSR", LssQuery|LssMaxSpeed|LssRPM);
  test_serialize_command("QAS", LssQuery|LssAngularStiffness);
  test_serialize_command("QLED", LssQuery|LssLEDColor);
  test_serialize_command("QID", LssQuery|LssID);
  test_serialize_command("QB", LssQuery|LssBaudRate);
  test_serialize_command("QG", LssQuery|LssGyreDirection);
  test_serialize_command("QFP", LssQuery|LssFirstPosition|LssPulse);
  test_serialize_command("QFD", LssQuery|LssFirstPosition|LssDegrees);
  test_serialize_command("QDT", LssQuery|LssTarget);
  //test_serialize_command("QM", LssQuery|LssModel);
  //test_serialize_command("QN", LssQuery|LssSerial);
  //test_serialize_command("QF", LssQuery|LssFirmware);
  test_serialize_command("Q", LssQuery);
  test_serialize_command("QV", LssQuery|LssVoltage);
  test_serialize_command("QT", LssQuery|LssTemperature);
  test_serialize_command("QC", LssQuery|LssCurrent);

  // config commands
  test_serialize_command("CO", LssConfig|LssOriginOffset);
  test_serialize_command("CAR", LssConfig|LssAngularRange);
  test_serialize_command("CSD", LssConfig|LssMaxSpeed|LssDegrees);
  test_serialize_command("CSR", LssConfig|LssMaxSpeed|LssRPM);
  test_serialize_command("CAS", LssConfig|LssAngularStiffness);
  test_serialize_command("CLED", LssConfig|LssLEDColor);
  test_serialize_command("CID", LssConfig|LssID);
  test_serialize_command("CB", LssConfig|LssBaudRate);
  test_serialize_command("CG", LssConfig|LssGyreDirection);
  //test_serialize_command("CFP", LssQuery|LssPowerUpPosition|LssPulse);
  //test_serialize_command("CFD", LssQuery|LssPowerUpPosition|LssDegrees);  
}
