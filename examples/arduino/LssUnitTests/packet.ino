
void test_parse_packet(const char* pkt, LynxPacket packet) 
{
  LynxPacket p;
  String test = "parsePacket:";
  test += pkt;
  bool parsed = p.parse(pkt);
  test += (p==packet) ? "=":"!=";
  test += p.toString();
  test += parsed ? ":parsed":":invalid";
  test_report(test, 
      (parsed || packet.command==LssInvalid) && p==packet);
}

void test_serialize_packet(const char* pkt, LynxPacket packet) 
{
  char buf[128];
  String test = "serializePacket:";
  test += pkt;
  char* serialized = packet.serialize(buf);
  bool passed = serialized!=NULL && strcmp(pkt, buf)==0;
  if(!passed) {
    test += "(returned ";
    test += buf;
    test += ")";
  }
  test_report(test, passed);
}

void test_packet_copy_constructor()
{
  LynxPacket a(16, LssQuery|LssPosition|LssDegrees, 125);
  LynxPacket b(a);
  test_report(F("test_packet_copy_constructor"), a==b);
}

void test_packet_copy_assignment()
{
  LynxPacket a(16, LssQuery|LssPosition|LssDegrees, 125);
  LynxPacket b(5, LssTarget, 34);
  b = a; 
  test_report(F("test_packet_copy_assignment"), a==b);
}

void test_packet_serialization()
{
  test_head(F("serialize packets C-strings"));
  test_serialize_packet("6QD1800", LynxPacket(6, LssQuery|LssPosition|LssDegrees, 1800));
  test_serialize_packet("16QD1800", LynxPacket(16, LssQuery|LssPosition|LssDegrees, 1800));
  test_serialize_packet("6QP3650", LynxPacket(6, LssQuery|LssPosition|LssPulse, 3650));
  test_serialize_packet("16QP3650", LynxPacket(16, LssQuery|LssPosition|LssPulse, 3650));
  test_serialize_packet("6QP-3650", LynxPacket(6, LssQuery|LssPosition|LssPulse, -3650));
  test_serialize_packet("16QP-3650", LynxPacket(16, LssQuery|LssPosition|LssPulse, -3650));
  test_serialize_packet("6P-3650", LynxPacket(6, LssPosition|LssPulse, -3650));
  test_serialize_packet("16P-3650", LynxPacket(16, LssPosition|LssPulse, -3650));
  test_serialize_packet("6D-3650", LynxPacket(6, LssPosition|LssDegrees, -3650));
  test_serialize_packet("16D-3650", LynxPacket(16, LssPosition|LssDegrees, -3650));
  test_serialize_packet("6CFD3650", LynxPacket(6, LssConfig|LssFirstPosition|LssDegrees, 3650));
  test_serialize_packet("16CFD3650", LynxPacket(16, LssConfig|LssFirstPosition|LssDegrees, 3650));
  test_serialize_packet("6CFD-650", LynxPacket(6, LssConfig|LssFirstPosition|LssDegrees, -650));
  test_serialize_packet("16CFD-650", LynxPacket(16, LssConfig|LssFirstPosition|LssDegrees, -650));
  test_serialize_packet("6CFP3650", LynxPacket(6, LssConfig|LssFirstPosition|LssPulse, 3650));
  test_serialize_packet("16CFP3650", LynxPacket(16, LssConfig|LssFirstPosition|LssPulse, 3650));
  test_serialize_packet("6CFP-650", LynxPacket(6, LssConfig|LssFirstPosition|LssPulse, -650));
  test_serialize_packet("16CFP-650", LynxPacket(16, LssConfig|LssFirstPosition|LssPulse, -650));
}

void test_packet_parsing()
{
  test_head(F("parse packets with servo ID and command"));
  test_parse_packet("1L", LynxPacket(1, LssLimp));
  test_parse_packet("2Q", LynxPacket(2,LssQuery));
  test_parse_packet("24LED", LynxPacket(24,LssLEDColor));

  test_head(F("parse packets with servo ID, command and values"));
  test_parse_packet("2D905", LynxPacket(2,LssPosition|LssDegrees,905));
  test_parse_packet("2D-905", LynxPacket(2,LssPosition|LssDegrees,-905));
  test_parse_packet("24LED5", LynxPacket(24,LssLEDColor,5));
  test_parse_packet("02D-905", LynxPacket(2,LssPosition|LssDegrees,-905));

  test_head(F("parsing packets with errors returns invalid"));
  test_parse_packet("-2Q", LynxPacket(0,LssInvalid));
  test_parse_packet("2@Q", LynxPacket(0,LssInvalid));
  test_parse_packet("2.Q", LynxPacket(0,LssInvalid));
  test_parse_packet(".2Q", LynxPacket(0,LssInvalid));
  test_parse_packet("Q2Q", LynxPacket(0,LssInvalid)); // no ID
  test_parse_packet("", LynxPacket(0,LssInvalid)); // nothing
}

void test_packet_class()
{
  test_head(F("Packet class"));
  test_packet_copy_constructor();
  test_packet_copy_assignment();
  test_packet_parsing();
  test_packet_serialization();
}
