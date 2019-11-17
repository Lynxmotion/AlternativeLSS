
void test_charstream_1w_1r()
{
  CharStream stream;
  stream.println("test");
  String test = stream.readStringUntil('\r');
  test_report(F("test_charstream_1w_1r"), test=="test");
}

void test_charstream_2w_1r()
{
  CharStream stream;
  stream.print("test");
  stream.println(" me");
  String test = stream.readStringUntil('\r');
  test_report(F("test_charstream_2w_1r"), test=="test me");
}

void test_charstream_3w_2r()
{
  bool pass = true;
  CharStream stream;
  stream.print(F("char"));
  stream.println(F(" stream"));
  stream.println(F("is working"));
  String test = stream.readStringUntil('\r');
  if(stream.read()=='\n' && test.equals("char stream")) {
    test = stream.readStringUntil('\r');
    pass = test.equals("is working") && stream.read()=='\n';
  } else
    pass=false;
  test_report(F("test_charstream_3w_2r"), pass);
}

void lynx_charstream_tests()
{
  test_head(F("CharStream class"));
  test_charstream_1w_1r();
  test_charstream_2w_1r();
  test_charstream_3w_2r();
}
