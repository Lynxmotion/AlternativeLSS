
void test_charstream_1w_1r()
{
  CharStream stream;
  stream.println("test");
  String test = stream.readStringUntil('\r');
  test_report("test_charstream_1w_1r", test=="test");
}

void test_charstream_2w_1r()
{
  CharStream stream;
  stream.print("test");
  stream.println(" me");
  String test = stream.readStringUntil('\r');
  test_report("test_charstream_2w_1r", test=="test me");
}

void test_charstream_3w_2r()
{
  bool pass = true;
  CharStream stream;
  stream.print("char");
  stream.println(" stream");
  stream.println("is working");
  String test = stream.readStringUntil('\r');
  if(stream.read()=='\n' && test.equals("char stream")) {
    test = stream.readStringUntil('\r');
    pass = test.equals("is working") && stream.read()=='\n';
  } else
    pass=false;
  test_report("test_charstream_3w_2r", pass);
}

void lynx_charstream_tests()
{
  test_head("CharStream class");
  test_charstream_1w_1r();
  test_charstream_2w_1r();
  test_charstream_3w_2r();
}
