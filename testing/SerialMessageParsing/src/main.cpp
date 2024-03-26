// Project to test EmotiBitSerial message parsing
#include <Arduino.h>
#include <EmotiBitSerial.h>

String STR_PASS = "PASS";
String STR_FAIL = "FAIL";
const uint8_t MAX_CASE_NUM = 10;
struct TestCase{
  String name = "";
  String testString = "";
  bool result = false;
  TestCase();

  TestCase(String t_name, String t_str, bool res) : name(t_name), testString(t_str), result(res) {}
}test_bank[MAX_CASE_NUM];

TestCase::TestCase() {}

void setup()
{
  Serial.begin(115200);
  uint8_t caseCount = 0;
  test_bank[0] = TestCase("GOOD INPUT: ", "@TT, PL~", true);caseCount++;
  test_bank[1] = TestCase("NO STAR CHAR: ", "TT, payload", false);caseCount++;
  test_bank[2] = TestCase("NO END CHAR: ", "@TT, payload", false);caseCount++;
  test_bank[3] = TestCase("EMPTY PAYLOAD (NO SPACE): ", "@TT,~", false);caseCount++;
  test_bank[4] = TestCase("EMPTY PAYLOAD (WITH SPACE): ", "@TT, ~", false);caseCount++;
  test_bank[5] = TestCase("ONLY TYPETAG ", "@TT~", true);caseCount++;
  test_bank[6] = TestCase("EMPTY PAYLOAD (WITH 2 SPACES): ", "@TT,  ~", true);caseCount++;
  String typetag, payload;
  for(int i=0;i<caseCount;i++)
  {
      Serial.print(test_bank[i].testString); Serial.print( " : TEST-" );
    if(EmotiBitSerial::parseSerialMessage(test_bank[i].testString, typetag, payload) == test_bank[i].result)
    {
      Serial.println(STR_PASS);
    }
    else
    {
      Serial.println(STR_FAIL);
    }
  }
  while(1); 
}

void loop()
{

}