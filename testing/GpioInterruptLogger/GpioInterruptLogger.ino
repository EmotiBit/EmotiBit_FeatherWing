// COde to read the timestamp of 1KHz wave
//#define DEBUG_SERIAL
#ifdef ARDUINO_FEATHER_ESP32
#define PIN_IN_1 5
#define PIN_OUT_1 18
#elif defined ADAFRUIT_FEATHER_M0
#define PIN_IN_1 24
#define PIN_OUT_1 23
#define PIN_IN_2 22
#define PIN_OUT_2 5
#endif
#include <DoubleBufferFloat.h>
#include <String.h>

DoubleBufferFloat us_timeBuffer_IN1(50);
DoubleBufferFloat ms_timeBuffer_IN1(50);
DoubleBufferFloat us_timeBuffer_IN2(50);
DoubleBufferFloat ms_timeBuffer_IN2(50);
#ifdef ARDUINO_FEATHER_ESP32
void IRAM_ATTR isr_pin1() {
	//counter++;
  // push to Double Buffer
  us_timeBuffer_IN1.push_back(micros());
  ms_timeBuffer_IN1.push_back(millis());
  digitalWrite(PIN_OUT_1, !digitalRead(PIN_OUT_1));  // toggle a pin everytime we get a rising edge. Half the freq of PIN_IN_1
}
#elif defined ADAFRUIT_FEATHER_M0
void isr_pin1() {
	//counter++;
  // push to Double Buffer
  us_timeBuffer_IN1.push_back(micros());
  ms_timeBuffer_IN1.push_back(millis());
  digitalWrite(PIN_OUT_1, !digitalRead(PIN_OUT_1));  // toggle a pin everytime we get a rising edge. Half the freq of PIN_IN_1
}

void isr_pin2() {
	//counter++;
  // push to Double Buffer
  us_timeBuffer_IN2.push_back(micros());
  ms_timeBuffer_IN2.push_back(millis());
  digitalWrite(PIN_OUT_2, !digitalRead(PIN_OUT_2));  // toggle a pin everytime we get a rising edge. Half the freq of PIN_IN_2
}
#endif


void getData(DoubleBufferFloat *buf_us, DoubleBufferFloat *buf_ms, String pin)
{
  float *us_timeData; float *ms_timeData;
  uint32_t tu, tm; // dummy variables.
  size_t dataAvailableuS, dataAvailablemS;
  // get the data
  #ifdef DEBUG_SERIAL
  Serial.println("getting data");
  #endif
  dataAvailableuS = buf_us->getData(&us_timeData, &tu, false);
  dataAvailablemS = buf_ms->getData(&ms_timeData, &tm, false);
  #ifdef DEBUG_SERIAL
  Serial.println("got data");
  #endif
  if(dataAvailableuS == dataAvailablemS)
  {
    #ifdef DEBUG_SERIAL
    Serial.print("dataAvailableuS"); Serial.println(dataAvailableuS);
    #endif
    for (uint8_t i = 0; i < dataAvailableuS; i++) 
    {
      String str = pin + ":" + String(ms_timeData[i]) + ":" + String(us_timeData[i])  ;
      Serial.println(str); 
    }
  }
  else
  {
    Serial.print("dataAvailableuS: "); Serial.println(dataAvailableuS);
    Serial.print("dataAvailablemS: "); Serial.println(dataAvailablemS);
    Serial.println("mismatch in buffer lengths");
    //while(1);
  }
}
void setup() {
	Serial.begin(2000000);
	pinMode(PIN_IN_1, INPUT);
  pinMode(PIN_OUT_1, OUTPUT);
	pinMode(PIN_IN_2, INPUT);
  pinMode(PIN_OUT_2, OUTPUT);
  #ifdef ARDUINO_FEATHER_ESP32
	attachInterrupt(PIN_IN_1, isr_pin1, RISING);
  #elif defined ADAFRUIT_FEATHER_M0
  attachInterrupt( digitalPinToInterrupt( PIN_IN_1 ), isr_pin1, RISING );
  attachInterrupt( digitalPinToInterrupt( PIN_IN_2 ), isr_pin2, RISING );
  #endif
}

void loop() {
  uint32_t timeNow = millis();

  while(1)
  {
    if(millis() - timeNow > 100)
    {
      #ifdef DEBUG_SERIAL
      Serial.println("timeNow > millis()");
      #endif
      timeNow = millis();
      #ifdef DEBUG_SERIAL
      Serial.println("swapping buffer");
      #endif
      us_timeBuffer_IN1.swap(); // swap buffers before reading
      ms_timeBuffer_IN1.swap(); // swap buffers before reading
      us_timeBuffer_IN2.swap(); // swap buffers before reading
      ms_timeBuffer_IN2.swap(); // swap buffers before reading
      #ifdef DEBUG_SERIAL
      Serial.println("swapped buffer");
      #endif
      getData(&us_timeBuffer_IN1, &ms_timeBuffer_IN1, String("PIN_1"));
      getData(&us_timeBuffer_IN2, &ms_timeBuffer_IN2, String("PIN_2"));
      
    }
    else
    {
      // do ntohing
    }
  }
}