// COde to read the timestamp of 1KHz wave
//#define DEBUG_SERIAL
#ifdef ARDUINO_FEATHER_ESP32
#define PIN_IN 5
#define PIN_OUT 18
#elif defined ADAFRUIT_FEATHER_M0
#define PIN_IN 24
#define PIN_OUT 23
#endif
#include <DoubleBufferFloat.h>
#include <String.h>

DoubleBufferFloat us_timeBuffer(50);
DoubleBufferFloat ms_timeBuffer(50);
#ifdef ARDUINO_FEATHER_ESP32
void IRAM_ATTR isr() {
	//counter++;
  // push to Double Buffer
  us_timeBuffer.push_back(micros());
  ms_timeBuffer.push_back(millis());
  digitalWrite(PIN_OUT, !digitalRead(PIN_OUT));  // toggle a pin everytime we get a rising edge. Half the freq of PIN_IN
}
#elif defined ADAFRUIT_FEATHER_M0
void isr() {
	//counter++;
  // push to Double Buffer
  us_timeBuffer.push_back(micros());
  ms_timeBuffer.push_back(millis());
  digitalWrite(PIN_OUT, !digitalRead(PIN_OUT));  // toggle a pin everytime we get a rising edge. Half the freq of PIN_IN
}

#endif

void setup() {
	Serial.begin(2000000);
	pinMode(PIN_IN, INPUT);
  pinMode(PIN_OUT, OUTPUT);
  #ifdef ARDUINO_FEATHER_ESP32
	attachInterrupt(PIN_IN, isr, RISING);
  #elif defined ADAFRUIT_FEATHER_M0
  attachInterrupt( digitalPinToInterrupt( PIN_IN ), isr, RISING );
  #endif
}

void loop() {
  uint32_t timeNow = millis();
  float *us_timeData; float *ms_timeData;
  uint32_t tu, tm; // dummy variables.
  size_t dataAvailableuS, dataAvailablemS;
  while(1)
  {
    if(millis() - timeNow > 200)
    {
      #ifdef DEBUG_SERIAL
      Serial.println("timeNow > millis()");
      #endif
      timeNow = millis();
      #ifdef DEBUG_SERIAL
      Serial.println("swapping buffer");
      #endif
      us_timeBuffer.swap(); // swap buffers before reading
      ms_timeBuffer.swap(); // swap buffers before reading
      #ifdef DEBUG_SERIAL
      Serial.println("swapped buffer");
      #endif
      // get the data
      #ifdef DEBUG_SERIAL
      Serial.println("getting data");
      #endif
      dataAvailableuS = us_timeBuffer.getData(&us_timeData, &tu, false);
      dataAvailablemS = ms_timeBuffer.getData(&ms_timeData, &tm, false);
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
          String str = String(ms_timeData[i]) + ":" + String(us_timeData[i])  ;
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
    else
    {
      // do ntohing
    }
  }
}