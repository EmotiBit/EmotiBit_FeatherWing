/*
 * Code to read the entire OTP on the SI-7013
 * NOTE::: when using, make sure to change the i2c pins of the ALT SI7013 to  the native i2c of the feather(not the emotibit i2c)
 */
#include <Wire.h>
#include "wiring_private.h"
#include "EmotiBit_Si7013.h"

#define USE_ALT_SI7013

#ifdef USE_ALT_SI7013
#define Addr 0x41
#else
#define Addr 0x40
#endif

Si7013 sensor;
TwoWire* _EmotiBit_i2c = nullptr;

int hibernatePin = 6;//gpio pin assigned ot the mosfet


void setup()
{
  // Initialise I2C communication as MASTER
  Serial.begin(9600);
  Serial.println("Activating the MOSFET");
  pinMode(hibernatePin, OUTPUT);
  Serial.println("Hibernate LOW");
  digitalWrite(hibernatePin, LOW);// Switch is ON. hence, The EmotiBit is powered
#ifdef USE_ALT_SI7013
  Serial.println("Reading the External sensor");
#else
  Serial.println("Reading the main emotiBit sensor");
#endif
  while (!Serial.available())
  {
	  Serial.println("enter any key to proceed");
	  delay(1000);
  }Serial.read();
  Serial.println("Reading all the memory locations in the OTP");
  _EmotiBit_i2c = new TwoWire(&sercom1, 11, 13);
  _EmotiBit_i2c->begin(); 
  _EmotiBit_i2c->setClock(100000);
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  Serial.println("Flushing I2C....");
  _EmotiBit_i2c->flush();

  sensor.setup(*_EmotiBit_i2c,Addr);

  uint8_t initAddr = 130; // 0x82
  uint8_t finalAddr = 183; // 0xB7
  uint8_t addrCount = initAddr;
  uint8_t counter = 1;
  uint8_t testStartAddr = (uint8_t)0xA0;
  uint8_t i2cResponse = 0;
  i2cResponse = sensor.sendCommand(0x00); // return false(0) if no sensor detected else return true(1) if sensor detected
  Serial.print("Response i2c: ");Serial.println(i2cResponse);
  if(!i2cResponse)
  {
    Serial.println("Chip not detected on the i2c line");
    Serial.println("make sure the sensor is connected and try again.");
    while(1);
  }
  Serial.println("Reading from the OTP");
  while (addrCount <= finalAddr)
  {
    Serial.print(counter); Serial.print("--:");
    readOtp(addrCount);
    addrCount++;
    counter++;
  }
  Serial.println("Reached End");
  while (1);
  
}

void loop()
{
  //Do nothing 
}

char readOtp(uint8_t addr)
{

	uint8_t otpByte=0;
	Serial.print("0x"); Serial.print(addr, HEX);
	otpByte = sensor.readRegister8(addr, true);
	Serial.print(" : "); Serial.println(otpByte);
	return (char)otpByte;

}