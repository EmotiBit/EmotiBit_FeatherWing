/*
 * Code to read the entire OTP on the SI-7013
 * NOTE::: when using, make sure to change the i2c pins of the ALT SI7013 to  the native i2c of the feather(not the emotibit i2c)
 */
#include <Wire.h>
#include "wiring_private.h"


#define USE_ALT_SI7013
#define CMD_OTP_READ 0x84
#define CMD_OTP_WRITE 0xC5

#ifdef USE_ALT_SI7013
#define Addr 0x41
#else
#define Addr 0x40
#endif


TwoWire EmotiBit_i2c(&sercom1, 11, 13);

int hibernatePin = 6;//gpio pin assigned ot the mosfet


void setup()
{
  // Initialise I2C communication as MASTER
  Serial.begin(9600);
  Serial.println("Activating the MOSFET");
  pinMode(hibernatePin, OUTPUT);
  Serial.println("Hibernate LOW");
  digitalWrite(hibernatePin, LOW);// Switch is ON. hence, The EmotiBit is powered

  while (!Serial.available())
  {
	  Serial.println("enter any key to proceed");
	  delay(1000);
  }Serial.read();
  Serial.println("Reading all the memory locations in the OTP");
  EmotiBit_i2c.begin();
  EmotiBit_i2c.setClock(100000);
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  Serial.println("Flushing I2C....");
  EmotiBit_i2c.flush();
  
  uint8_t initAddr = 130; // 0x82
  uint8_t finalAddr = 183; // 0xB7
  uint8_t addrCount = initAddr;
  uint8_t counter = 1;
  uint8_t testStartAddr = (uint8_t)0xA0;
  
  EmotiBit_i2c.beginTransmission(Addr);
  // Stop I2C transmission
  uint8_t i2cResponse = 0;
  EmotiBit_i2c.write((int)0x00);
  i2cResponse = EmotiBit_i2c.endTransmission();
  Serial.print("Response i2c: ");Serial.println(i2cResponse);
  if(i2cResponse)
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
	EmotiBit_i2c.beginTransmission(Addr);
	EmotiBit_i2c.write(CMD_OTP_READ);
	EmotiBit_i2c.write(addr);
	EmotiBit_i2c.endTransmission();
	EmotiBit_i2c.requestFrom(Addr, 1);
	while(!EmotiBit_i2c.available());
	otpByte = EmotiBit_i2c.read();
	Serial.print(" : "); Serial.println(otpByte);
	return (char)otpByte;

}