/*
 * Code to read the entire OTP on the SI-7013
 * NOTE::: when using, make sure to change the i2c pins of the ALT SI7013 to  the native i2c of the feather(not the emotibit i2c)
 */
#include <Wire.h>
// SI7013-A20 I2C address is 0x40(64)
#define Addr 0x41
#define CMD_OTP_READ 0x84
#define CMD_OTP_WRITE 0xC5

//TwoWire EmotiBit_i2c(&sercom1, 11, 13);
void setup()
{
  // Initialise I2C communication as MASTER
  // Flush the I2C
  Serial.begin(9600);
  Wire.begin(); // MUST call Wire.begin() befre calling Si7013::setup()
  //EmotiBit_i2c->setClock(100000);
//  pinPeripheral(11, PIO_SERCOM);
//  pinPeripheral(13, PIO_SERCOM);
  Serial.println("Flushing I2C....");
  //Wire.flush();
  // Initialise serial communication, set baud rate = 9600
  
  uint8_t initAddr = 130; // 0x82
  uint8_t finalAddr = 183; // 0xB7
  uint8_t addrCount = initAddr;
  uint8_t counter = 1;
  uint8_t testStartAddr = (uint8_t)0xA0;
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  while (!Serial.available())
  {
    Serial.println("enter any key to proceed");
    delay(1000); 
  }
  Serial.read();
  Serial.println("make sure to change the i2c pins of the ALT SI7013 to  the native i2c of the feather");
    Serial.println("Reading from the OTP");
    while (addrCount <= finalAddr)
    {
      Serial.print(counter); Serial.print("--:");
      readOtp(addrCount);
      addrCount++;
      counter++;
    }

  
}

void loop()
{
  //Do nothing 
}

char readOtp(uint8_t addr)
{
  uint8_t otpByte=0;
  Serial.print("0x"); Serial.print(addr, HEX);
  Wire.beginTransmission(Addr);
  Wire.write(CMD_OTP_READ);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(Addr, 1);
  while(!Wire.available());
  otpByte = Wire.read();
  Serial.print(" : "); Serial.println(otpByte);
  return (char)otpByte;
}
