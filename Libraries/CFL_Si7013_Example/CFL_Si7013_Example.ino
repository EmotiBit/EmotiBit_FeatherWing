/*
Example using CFL_Si7013 Arduino library tp measure humidity + temperature
Copyright (c) 2018 Connected Future Labs. All rights reserved. http://www.connectedfuturelabs.com/
See Si7013 datasheet at https://www.silabs.com/documents/public/data-sheets/Si7013-A20.pdf

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "CFL_Si7013.h"

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
#define PRINT_SI7013_DATA 1
#define PRINT_SI7013_TIMING 0

#define BOARD_ADAFRUIT_FEATHER_M0
/////////////////////MOSFET//////////////////////////////
#if defined(BOARD_ADAFRUIT_FEATHER_M0)
  int MOS = 5 ;//gpio pin assigned ot the mosfet
#elif defined(BOARD_ADAFRUIT_FEATHER_NRF52)
  int MOS = 27;//gpio pin assigned ot the mosfet
#endif

Si7013 sensor;
uint32_t loopStart; 
uint32_t completeMeasStart;
float humidity;
float temperature;
float adc;
int loopTime;
int humidityMeasTime;
int adcMeasTime;
int startMeasTime;

void setup()
{
  Serial.begin(9600);
  pinMode(MOS,OUTPUT);
  digitalWrite(MOS,LOW);// Switch is ON. hence, The GSR is powered

  Wire.begin();  // MUST call Wire.begin() befre calling Si7013::setup()
	sensor.setup();
  sensor.changeSetting(Si7013::Settings::RESOLUTION_H12_T14);
  sensor.changeSetting(Si7013::Settings::ADC_NORMAL);
  sensor.changeSetting(Si7013::Settings::VIN_UNBUFFERED);
  sensor.changeSetting(Si7013::Settings::VREFP_VDDA);
  sensor.changeSetting(Si7013::Settings::ADC_NO_HOLD);
  sensor.startHumidityTempMeasurement();

  completeMeasStart = millis();
  loopStart = millis();
}

void loop() {
	while (true) {
		if (PRINT_SI7013_TIMING) { // Print sensor measurement delays 
			loopTime = millis() - loopStart;
			loopStart = millis();
		}

		if (sensor.getStatus() == Si7013::STATUS_IDLE) {
			if (sensor.isHumidityNew() == true) {
				humidity = sensor.getHumidity();
				temperature = sensor.getPreviousTemperature();
				if (PRINT_SI7013_TIMING) { // Print sensor measurement delays 
					humidityMeasTime = millis() - completeMeasStart;
					completeMeasStart = millis();
				}
				sensor.startAdcMeasurement();
			}
			if (sensor.isAdcNew() == true) {
				adc = sensor.getAdc() / 500.f;
				if (PRINT_SI7013_TIMING) { // Print sensor measurement delays 
					adcMeasTime = millis() - completeMeasStart;
					completeMeasStart = millis();
				}
				sensor.startHumidityTempMeasurement();
			}
			Serial.println("");
			if (PRINT_SI7013_DATA) {
				Serial.print(humidity);
				Serial.print(",");
				Serial.print(temperature);
				Serial.print(",");
				Serial.print(adc);
			}
			if (PRINT_SI7013_TIMING) { // Print sensor measurement delays 
				Serial.print(",");
				Serial.print(humidityMeasTime);
				Serial.print(",");
				Serial.print(adcMeasTime);
				Serial.print(",");
				Serial.print(loopTime);
			}
			
		}
		if (PRINT_SI7013_TIMING) { // Print sensor measurement delays 
			Serial.print(",");
			Serial.print(loopTime);
		}
	}
}
  
