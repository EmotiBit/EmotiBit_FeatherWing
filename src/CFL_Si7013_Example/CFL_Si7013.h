/*
CFL_Si7013.h - Arduino library for Si7013 humidity + temperature sensor
Library provides management of sensor measurement delays to allow activities to be performed during measurement delays
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

#ifndef _CFL_SI7013_H_
#define _CFL_SI7013_H_

#include "Arduino.h"
#include <Wire.h>

class Si7013 {
  public: 

  enum class Settings {
    RESOLUTION_H12_T14,
    RESOLUTION_H08_T12,
    RESOLUTION_H10_T13,
    RESOLUTION_H11_T11,
		ENABLE_HEATER,
		DISABLE_HEATER,
    ADC_NO_HOLD,
    ADC_HOLD,
		ENABLE_THERMISTOR_CORRECTION,
		DISABLE_THERMISTOR_CORRECTION,
    ADC_FAST,
    ADC_NORMAL,
    VIN_BUFFERED,
    VIN_UNBUFFERED,
    VREFP_VDDA,
    VREFP_125V,
    VOUT_VDDD,
    VOUT_GNDD
  };


  // Status
  static const uint8_t STATUS_IDLE = 0;
  static const uint8_t STATUS_MEASURING_HUMIDITY = 1;
  static const uint8_t STATUS_MEASURING_TEMPERATURE = 2;
  static const uint8_t STATUS_MEASURING_ADC = 3;

  // Delays
  static const uint8_t DELAY_HUMIDITY12BIT_TEMP14BIT = 23;
  static const uint8_t DELAY_HUMIDITY8BIT_TEMP12BIT = 7;
  static const uint8_t DELAY_HUMIDITY10BIT_TEMP13BIT = 11;
  static const uint8_t DELAY_HUMIDITY11BIT_TEMP11BIT = 10;
  static const uint8_t DELAY_ADC_FAST = 4;
  static const uint8_t DELAY_ADC_NORMAL = 7;
  static const uint8_t DELAY_RESET = 15;
  static const uint8_t DELAY_POWER_UP = 80;
  
  // Commands  
  static const uint8_t CMD_MEASURE_HUMIDITY_HOLD = 0xE5;
  static const uint8_t CMD_MEASURE_HUMIDITY_NO_HOLD = 0xF5;
  static const uint8_t CMD_MEASURE_TEMPERATURE_HOLD = 0xE3;
  static const uint8_t CMD_MEASURE_TEMPERATURE_NO_HOLD = 0xF3;
  static const uint8_t CMD_MEASURE_ADC = 0xEE;
  static const uint8_t CMD_READ_PREVIOUS_TEMPERATURE = 0xE0;
  static const uint8_t CMD_RESET = 0xFE;  
  static const uint8_t CMD_WRITE_REGISTER_2 = 0x50;
  static const uint8_t CMD_READ_REGISTER_2 = 0x10;
  static const uint8_t CMD_WRITE_REGISTER_1 = 0xE6;
  static const uint8_t CMD_READ_REGISTER_1 = 0xE7;
  static const uint8_t CMD_WRITE_REGISTER_3 = 0x51;
  static const uint8_t CMD_READ_REGISTER_3 = 0x11;  
  static const uint8_t CMD_WRITE_COEFFICIENT = 0xC5;  
  static const uint8_t CMD_READ_COEFFICIENT = 0x84;  
  static const uint8_t CMD_READ_ELECTRONIC_ID_1ST_WORD_1 = 0xFA;
  static const uint8_t CMD_READ_ELECTRONIC_ID_1ST_WORD_2 = 0x0F;
  static const uint8_t CMD_READ_ELECTRONIC_ID_2ND_WORD_1 = 0xFC;
  static const uint8_t CMD_READ_ELECTRONIC_ID_2ND_WORD_2 = 0xC9;
  static const uint8_t CMD_READ_FIRMWARE_VERSION_1 = 0x84;
  static const uint8_t CMD_READ_FIRMWARE_VERSION_2 = 0xB8;

  static const int16_t ERROR_READ = -1;
  
  // Register values
  static const uint8_t REG1_VALUE_RESOLUTION_H12_T14 =            0b00000000;
  static const uint8_t REG1_VALUE_RESOLUTION_H08_T12 =            0b00000001;
  static const uint8_t REG1_VALUE_RESOLUTION_H10_T13 =            0b10000000;
  static const uint8_t REG1_VALUE_RESOLUTION_H11_T11 =            0b10000001;
  static const uint8_t REG1_VALUE_ENABLE_HEATER =                 0b00000010;
  static const uint8_t REG1_VALUE_DISABLE_HEATER =                0b00000000;

  static const uint8_t REG2_VALUE_ADC_NO_HOLD =                   0b01000000;
  static const uint8_t REG2_VALUE_ADC_HOLD =                      0b00000000;
  static const uint8_t REG2_VALUE_ENABLE_THERMISTOR_CORRECTION =  0b00100000;
  static const uint8_t REG2_VALUE_DISABLE_THERMISTOR_CORRECTION = 0b00000000;
  static const uint8_t REG2_VALUE_ADC_FAST =                      0b00010000;
  static const uint8_t REG2_VALUE_ADC_NORMAL =                    0b00000000;
  static const uint8_t REG2_VALUE_VIN_BUFFERED =                  0b00000100;
  static const uint8_t REG2_VALUE_VIN_UNBUFFERED =                0b00000000;
  static const uint8_t REG2_VALUE_VREFP_VDDA =                    0b00000010;
  static const uint8_t REG2_VALUE_VREFP_125V =                    0b00000000;
  static const uint8_t REG2_VALUE_VOUT_VDDD =                     0b00000001;
  static const uint8_t REG2_VALUE_VOUT_GNDD =                     0b00000000;

  
  // Register masks
  static const uint8_t REG1_MASK_HUMIDITY_TEMP_RESOLUTION =       0b10000001;
  static const uint8_t REG1_MASK_RSVD =                           0b00111010;
  static const uint8_t REG1_MASK_ENABLE_HEATER =                  0b00000010;

  static const uint8_t REG2_MASK_ADC_HOLD =                       0b01000000;
  static const uint8_t REG2_MASK_ENABLE_THERMISTOR_CORRECTION =   0b00100000;
  static const uint8_t REG2_MASK_ADC_SPEED =                      0b00010000;
  static const uint8_t REG2_MASK_RSVD =                           0b00001000;
  static const uint8_t REG2_MASK_VIN_BUF =                        0b00000100;
  static const uint8_t REG2_MASK_VREFP =                          0b00000010;
  static const uint8_t REG2_MASK_VOUT =                           0b00000001;
  
  static const uint8_t REG3_MASK_RSVD =                           0b11110000;
     
  /// \brief Constructor
  Si7013();

  /// \brief Setup the Si7013
  /// \param	address		I2C address of the Si7013
  /// \returns		true		Setup was successful
  bool setup(uint8_t address = 0x40);

  /// \brief Reset the Si7013
  /// \returns		true		Teset was successful
  bool reset();

  /// \brief Get the current status of the Si7013
  /// \returns		STATUS_IDLE						Device is ready
  /// \retval		STATUS_MEASURING_HUMIDITY		Device is busy
  /// \retval		STATUS_MEASURING_TEMPERATURE	Device is busy
  /// \retval		STATUS_MEASURING_ADC			Device is busy
  uint8_t getStatus();

  /// \brief Check for new humidity data
  /// \returns		true	There is a new humidity measurement ready
  bool isHumidityNew();

  /// \brief Check for new temperature data
  /// \returns		true	There is a new temperature measurement ready
  bool isTemperatureNew();

  /// \brief Check for new adc data
  /// \returns		true	There is a new adc measurement ready
  bool isAdcNew();

  /// \brief Change a setting on the device
  /// \param	setting	New setting (see Settings)
  /// \returns		true	Setting was successfully changed
  bool changeSetting(Settings setting);

  /// \brief Send a command to the device
  /// \param	cmd		Command (see CMD_ options)
  /// \returns		true	Command was successfully sent
  bool sendCommand(uint8_t cmd);

  /// \brief Start a specific measurement
  /// \param	cmd		Command (see CMD_MEASURE options)
  /// \returns		true	Command was successfully sent
  bool startMeasurement(uint8_t cmd);

  /// \brief Start a humidity + temperature measurement
  /// \returns		true	Measurement successfully started
  bool startHumidityTempMeasurement();


  /// \brief Start a humidity + temperature measurement
  /// \returns		true	Measurement successfully started
  bool startTempMeasurement();

  /// \brief Start a humidity + ADC measurement
  /// \returns		true	Measurement successfully started
  bool startAdcMeasurement();

  /// \brief Retrieves measured humidity data
  /// \returns		humidity value
  /// \retval		NAN			Get data failed
  float getHumidity();

  /// \brief Retrieves measured temperature data acquired with humidity measurement
  /// \returns		temeperature value
  /// \retval		NAN			Get data failed
  float getPreviousTemperature();

  /// \brief Retrieves measured temperature data
  /// \returns		temperature value
  /// \retval		NAN			Get data failed
  float getTemperature();

  /// \brief Retrieves measured adc data
  /// \returns		ADC value
  /// \retval		NAN			Get data failed
  float getAdc();

  /// \brief Sets the timeout for I2C transactions
  void setTransactionTimeout(uint16_t transactionTimeout);

  /// \brief Sets delay multiplier for measurement delays to alter delay timeouts
  void setMeasurementDelayMultiplier(float delayMultiplier);
    
  /// \brief Read the specified device register
  /// \param	reg			Register (see CMD_READ_ options)
  /// \returns		Returns		value of register
  /// \retval		ERROR_READ	Read was unsuccessful
  int16_t readRegister8(uint8_t reg);

  /// \brief Read the specified device register
  /// \param	reg		Register (see CMD_WRITE_ options)
  /// \param	value	Value to write (see REGX_VALUE_ options)
  /// \param	mask	Mask to write/change only specific bits (see REGX_MASK_ options)
  /// \returns		true	Register write was successful
  bool writeRegister8(uint8_t reg, uint8_t value, uint8_t mask = 0xAA);
  

private:

  bool _humidityNew = false;	  // New humidity data is ready
  bool _temperatureNew = false;	// New temperature data is ready
  bool _adcNew = false;         // New ADC data is ready
  //uint8_t _register1Value;  	// Stores latest register read values
  //uint8_t _register2Value;  	// Stores latest register read values
  
  uint16_t _humidityDelay;	  	// Delay for humidity conversion based on settings
  uint16_t _temperatureDelay;  	// Delay for temperature conversion based on settings
  uint16_t _adcDelay;           // Delay for ADC conversion based on settings
  uint16_t _measurementDelay;  	// Delay duration of present measurement
  float _delayMultiplier;       // Factor to over-estimate the delays
  uint32_t _measureStartTime;  	// timer
  uint8_t _status;              // Current status of the device
  bool _adcNoHold = false;	  	// Keeps track of whether ADC No Hold has been set by user an automates turning it off during humidity and temp readings
  uint8_t _address;             // I2C address of the device
  uint16_t _transactionTimeout;	// Wire NAK/Busy timeout in ms
  
};

#endif
