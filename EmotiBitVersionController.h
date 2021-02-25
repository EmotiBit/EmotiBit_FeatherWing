// the Emotibit Version Controller class handles all things related to EmotiBit Versions. This includes
// 1. Detecting EmotiBit Version on startup
// 2. Loading feather-emotibit pin mappings
// 3. Loading emotibit specific constants
// Author : Nitin
// date: 17 Feb 2021

#ifndef _EMOTIBIT_VERSION_CONTROLLER_H
#define _EMOTIBIT_VERSION_CONTROLLER_H

#include <SPI.h>
#include <SdFat.h>
#include <Arduino.h>
#include <Wire.h>
#include <SdFat.h>
#include <EmotiBit_Si7013.h>
#include "wiring_private.h"
#include <ArduinoJson.h>
#include "EmotiBitWiFi.h"
//#include "EdaCorrection.h"

// Controls which sensor is used OTP access. uncomment the line below is use external SI-7013 connected to teh emotibit
//#define USE_ALT_SI7013

class EmotiBitVersionController
{
private:
	const char *_configFilename = "config.txt";
public:
	// Important: changing this address will change where the EmotiBit version is stored on the OTP
	static const uint8_t EMOTIBIT_VERSION_ADDR_SI7013_OTP = 0xB7;
public:
	// !!! The following ORDER of the enum class holding the Version numbers SHOULD NOT BE ALTERED.
	// New versions shold be ADDED to the end of this list 
	enum class EmotiBitVersion {
		V01B = 0,
		V01C = 1,
		V02B = 2,
		V02F = 3,
		V02H = 4,
		V03B = 5
	};

	class EmotiBitPinMapping
	{
	public:
		enum class EmotiBitPinName
		{
			EMOTIBIT_I2C_CLOCK,
			EMOTIBTI_I2C_DATA,
			HIBERNATE,
			EMOTIBIT_BUTTON,
			EDL,
			EDR,
			SD_CARD_CHIP_SELECT,
			SPI_CLK,
			SPI_MOSI,
			SPI_MISO,
			PPG_INT,
			BMI_INT1,
			BMI_INT2,
			BMM_INT,
			BATTERY_READ_PIN,
			COUNT //cannot be more than 28 (16 + 12) 
		};

	private:
		static const int _MAX_EMOTIBIT_PIN_COUNT = 28;
		int _assignedPin[_MAX_EMOTIBIT_PIN_COUNT] = { 0 };
		//const char *emotiBitPinNameString[28];

	public:
		/*
		* At the time of initialization, assign all the pin numbers to the emotibit pin names
		*/
		bool initMapping(EmotiBitVersionController::EmotiBitVersion version);

		int getAssignedPin(EmotiBitPinMapping::EmotiBitPinName pin);

		void echoPinMapping();

	}emotiBitPinMapping;

	class EmotiBitConstantsMapping
	{
	public:
		enum class MathConstants
		{
			VCC,
			ADC_BITS,
			ADC_MAX_VALUE,
			EDR_AMPLIFICATION,
			VREF1,
			VREF2,
			EDA_FEEDBACK_R,
			EDA_CROSSOVER_FILTER_FREQ,
			EDA_SERIES_RESISTOR,
			COUNT // cannot be > than the _MAX_MATH_CONSTANT_COUNT
		};

		enum class SystemConstants
		{
			EMOTIBIT_HIBERNATE_LEVEL,
			LED_DRIVER_CURRENT,
			COUNT // cannot be greater than the _MAX_SYSTEM_CONSTANT_COUNT
		};
	private:
		static const int _MAX_MATH_CONSTANT_COUNT = 10;
		static const int _MAX_SYSTEM_CONSTANT_COUNT = 10;

	private:
		float _assignedMathConstants[_MAX_MATH_CONSTANT_COUNT] = { 0 };
		int _assignedSystemConstants[_MAX_SYSTEM_CONSTANT_COUNT] = { 0 };
		const int _INVALID_CONSTANT_FOR_VERSION = -1;
		const int _INVALID_REQUEST = -2; // addresses out of bounds(for array indexing) request
		bool _initAssignmentComplete = false;
	private:
		bool _initMappingMathConstants(EmotiBitVersionController::EmotiBitVersion version);
		bool _initMappingSystemConstants(EmotiBitVersionController::EmotiBitVersion version);
	public:
		bool initMapping(EmotiBitVersionController::EmotiBitVersion version);
		float getMathConstant(MathConstants constant);
		int getSystemConstant(SystemConstants constant);
		void echoConstants();
		bool setMathConstantForTesting(MathConstants constant);
		bool setSystemConstantForTesting(SystemConstants constant);
	}emotiBitConstantsMapping;

	class EmotiBitVersionDetection
	{
	private:
		int _versionEst;
		int _otpEmotiBitVersion;
		int _hibernatePin;
		int _emotiBitI2cClkPin;
		int _emotiBitI2cDataPin;
		int _sdCardChipSelectPin;
		//const char *_configFileName = "config.txt";
		//bool _isConfigFilePresent;
		//bool *_si7013ChipBegun;
	private:
		TwoWire *_EmotiBit_i2c;
		Si7013 _tempHumiditySensor;
		//SdFat *_SD;
		//EmotiBitWiFi *_emotiBitWiFi;
		//EmotiBitPinMapping *_emotiBitPinMapping;
		//EmotiBitConstantsMapping *_emotiBitConstantsMapping;
		//EdaCorrection *_edaCorrection;

	public:
		EmotiBitVersionDetection(TwoWire* EmotiBit_I2c);
		int begin();
		bool detectSdCard();
		//bool loadConfigFile();
		int readEmotiBitVersionFromSi7013();

	};

public:
	static const char* getHardwareVersion(EmotiBitVersion version);
};
#endif
