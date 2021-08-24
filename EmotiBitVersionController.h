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
#include <EmotiBit_Si7013.h>

// Controls which sensor is used OTP access. uncomment the line below is use external SI-7013 connected to the emotibit
//#define USE_ALT_SI7013

// All EmotiBit Pin names should be entered in this class structure.
enum class EmotiBitPinName
{
	EMOTIBIT_I2C_CLOCK = 0,
	EMOTIBTI_I2C_DATA = 1,
	HIBERNATE = 2,
	EMOTIBIT_BUTTON = 3,
	EDL = 4,
	EDR = 5,
	SD_CARD_CHIP_SELECT = 6,
	SPI_CLK = 7,
	SPI_MOSI = 8,
	SPI_MISO = 9,
	PPG_INT = 10,
	BMI_INT1 = 11,
	BMI_INT2 = 12,
	BMM_INT = 13,
	BATTERY_READ_PIN = 14,
	COUNT = 15 //cannot be more than 28 (16 + 12) 
};

enum class MathConstants
{
	VCC = 0,
	ADC_BITS = 1,
	ADC_MAX_VALUE = 2,
	EDR_AMPLIFICATION = 3,
	VREF1 = 4,
	VREF2 = 5,
	EDA_FEEDBACK_R = 6,
	EDA_CROSSOVER_FILTER_FREQ = 7,
	EDA_SERIES_RESISTOR = 8,
	COUNT = 9 // cannot be > than the _MAX_MATH_CONSTANT_COUNT
};

enum class SystemConstants
{
	EMOTIBIT_HIBERNATE_LEVEL = 0,
	LED_DRIVER_CURRENT = 1,
	EMOTIBIT_HIBERNATE_PIN_MODE = 2,
	COUNT = 3 // cannot be greater than the _MAX_SYSTEM_CONSTANT_COUNT
};

class EmotiBitVersionController
{

public:
	// !!! The following ORDER of the enum class holding the Version numbers SHOULD NOT BE ALTERED.
	// New versions shold be ADDED to the end of this list 
	enum class EmotiBitVersion {
		UNKNOWN = -1,
		V01B = 0,
		V01C = 1,
		V02B = 2,
		V02F = 3,
		V02H = 4,
		V03B = 5,
		V04A = 6
	};
	// For detecting EmotiBit Version
public:
#if defined(ADAFRUIT_FEATHER_M0) 
	static const int HIBERNATE_PIN = 6;
	static const int EMOTIBIT_I2C_CLK_PIN = 13;
	static const int EMOTIBIT_I2C_DAT_PIN = 11;
	static const int SD_CARD_CHIP_SEL_PIN = 19;
#endif
private:
	EmotiBitVersion _versionEst;
	int _otpEmotiBitVersion;
	//TwoWire *_EmotiBit_i2c;
	Si7013 _tempHumiditySensor;

public:
	// Important: changing this address will change where the EmotiBit version is stored on the OTP
	static const uint8_t EMOTIBIT_VERSION_ADDR_SI7013_OTP = 0xB7;

	// For Handling Pin Mapping
private:
	static const int _MAX_EMOTIBIT_PIN_COUNT = 28;
	int _assignedPin[_MAX_EMOTIBIT_PIN_COUNT] = { 0 };

	// For Handling Constant Mapping
private:
	static const int _MAX_MATH_CONSTANT_COUNT = 10;
	static const int _MAX_SYSTEM_CONSTANT_COUNT = 10;
private:
	float _assignedMathConstants[_MAX_MATH_CONSTANT_COUNT] = { 0 };
	int _assignedSystemConstants[_MAX_SYSTEM_CONSTANT_COUNT] = { 0 };
	const int _INVALID_CONSTANT_FOR_VERSION = -1;
	const int _INVALID_REQUEST = -2; // addresses out of bounds(for array indexing) request
	bool _initAssignmentComplete = false;
public:
	bool versionDetectionComplete = false;

	// Member functions to control pin mapping
public:
	/*
	* At the time of initialization, assign all the pin numbers to the emotibit pin names
	*/
	bool initPinMapping(EmotiBitVersionController::EmotiBitVersion version);

	int getAssignedPin(EmotiBitPinName pin);

	void echoPinMapping();

	// member functions to control constant mapping
private:
	bool _initMappingMathConstants(EmotiBitVersionController::EmotiBitVersion version);
	bool _initMappingSystemConstants(EmotiBitVersionController::EmotiBitVersion version);
public:
	bool initConstantMapping(EmotiBitVersionController::EmotiBitVersion version);
	float getMathConstant(MathConstants constant);
	int getSystemConstant(SystemConstants constant);
	void echoConstants();
	bool setMathConstantForTesting(MathConstants constant);
	bool setSystemConstantForTesting(SystemConstants constant);

	// member functions to perform version detection
public:
	EmotiBitVersion detectEmotiBitVersion(TwoWire* EmotiBit_i2c);
	bool detectSdCard();
	int readEmotiBitVersionFromSi7013();

public:
	static const char* getHardwareVersion(EmotiBitVersion version);
};
#endif
