#ifndef _EMOTIBIT_VERSION_CONTROLLER_H
#define _EMOTIBIT_VERSION_CONTROLLER_H

#include <SPI.h>
#include <SdFat.h>
#include <Arduino.h>
#include <Wire.h>
#include <EmotiBit_Si7013.h>
#include "EmotiBitVariants.h"
#include "EmotiBitFactoryTest.h"
#include "EmotiBitNvmController.h"

// All EmotiBit Pin names should be entered in this class structure.
enum class EmotiBitPinName
{
	EMOTIBIT_BUTTON = 1,
	SPI_CLK = 2,
	SPI_MOSI = 3,
	SPI_MISO = 4,
	PPG_INT = 5,
	BMI_INT1 = 6,
	BMI_INT2 = 7,
	BMM_INT = 8,
	BATTERY_READ_PIN = 9,
	COUNT = 10 //cannot be more than 28 (16 + 12) 
};

enum class MathConstants
{
	VCC = 0,
	ADC_BITS = 1,
	ADC_MAX_VALUE = 2,
	COUNT = 3 // cannot be > than the _MAX_MATH_CONSTANT_COUNT
};

enum class SystemConstants
{
	EMOTIBIT_HIBERNATE_LEVEL = 0,
	LED_DRIVER_CURRENT = 1,
	EMOTIBIT_HIBERNATE_PIN_MODE = 2,
	COUNT = 3 // cannot be greater than the _MAX_SYSTEM_CONSTANT_COUNT
};

enum class PinActivationLogic
{
	ACTIVE_LOW,
	ACTIVE_HIGH
};

class EmotiBitVersionController
{
public: 	
	// ToDo: move these pin definitions inside a struct.
#if defined(ADAFRUIT_FEATHER_M0) 
	static const int EMOTIBIT_I2C_CLK_PIN = 13;
	static const int EMOTIBIT_I2C_DAT_PIN = 11;
	static int HIBERNATE_PIN;
	static int SD_CARD_CHIP_SEL_PIN;
#endif
	
	// !!! The following ORDER of the enum class holding the Version numbers SHOULD NOT BE ALTERED.
	// New versions should be ADDED to the end of this list 
	enum class EmotiBitVersion {
		UNKNOWN = -1,
		V01B = 0,
		V01C = 1,
		V02B = 2,
		V02F = 3,
		V02H = 4,
		V03B = 5,
		V04A = 6,
		length
	};

	struct EmotiBitHardwareParameterTable {
		bool isSi7013Present;
		bool isEepromPresent;
		bool isThermopilePresent;
	};

private:
	static const int _MAX_EMOTIBIT_PIN_COUNT = 28;
	int _assignedPin[_MAX_EMOTIBIT_PIN_COUNT] = { 0 };
	static const int _MAX_MATH_CONSTANT_COUNT = 3;
	static const int _MAX_SYSTEM_CONSTANT_COUNT = 3;
	float _assignedMathConstants[_MAX_MATH_CONSTANT_COUNT] = { 0 };
	int _assignedSystemConstants[_MAX_SYSTEM_CONSTANT_COUNT] = { 0 };
	const int _INVALID_CONSTANT_FOR_VERSION = -1;
	const int _INVALID_REQUEST = -2; // addresses out of bounds(for array indexing) request
	bool _initAssignmentComplete = false;
	bool _initMappingMathConstants(EmotiBitVersionController::EmotiBitVersion version);
	bool _initMappingSystemConstants(PinActivationLogic logic);
	// ToDo: This needs to be refactored so that the addresses can be read from individuall drivers.
	static const uint8_t SI7013_I2C_ADDR = 0x40;
	static const uint8_t EEPROM_I2C_ADDR = 0x50;
	static const uint8_t MLX90632_I2C_ADDR = 0x3A;


public:
	bool initConstantMapping(EmotiBitVersionController::EmotiBitVersion version);
	float getMathConstant(MathConstants constant);
	int getSystemConstant(SystemConstants constant);
	void echoConstants();
	bool setMathConstantForTesting(MathConstants constant);
	bool setSystemConstantForTesting(SystemConstants constant);
	PinActivationLogic hibernatePinLogic;

	bool isEmotiBitReady();
	bool validateBarcodeInfo(TwoWire &emotibit_i2c, Barcode barcode, bool &hwValidation, bool &skuValidation);
	void updateVersionParameterTable(TwoWire &emotibit_i2c, EmotiBitHardwareParameterTable &hardwareParameterTable);
	bool writeVariantInfoToNvm(TwoWire &emotibit_i2c, EmotiBitNvmController &emotiBitNvmController, Barcode barcode);
	bool getEmotiBitVariantInfo(TwoWire &emotibit_i2c, EmotiBitNvmController &emotiBitNvmController, EmotiBitVersion &hwVersion, String &sku, uint32_t &emotiBitNumber);
	bool detectVariantFromHardware(TwoWire &emotibit_i2c, EmotiBitVersion &hwVersion, String &sku);
	static const char* getHardwareVersion(EmotiBitVersion version);

	bool initPinMapping(EmotiBitVersionController::EmotiBitVersion version);

	int getAssignedPin(EmotiBitPinName pin);

	void echoPinMapping();
};
#endif
