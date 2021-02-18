#include "EmotiBitVersionController.h"
#include "Arduino.h"

const char* EmotiBitVersionController::getHardwareVersion(EmotiBitVersion version)
{
	if (version == EmotiBitVersion::V02H) 
	{
		return "V02h";
	}
	else if (version == EmotiBitVersion::V03B)
	{
		return "V03b";
	}
}

bool EmotiBitVersionController::EmotiBitPinMapping::initMapping(EmotiBitVersionController::EmotiBitVersion version)
{
#if defined(ADAFRUIT_FEATHER_M0)

	_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::BATTERY_READ_PIN] = A7;
	_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SPI_CLK] = 24;
	_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SPI_MOSI] = 23;
	_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SPI_MISO] = 22;

	if (version == EmotiBitVersionController::EmotiBitVersion::V02B || version == EmotiBitVersionController::EmotiBitVersion::V02H || version == EmotiBitVersionController::EmotiBitVersion::V03B)
	{
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::HIBERNATE] = 6;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EMOTIBIT_BUTTON] = 12;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EDL] = A4;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EDR] = A3;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SD_CARD_CHIP_SELECT] = 19;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EMOTIBIT_I2C_CLOCK] = 13;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EMOTIBTI_I2C_DATA] = 11;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::PPG_INT] = 15;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::BMI_INT1] = 5;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::BMI_INT2] = 10;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::BMM_INT] = 14;
	}
	else if (version == EmotiBitVersionController::EmotiBitVersion::V01B || version == EmotiBitVersionController::EmotiBitVersion::V01C)
	{
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::HIBERNATE] = 5;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EMOTIBIT_BUTTON] = 13;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EDL] = A3;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EDR] = A4;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SD_CARD_CHIP_SELECT] = 6;
	}
	else
	{
		// unknown version
		return false;
	}
#elif defined(ADAFRUIT_BLUEFRUIT_NRF52_FEATHER)
	if (version == EmotiBitVersionController::EmotiBitVersion::V01B || version == EmotiBitVersionController::EmotiBitVersion::V01C)
	{
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::HIBERNATE] = 27;//gpio pin assigned ot the mosfet
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EMOTIBIT_BUTTON] = 16;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EDL] = A3; = A3;
		_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EDR] = A4;
	}
	else
	{
		// unknown version
		return false;
	}
#endif
	return true;
}

int EmotiBitVersionController::EmotiBitPinMapping::getAssignedPin(EmotiBitPinMapping::EmotiBitPinName pin)
{
	if ((int)pin >= _MAX_EMOTIBIT_PIN_COUNT)
	{
		Serial.println("out of bounds pin accessed. Please check pin number again");
		return -1;
	}
	else
	{
		return _assignedPin[(int)pin];
	}
}

void EmotiBitVersionController::EmotiBitPinMapping::echoPinMapping()
{
	Serial.print("EMOTIBIT_I2C_CLOCK: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EMOTIBIT_I2C_CLOCK]);
	Serial.print("EMOTIBTI_I2C_DATA: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EMOTIBTI_I2C_DATA]);
	Serial.print("HIBERNATE: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::HIBERNATE]);
	Serial.print("EMOTIBIT_BUTTON: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EMOTIBIT_BUTTON]);
	Serial.print("EDL: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EDL]);
	Serial.print("EDR: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::EDR]);
	Serial.print("SD_CARD_CHIP_SELECT: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SD_CARD_CHIP_SELECT]);
	Serial.print("SPI_CLK: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SPI_CLK]);
	Serial.print("SPI_MOSI: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SPI_MOSI]);
	Serial.print("SPI_MISO: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::SPI_MISO]);
	Serial.print("PPG_INT: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::PPG_INT]);
	Serial.print("BMI_INT1: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::BMI_INT1]);
	Serial.print("BMI_INT2: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::BMI_INT2]);
	Serial.print("BMM_INT: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::BMM_INT]);
	Serial.print("BATTERY_READ_PIN: "); Serial.println(_assignedPin[(int)EmotiBitPinMapping::EmotiBitPinName::BATTERY_READ_PIN]);

}




bool EmotiBitVersionController::EmotiBitConstantsMapping::initMapping(EmotiBitVersionController::EmotiBitVersion version)
{
	if (!_initMappingMathConstants(version))
	{
		return false;
	}
	if (!_initMappingSystemConstants(version))
	{
		return false;
	}
	return true;
}

bool EmotiBitVersionController::EmotiBitConstantsMapping::_initMappingMathConstants(EmotiBitVersionController::EmotiBitVersion version)
{
	if ((int)MathConstants::COUNT > (int)_MAX_MATH_CONSTANT_COUNT)
	{
		// more consatnts that the array has been initialized for
		Serial.println("Out of bounds error for Math constants. please check total constant count with array memory allocation.");
		return false;
	}
#if defined(ADAFRUIT_FEATHER_M0)
	_assignedMathConstants[(int)MathConstants::VCC] = 3.3f;
	_assignedMathConstants[(int)MathConstants::ADC_BITS] = 12;
	_assignedMathConstants[(int)MathConstants::ADC_MAX_VALUE] = pow(2, _assignedMathConstants[(int)MathConstants::ADC_BITS]) - 1;;
	
	if (version == EmotiBitVersionController::EmotiBitVersion::V02H || version == EmotiBitVersionController::EmotiBitVersion::V03B)
	{
		_assignedMathConstants[(int)MathConstants::EDR_AMPLIFICATION] = 100.f / 3.3f;
		_assignedMathConstants[(int)MathConstants::VREF1] = 0.426f; // empirically derived minimum voltage divider value [theoretical 15/(15 + 100)]
		_assignedMathConstants[(int)MathConstants::VREF2] = 1.634591173; // empirically derived average voltage divider value [theoretical _vcc * (100.f / (100.f + 100.f))]
		_assignedMathConstants[(int)MathConstants::EDA_FEEDBACK_R] = 5070000.f; // empirically derived average edaFeedbackAmpR in Ohms (theoretical 4990000.f)
		_assignedMathConstants[(int)MathConstants::EDA_SERIES_RESISTOR] = 0;
		_assignedMathConstants[(int)MathConstants::EDA_CROSSOVER_FILTER_FREQ] = 1.f / (2.f * PI * 200000.f * 0.0000047f);
		
	}
	else if (version == EmotiBitVersionController::EmotiBitVersion::V02B)
	{
		_assignedMathConstants[(int)MathConstants::EDR_AMPLIFICATION] = 100.f / 1.2f;
		_assignedMathConstants[(int)MathConstants::VREF1] = _INVALID_CONSTANT_FOR_VERSION;
		_assignedMathConstants[(int)MathConstants::VREF2] = _INVALID_CONSTANT_FOR_VERSION;
		_assignedMathConstants[(int)MathConstants::EDA_FEEDBACK_R] = 4990000.f;
		_assignedMathConstants[(int)MathConstants::EDA_SERIES_RESISTOR] = _INVALID_CONSTANT_FOR_VERSION;
		_assignedMathConstants[(int)MathConstants::EDA_CROSSOVER_FILTER_FREQ] = _INVALID_CONSTANT_FOR_VERSION;
	}
	_initAssignmentComplete = true;
	return true;
#endif
}

bool EmotiBitVersionController::EmotiBitConstantsMapping::_initMappingSystemConstants(EmotiBitVersionController::EmotiBitVersion version)
{
	if ((int)SystemConstants::COUNT > (int)_MAX_SYSTEM_CONSTANT_COUNT)
	{
		Serial.println("Out of bounds error for System constants. please check total constant count with array memory allocation.");
		return false;
	}
#if defined(ADAFRUIT_FEATHER_M0)
	if (version == EmotiBitVersionController::EmotiBitVersion::V02H)
	{
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL] = HIGH;
		_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT] = 26;
	}
	else if (version == EmotiBitVersionController::EmotiBitVersion::V03B)
	{
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL] = LOW;
		_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT] = 6;
	}
	else if (version == EmotiBitVersionController::EmotiBitVersion::V02B)
	{
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL] = HIGH;
		_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT] = _INVALID_CONSTANT_FOR_VERSION;
	}
	return true;
#endif
}

float EmotiBitVersionController::EmotiBitConstantsMapping::getMathConstant(EmotiBitVersionController::EmotiBitConstantsMapping::MathConstants constant)
{
	if (_initAssignmentComplete)
	{
		if ((int)constant >= _MAX_MATH_CONSTANT_COUNT)
		{
			Serial.println("Invalid request");
			return _INVALID_REQUEST;
		}
		else
		{
			return _assignedMathConstants[(int)constant];
		}
	}
	else
	{
		Serial.println("Constants not initialized yet. call emotiBitVersionController.emotibitConstantMapping.initMapping()");
		return _INVALID_REQUEST;
	}
}

int EmotiBitVersionController::EmotiBitConstantsMapping::getSystemConstant(EmotiBitVersionController::EmotiBitConstantsMapping::SystemConstants constant)
{
	if (_initAssignmentComplete)
	{
		if ((int)constant >= _MAX_SYSTEM_CONSTANT_COUNT)
		{
			Serial.println("Invalid request");
			return _INVALID_REQUEST;
		}
		else
		{
			return _assignedSystemConstants[(int)constant];
		}
	}
	else
	{
		Serial.println("Constants not initialized yet.  call emotiBitVersionController.emotibitConstantMapping.initMapping()");
		return _INVALID_REQUEST;
	}
}

void EmotiBitVersionController::EmotiBitConstantsMapping::echoConstants()
{
	if (_initAssignmentComplete)
	{
		Serial.print("MathConstants: VCC - "); Serial.println(_assignedMathConstants[(int)MathConstants::VCC]);
		Serial.print("MathConstants: ADC_BITS - "); Serial.println(_assignedMathConstants[(int)MathConstants::ADC_BITS]);
		Serial.print("MathConstants: ADC_MAX_VALUE - "); Serial.println(_assignedMathConstants[(int)MathConstants::ADC_MAX_VALUE]);
		Serial.print("MathConstants: EDR_AMPLIFICATION - "); Serial.println(_assignedMathConstants[(int)MathConstants::EDR_AMPLIFICATION]);
		Serial.print("MathConstants: VREF1 - "); Serial.println(_assignedMathConstants[(int)MathConstants::VREF1]);
		Serial.print("MathConstants: VREF2 - "); Serial.println(_assignedMathConstants[(int)MathConstants::VREF2]);
		Serial.print("MathConstants: EDA_FEEDBACK_R - "); Serial.println(_assignedMathConstants[(int)MathConstants::EDA_FEEDBACK_R]);
		Serial.print("MathConstants: EDA_CROSSOVER_FILTER_FREQ - "); Serial.println(_assignedMathConstants[(int)MathConstants::EDA_CROSSOVER_FILTER_FREQ]);
		Serial.print("MathConstants: EDA_SERIES_RESISTOR - "); Serial.println(_assignedMathConstants[(int)MathConstants::EDA_SERIES_RESISTOR]);
		Serial.print("SystemConstant: EMOTIBIT_HIBERNATE_LEVEL - "); Serial.println(_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL]);
		Serial.print("SystemConstant: LED_DRIVER_CURRENT - "); Serial.println(_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT]);
	}
	else
	{
		Serial.println("Please initilize the constants.  call emotiBitVersionController.emotibitConstantMapping.initMapping()");
	}
}

bool EmotiBitVersionController::EmotiBitConstantsMapping::setMathConstantForTesting(EmotiBitVersionController::EmotiBitConstantsMapping::MathConstants constant)
{
	//ToDo: write the function to assign test values to the constants 
}

bool EmotiBitVersionController::EmotiBitConstantsMapping::setSystemConstantForTesting(EmotiBitVersionController::EmotiBitConstantsMapping::SystemConstants constant)
{
	//ToDo: write the function to assign test values to the constants 
}