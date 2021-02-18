// the Emotibit Pin mapping class handles mapping of the Feather pin number to the EmotiBit pin name
// Author : Nitin
// date: 17 Feb 2021

#ifndef _EMOTIBIT_PIN_MAPPING_H
#define _EMOTIBIT_PIN_MAPPING_H


class EmotiBitVersionController
{
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

public:
	static const char* getHardwareVersion(EmotiBitVersion version);

};
#endif
