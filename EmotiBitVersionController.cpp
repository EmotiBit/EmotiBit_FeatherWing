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
	else if (version == EmotiBitVersion::V01B)
	{
		return "V01b";
	}
	else if (version == EmotiBitVersion::V01C)
	{
		return "V01c";
	}
	else if (version == EmotiBitVersion::V02B)
	{
		return "V02b";
	}
	else if (version == EmotiBitVersion::V02F)
	{
		return "V02f";
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

EmotiBitVersionController::EmotiBitVersionDetection::EmotiBitVersionDetection(TwoWire* emotiBit_I2c)
{
	_emotiBit_i2c = emotiBit_I2c;
	//_tempHumiditySensor = tempHumiditySensor;
	//_SD = SD;
	//_emotiBitWiFi = emotiBitWiFi;
	//_edaCorrection = edaCorrection;
	_versionEst = -1;
	_otpEmotiBitVersion = -1;
	_hibernatePin = 6;
	_emotiBitI2cClkPin = 13;
	_emotiBitI2cDataPin = 11;
	_sdCardChipSelectPin = 19;
	//_isConfigFilePresent = false;
	//_si7013ChipBegun = si7013ChipBegun;
}

int EmotiBitVersionController::EmotiBitVersionDetection::begin()
{
	//uint8_t versionEst = 0;
	//int otpEmotiBitVersion = 0;
	// V02B, V02H and V03B all have pin 6 as hibernate, and this code supports only those versions
	//int hibernatePin = 6;
	//int emotiBitI2cClkPin = 13;
	//int _sdCardChipSelectPin = 19; 
	pinMode(_hibernatePin, OUTPUT);
	bool status;
	//bool isConfigFilePresent = true;
	Serial.println("****************************** DETECTING EMOTIBIT VERSION ************************************");
	Serial.println("Making hibernate LOW");
	digitalWrite(_hibernatePin, LOW);
	delay(100);
	// Try Setting up SD Card
	status = detectSdCard();
	delay(200);
	if (status)
	{
		// SD-CArd detected
		_versionEst = (int)EmotiBitVersion::V02H;
		/*
		if (_isConfigFilePresent)
		{
			_versionEst = (int)EmotiBitVersion::V02H;
		}
		else
		{
			Serial.println("Disabling EmotiBit Power");
			digitalWrite(_hibernatePin, HIGH);// disables emotibit power supply
		}
		*/
	}
	else
	{
		// Sd-Card not detected in V2 startup
		Serial.println("SD-Card not detected with V2 power up Sequence.\nTrying V3 power-up sequence");
		Serial.println("Making hibernate HIGH");
		digitalWrite(_hibernatePin, HIGH);
		delay(100);
		// Try Setting up SD Card
		status = detectSdCard();
		delay(200);
		if (status)
		{
			_versionEst = (int)EmotiBitVersion::V03B;
			/*
			if (_isConfigFilePresent)
			{
				_versionEst = (int)EmotiBitVersion::V03B;
			}
			else
			{
				Serial.println("Disabling EmotiBit Power");
				digitalWrite(_hibernatePin, LOW);// disables emotibit power supply
			}
			*/
		}
		else
		{
			Serial.println("initialization failed. Things to check:");
			Serial.println("* is a card inserted?");
			Serial.println("* is your wiring correct?");
			Serial.println("Version not detected. stopping execution.");
			pinMode(_emotiBitI2cClkPin, OUTPUT);
			digitalWrite(_emotiBitI2cClkPin, LOW);
			while (true);
		}
	}
	//if (!status || !_isConfigFilePresent)
	//{
	//	pinMode(_emotiBitI2cClkPin, OUTPUT);
	//	digitalWrite(_emotiBitI2cClkPin, LOW);
	//	while (true);
	//}
	Serial.print("Estimated version of the emotibit is:"); Serial.println(EmotiBitVersionController::getHardwareVersion((EmotiBitVersionController::EmotiBitVersion)_versionEst));
	Serial.println();
	Serial.print("Powering emotibit according to the estimate. ");
	if (_versionEst == (int)EmotiBitVersionController::EmotiBitVersion::V02H)
	{
		digitalWrite(_hibernatePin, LOW);
		Serial.println("made hibernate LOW");
	}
	else if (_versionEst == (int)EmotiBitVersionController::EmotiBitVersion::V03B)
	{
		digitalWrite(_hibernatePin, HIGH);
		Serial.println("made hibernate HIGH");
	}
	// Flush the I2C
	Serial.print("Setting up I2C....");
	_emotiBit_i2c->begin();
	uint32_t i2cRate = 100000;
	Serial.print("setting clock to");
	Serial.print(i2cRate);
	_emotiBit_i2c->setClock(i2cRate);
	Serial.print("...setting PIO_SERCOM");
	pinPeripheral(_emotiBitI2cDataPin, PIO_SERCOM);
	pinPeripheral(_emotiBitI2cClkPin, PIO_SERCOM);
	Serial.print("...flushing");
	_emotiBit_i2c->flush();

	status = true;
	// Setup Temperature / Humidity Sensor
	Serial.println("\n\nConfiguring Temperature / Humidity Sensor");
	// moved the macro definition from EdaCorrection to EmotiBitVersionController
#ifdef USE_ALT_SI7013 
	status = _tempHumiditySensor.setup(*_emotiBit_i2c, 0x41);
#else
	status = _tempHumiditySensor.setup(*_emotiBit_i2c);
#endif
	if (status)
	{
		// Si-7013 detected on the EmotiBit
		while (_tempHumiditySensor.getStatus() != Si7013::STATUS_IDLE);
		_otpEmotiBitVersion = readEmotiBitVersionFromSi7013();

		//_tempHumiditySensor->changeSetting(Si7013::Settings::RESOLUTION_H11_T11);
		//_tempHumiditySensor->changeSetting(Si7013::Settings::ADC_NORMAL);
		//_tempHumiditySensor->changeSetting(Si7013::Settings::VIN_UNBUFFERED);
		//_tempHumiditySensor->changeSetting(Si7013::Settings::VREFP_VDDA);
		//_tempHumiditySensor->changeSetting(Si7013::Settings::ADC_NO_HOLD);

		//_tempHumiditySensor->readSerialNumber();
		//Serial.print("Si7013 Electronic Serial Number: ");
		//Serial.print(_tempHumiditySensor->sernum_a);
		//Serial.print(", ");
		//Serial.print(_tempHumiditySensor->sernum_b);
		//Serial.print("\n");
		//Serial.print("Model: ");
		//Serial.println(_tempHumiditySensor->_model);
		////chipBegun.SI7013 = true;
		//*_si7013ChipBegun = true;

		//_tempHumiditySensor->startHumidityTempMeasurement();
	}
	else
	{
		// Si-7013 Not detected
		// Killing EmotiBit Power
		//hibernate();
		if (_versionEst == (int)EmotiBitVersionController::EmotiBitVersion::V02H)
		{
			digitalWrite(_hibernatePin, HIGH);
			Serial.println("made hibernate HIGH");
		}
		else if (_versionEst == (int)EmotiBitVersionController::EmotiBitVersion::V03B)
		{
			digitalWrite(_hibernatePin, LOW);
			Serial.println("made hibernate LOW");
		}
	}

	//_emotiBit_i2c->setClock(400000);// setting the rate back to 400K for normal I2C operation
	if (_otpEmotiBitVersion == 255)
	{
		Serial.println("OTP has not yet been updated");
		Serial.print("using the Estimated emotibit version detected from power up sequence: "); Serial.println(EmotiBitVersionController::getHardwareVersion((EmotiBitVersionController::EmotiBitVersion)_versionEst));
		Serial.println("************************** END DETECTING EMOTIBIT VERSION ************************************");
		return _versionEst;
	}
	else if (_otpEmotiBitVersion == -1)// Sensor not detected on the I2C
	{
		Serial.println("Stopping code execution");
		while (true);
	}
	else
	{
		if (_otpEmotiBitVersion == _versionEst)
		{
			Serial.println("######################");
			Serial.print("The EmotiBit Version read from OTP is: ");
			Serial.println(EmotiBitVersionController::getHardwareVersion((EmotiBitVersionController::EmotiBitVersion)_otpEmotiBitVersion));
			//Serial.println(emotibitVersion);
			Serial.println("######################");
			Serial.println("************************** END DETECTING EMOTIBIT VERSION ************************************");
			return _otpEmotiBitVersion;
		}
		else
		{
			// Version on EmotiBit not equal to estimated version
			// ToDO: Resolve conflict. 
		}
	}
}


bool EmotiBitVersionController::EmotiBitVersionDetection::detectSdCard()
{
	SdFat sd;
	Serial.print("\nInitializing SD card...");

	// coed snippet taken from CardInfo exmaple from the SD library in Arduino
	// we'll use the initialization code from the utility libraries
	// since we're just testing if the card is working!
	if (!sd.cardBegin(_sdCardChipSelectPin, SD_SCK_MHZ(50)))
	{
		return false;
	}
	else 
	{
		Serial.println("Wiring is correct and a card is present.");
		return true;
	}
}
/*
bool EmotiBitVersionController::EmotiBitVersionDetection::loadConfigFile()
{
	// Open file for reading
	File file = _SD->open(_configFileName);

	if (!file) {
		Serial.print("File ");
		Serial.print(_configFileName);
		Serial.println(" not found");
		return false;
	}

	Serial.print("Parsing: ");
	Serial.println(_configFileName);

	// Allocate the memory pool on the stack.
	// Don't forget to change the capacity to match your JSON document.
	// Use arduinojson.org/assistant to compute the capacity.
	//StaticJsonBuffer<1024> jsonBuffer;
	StaticJsonBuffer<1024> jsonBuffer;

	// Parse the root object
	JsonObject &root = jsonBuffer.parseObject(file);

	if (!root.success()) {
		Serial.println(F("Failed to parse config file"));
		return false;
	}

	size_t configSize;
	// Copy values from the JsonObject to the Config
	configSize = root.get<JsonVariant>("WifiCredentials").as<JsonArray>().size();
	Serial.print("ConfigSize: ");
	Serial.println(configSize);
	for (size_t i = 0; i < configSize; i++) {
		String ssid = root["WifiCredentials"][i]["ssid"] | "";
		String pass = root["WifiCredentials"][i]["password"] | "";
		Serial.print("Adding SSID: ");
		Serial.println(ssid);
		_emotiBitWiFi->addCredential(ssid, pass);
		Serial.println(ssid);
		Serial.println(pass);
	}

	//strlcpy(config.hostname,                   // <- destination
	//	root["hostname"] | "example.com",  // <- source
	//	sizeof(config.hostname));          // <- destination's capacity

	// Close the file (File's destructor doesn't close the file)
	// ToDo: Handle multiple credentials

	file.close();
	return true;
}
*/
int EmotiBitVersionController::EmotiBitVersionDetection::readEmotiBitVersionFromSi7013()
{
	if (!_tempHumiditySensor.sendCommand(0x00)) // returns false if failed to send command
	{
		// Si7013 not found on EmotiBit
		Serial.println("Si-7013 not found on EmotiBit. Check I2C wiring.");
		return -1;
	}
	else
	{
		// Sensor found
		uint8_t emotibitVersion = 0;
		uint8_t emotiBitVersionAddr = EmotiBitVersionController::EMOTIBIT_VERSION_ADDR_SI7013_OTP;
		emotibitVersion = (uint8_t)_tempHumiditySensor.readRegister8(emotiBitVersionAddr, true);

		return emotibitVersion;
	}
}