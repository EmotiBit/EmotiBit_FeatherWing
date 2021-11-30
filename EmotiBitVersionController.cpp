#include "EmotiBitVersionController.h"
#include "Arduino.h"

const char* EmotiBitVersionController::getHardwareVersion(EmotiBitVersion version)
{

	if (version == EmotiBitVersion::V01B)
	{
		return "V01b\0";
	}
	else if (version == EmotiBitVersion::V01C)
	{
		return "V01c\0";
	}
	else if (version == EmotiBitVersion::V02B)
	{
		return "V02b\0";
	}
	else if (version == EmotiBitVersion::V02F)
	{
		return "V02f\0";
	}
	else if (version == EmotiBitVersion::V02H)
	{
		return "V02h\0";
	}
	else if (version == EmotiBitVersion::V03B)
	{
		return "V03b\0";
	}
	else if (version == EmotiBitVersion::V04A)
	{
		return "V04a\0";
	}
}

bool EmotiBitVersionController::initPinMapping(EmotiBitVersionController::EmotiBitVersion version)
{
#if defined(ADAFRUIT_FEATHER_M0)
	// ToDo: Move these pin Assignments(maybe inside a constructor). These are specific to MCU platform and cannot change.
	_assignedPin[(int)EmotiBitPinName::BATTERY_READ_PIN] = A7;
	_assignedPin[(int)EmotiBitPinName::SPI_CLK] = 24;
	_assignedPin[(int)EmotiBitPinName::SPI_MOSI] = 23;
	_assignedPin[(int)EmotiBitPinName::SPI_MISO] = 22;

	if (version == EmotiBitVersion::V02B || version == EmotiBitVersion::V02H || version == EmotiBitVersion::V03B || version == EmotiBitVersion::V04A)
	{
		//_assignedPin[(int)EmotiBitPinName::HIBERNATE] = 6;
		_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON] = 12;
		// Dont assign EDR or EDL pins for V4
		/*
		if (version != EmotiBitVersion::V04A)
		{
			_assignedPin[(int)EmotiBitPinName::EDL] = A4;
			_assignedPin[(int)EmotiBitPinName::EDR] = A3;
		}
		*/
		//_assignedPin[(int)EmotiBitPinName::SD_CARD_CHIP_SELECT] = 19;
		//_assignedPin[(int)EmotiBitPinName::EMOTIBIT_I2C_CLOCK] = 13;
		//_assignedPin[(int)EmotiBitPinName::EMOTIBTI_I2C_DATA] = 11;
		_assignedPin[(int)EmotiBitPinName::PPG_INT] = 15;
		_assignedPin[(int)EmotiBitPinName::BMI_INT1] = 5;
		_assignedPin[(int)EmotiBitPinName::BMI_INT2] = 10;
		_assignedPin[(int)EmotiBitPinName::BMM_INT] = 14;
	}
	else if (version == EmotiBitVersion::V01B || version == EmotiBitVersion::V01C)
	{
		//_assignedPin[(int)EmotiBitPinName::HIBERNATE] = 5;
		_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON] = 13;
		//_assignedPin[(int)EmotiBitPinName::EDL] = A3;
		//_assignedPin[(int)EmotiBitPinName::EDR] = A4;
		//_assignedPin[(int)EmotiBitPinName::SD_CARD_CHIP_SELECT] = 6;
	}
	else
	{
		// unknown version
		Serial.println("Unknown Version");
		return false;
	}
#elif defined(ADAFRUIT_BLUEFRUIT_NRF52_FEATHER)
	if (version == EmotiBitVersion::V01B || version == EmotiBitVersion::V01C)
	{
		//_assignedPin[(int)EmotiBitPinName::HIBERNATE] = 27;//gpio pin assigned ot the mosfet
		_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON] = 16;
		//_assignedPin[(int)EmotiBitPinName::EDL] = A3; = A3;
		//_assignedPin[(int)EmotiBitPinName::EDR] = A4;
	}
	else
	{
		// unknown version
		Serial.println("Unknown Version");
		return false;
	}
#endif
	return true;
}

int EmotiBitVersionController::getAssignedPin(EmotiBitPinName pin)
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


void EmotiBitVersionController::echoPinMapping()
{
	Serial.print("EMOTIBIT_I2C_CLOCK: "); Serial.println(EMOTIBIT_I2C_CLK_PIN);
	Serial.print("EMOTIBTI_I2C_DATA: "); Serial.println(EMOTIBIT_I2C_DAT_PIN);
	Serial.print("HIBERNATE: "); Serial.println(HIBERNATE_PIN);
	Serial.print("EMOTIBIT_BUTTON: "); Serial.println(_assignedPin[(int)EmotiBitPinName::EMOTIBIT_BUTTON]);
	//Serial.print("EDL: "); Serial.println(_assignedPin[(int)EmotiBitPinName::EDL]);
	//Serial.print("EDR: "); Serial.println(_assignedPin[(int)EmotiBitPinName::EDR]);
	Serial.print("SD_CARD_CHIP_SELECT: "); Serial.println(SD_CARD_CHIP_SEL_PIN);
	Serial.print("SPI_CLK: "); Serial.println(_assignedPin[(int)EmotiBitPinName::SPI_CLK]);
	Serial.print("SPI_MOSI: "); Serial.println(_assignedPin[(int)EmotiBitPinName::SPI_MOSI]);
	Serial.print("SPI_MISO: "); Serial.println(_assignedPin[(int)EmotiBitPinName::SPI_MISO]);
	Serial.print("PPG_INT: "); Serial.println(_assignedPin[(int)EmotiBitPinName::PPG_INT]);
	Serial.print("BMI_INT1: "); Serial.println(_assignedPin[(int)EmotiBitPinName::BMI_INT1]);
	Serial.print("BMI_INT2: "); Serial.println(_assignedPin[(int)EmotiBitPinName::BMI_INT2]);
	Serial.print("BMM_INT: "); Serial.println(_assignedPin[(int)EmotiBitPinName::BMM_INT]);
	Serial.print("BATTERY_READ_PIN: "); Serial.println(_assignedPin[(int)EmotiBitPinName::BATTERY_READ_PIN]);

}




bool EmotiBitVersionController::initConstantMapping(EmotiBitVersionController::EmotiBitVersion version)
{
	if (!_initMappingMathConstants(version))
	{
		return false;
	}
	/*
	if (!_initMappingSystemConstants(version))
	{
		return false;
	}
	*/
	_initAssignmentComplete = true;
	return true;
}

bool EmotiBitVersionController::_initMappingMathConstants(EmotiBitVersionController::EmotiBitVersion version)
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
	/*
	if (version == EmotiBitVersion::V04A)
	{
		_assignedMathConstants[(int)MathConstants::EDA_CROSSOVER_FILTER_FREQ] = 1.f / (2.f * PI * 200000.f * 0.0000047f);
	}
	else if (version == EmotiBitVersion::V02H || version == EmotiBitVersion::V03B)
	{
		_assignedMathConstants[(int)MathConstants::EDR_AMPLIFICATION] = 100.f / 3.3f;
		// The Vref1 value being used is the empirical mean of Vref1 measured in emotibits during testing.
		_assignedMathConstants[(int)MathConstants::VREF1] = 0.426f; // empirically derived minimum voltage divider value [theoretical 15/(15 + 100)]. 
		// The Vref2 value being used is the empirical mean of Vref2 measured in emotibits during testing.
		_assignedMathConstants[(int)MathConstants::VREF2] = 1.634591173; // empirically derived average voltage divider value [theoretical _vcc * (100.f / (100.f + 100.f))]
		_assignedMathConstants[(int)MathConstants::EDA_FEEDBACK_R] = 5070000.f; // empirically derived average edaFeedbackAmpR in Ohms (theoretical 4990000.f)
		_assignedMathConstants[(int)MathConstants::EDA_SERIES_RESISTOR] = 0;
		_assignedMathConstants[(int)MathConstants::EDA_CROSSOVER_FILTER_FREQ] = 1.f / (2.f * PI * 200000.f * 0.0000047f);
		
	}
	*/
	return true;
#endif
}

bool EmotiBitVersionController::_initMappingSystemConstants(PinActivationLogic logic)
{
	if ((int)SystemConstants::COUNT > (int)_MAX_SYSTEM_CONSTANT_COUNT)
	{
		Serial.println("Out of bounds error for System constants. please check total constant count with array memory allocation.");
		return false;
	}
#if defined(ADAFRUIT_FEATHER_M0)
	if (logic == PinActivationLogic::ACTIVE_HIGH)
	{
		// For V2
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL] = HIGH;
		_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT] = 26;
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_PIN_MODE] = INPUT;
	}
	else 
	{
		// For V3 and bove
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL] = LOW;
		_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT] = 6;
		_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_PIN_MODE] = INPUT_PULLDOWN;
	}

	return true;
#endif
}

float EmotiBitVersionController::getMathConstant(MathConstants constant)
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

int EmotiBitVersionController::getSystemConstant(SystemConstants constant)
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

void EmotiBitVersionController::echoConstants()
{
	if (_initAssignmentComplete)
	{
		Serial.print("MathConstants: VCC - "); Serial.println(_assignedMathConstants[(int)MathConstants::VCC]);
		Serial.print("MathConstants: ADC_BITS - "); Serial.println(_assignedMathConstants[(int)MathConstants::ADC_BITS]);
		Serial.print("MathConstants: ADC_MAX_VALUE - "); Serial.println(_assignedMathConstants[(int)MathConstants::ADC_MAX_VALUE]);
		//Serial.print("MathConstants: EDR_AMPLIFICATION - "); Serial.println(_assignedMathConstants[(int)MathConstants::EDR_AMPLIFICATION]);
		//Serial.print("MathConstants: VREF1 - "); Serial.println(_assignedMathConstants[(int)MathConstants::VREF1]);
		//Serial.print("MathConstants: VREF2 - "); Serial.println(_assignedMathConstants[(int)MathConstants::VREF2]);
		//Serial.print("MathConstants: EDA_FEEDBACK_R - "); Serial.println(_assignedMathConstants[(int)MathConstants::EDA_FEEDBACK_R]);
		//Serial.print("MathConstants: EDA_CROSSOVER_FILTER_FREQ - "); Serial.println(_assignedMathConstants[(int)MathConstants::EDA_CROSSOVER_FILTER_FREQ]);
		//Serial.print("MathConstants: EDA_SERIES_RESISTOR - "); Serial.println(_assignedMathConstants[(int)MathConstants::EDA_SERIES_RESISTOR]);
		Serial.print("SystemConstant: EMOTIBIT_HIBERNATE_LEVEL - "); Serial.println(_assignedSystemConstants[(int)SystemConstants::EMOTIBIT_HIBERNATE_LEVEL]);
		Serial.print("SystemConstant: LED_DRIVER_CURRENT - "); Serial.println(_assignedSystemConstants[(int)SystemConstants::LED_DRIVER_CURRENT]);
	}
	else
	{
		Serial.println("Please initilize the constants.  call emotiBitVersionController.emotibitConstantMapping.initMapping()");
	}
}

bool EmotiBitVersionController::setMathConstantForTesting(MathConstants constant)
{
	//ToDo: write the function to assign test values to the constants 
}

bool EmotiBitVersionController::setSystemConstantForTesting(SystemConstants constant)
{
	//ToDo: write the function to assign test values to the constants 
}

bool EmotiBitVersionController::isEmotiBitReady()
{
	SdFat SD;
	pinMode(HIBERNATE_PIN, INPUT);

	if (digitalRead(HIBERNATE_PIN) == LOW)
	{
		// V3+: setting hibernate pin low hibernates EmotiBit
		Serial.println("Hibernate Logic: Active LOW(V3+)");
		hibernatePinLogic = PinActivationLogic::ACTIVE_LOW;
		pinMode(HIBERNATE_PIN, OUTPUT);
		digitalWrite(HIBERNATE_PIN, HIGH);
		delay(100);
		// ToDo: Think about adding battery voltage measurement here as well
		_initMappingSystemConstants(hibernatePinLogic);
		if (SD.begin(SD_CARD_CHIP_SEL_PIN))
		{
			return true;
		}
	}
	else
	{
		// V2: setting hibernate pin low hibernates EmotiBit
		Serial.println("Hibernate Logic: Active HIGH(V2)");
		hibernatePinLogic = PinActivationLogic::ACTIVE_HIGH;
		pinMode(HIBERNATE_PIN, OUTPUT);
		digitalWrite(HIBERNATE_PIN, LOW);
		delay(100);
		// ToDo: Think about adding battery voltage measurement here as well
		_initMappingSystemConstants(hibernatePinLogic);
		if (SD.begin(SD_CARD_CHIP_SEL_PIN))
		{
			return true;
		}
	}
	Serial.println("EmotiBit not ready. Please check if Battery and SD-Card are present on the EmotiBit.");
	return false;
}

bool EmotiBitVersionController::validateBarcodeInfo(TwoWire &emotibit_i2c, Barcode barcode, bool &hwValidation, bool &skuValidation)
{
	EmotiBitHardwareParameterTable hardwareParameterTable;
	updateVersionParameterTable(emotibit_i2c, hardwareParameterTable);

	if (hibernatePinLogic == PinActivationLogic::ACTIVE_HIGH)
	{
		Serial.println("Hardware version detected as V2. No Write operations allowed for this HW version.");
		hwValidation = false;
		return false;
	}
	else
	{
		if (hardwareParameterTable.isSi7013Present && !hardwareParameterTable.isEepromPresent)
		{
			Serial.println("Hardware version detected as V3. No Write operations allowed for this HW version.");
			hwValidation = false;
			return false;
		}
		else if (hardwareParameterTable.isEepromPresent && !hardwareParameterTable.isSi7013Present)
		{
			String barcodeHwVersion = barcode.hwVersion;
			barcodeHwVersion.remove(barcodeHwVersion.indexOf(EmotiBitVariants::HARDWARE_VERSION_PREFIX), 1);
			if (barcodeHwVersion.toInt() >= 4)
			{
				Serial.println("HW validation: Passed");
				hwValidation = true;
			}
			else
			{
				Serial.println("HW validation: Failed");
				hwValidation = false;
			}
			if (hardwareParameterTable.isThermopilePresent)
			{
				if (barcode.sku.equals(EmotiBitVariants::EMOTIBIT_SKU_MD))
				{
					Serial.println("SKU validation: Passed");
					skuValidation = true;
					return true;
				}
				else
				{
					skuValidation = false;
					Serial.println("SKU validation Failed. Hardware SKU detected: MD");
					return false;
				}
			}
			else
			{
				if (barcode.sku.equals(EmotiBitVariants::EMOTIBIT_SKU_EM))
				{
					Serial.println("SKU validation: Passed");
					skuValidation = true;
					return true;
				}
				else
				{
					skuValidation = false;
					Serial.println("SKU validation Failed. Hardware SKU detected: EM");
					return false;
				}
			}
		}
	}
}

bool EmotiBitVersionController::writeVariantInfoToNvm(TwoWire &emotibit_i2c, EmotiBitNvmController &emotiBitNvmController, Barcode barcode)
{
	EmotiBitVariantInfo emotiBitVariantInfo;
	EmotiBitFactoryTest::convertBarcodeToVariantInfo(barcode, emotiBitVariantInfo);
	Serial.println("Data being written to NVM");
	printEmotiBitVariantInfo(emotiBitVariantInfo);

	uint8_t* data;
	EmotiBitVariantInfo* variantInfo = &emotiBitVariantInfo;
	data = (uint8_t*)variantInfo;
	uint8_t status;
	status = emotiBitNvmController.stageToWrite(EmotiBitNvmController::DataType::VARIANT_INFO, (uint8_t)EmotiBitVariantDataFormat::V1, sizeof(EmotiBitVariantInfo), data, true);
	if (status == 0)
	{
		Serial.println("Variant Information written into the NVM.");
		return true;
	}
	else
	{
		Serial.print("Error writing Variant Info. ErrorCode: "); Serial.println(status);
		return false;
	}
}

bool EmotiBitVersionController::getEmotiBitVariantInfo(TwoWire &emotibit_i2c, EmotiBitNvmController &emotiBitNvmController, EmotiBitVersion &hwVersion, String &sku, uint32_t &emotiBitNumber)
{
	uint8_t* nvmData;
	uint8_t datatypeVersion;
	uint32_t dataSize;
	uint8_t status;
	status = emotiBitNvmController.stageToRead(EmotiBitNvmController::DataType::VARIANT_INFO, datatypeVersion, dataSize, nvmData, true);
	if (status == 0)
	{
		Serial.println("Successfully read variant info from NVM");
		if (datatypeVersion == (uint8_t)EmotiBitVariantDataFormat::V0 && dataSize == 1)
		{
			hwVersion = (EmotiBitVersion)(*nvmData);
			Serial.print("[NVM VARIANT INFO] HW version: "); Serial.println(EmotiBitVersionController::getHardwareVersion((EmotiBitVersion)hwVersion));
			sku = EmotiBitVariants::EMOTIBIT_SKU_MD;
			Serial.print("[NVM VARIANT INFO] SKU: "); Serial.println(sku);
			emotiBitNumber = UINT32_MAX;
			Serial.println("No EmotiBitNumber recorded for this HW version");
		}
		else if (datatypeVersion == (uint8_t)EmotiBitVariantDataFormat::V1)
		{
			EmotiBitVariantInfo* variantInfo;
			variantInfo = (EmotiBitVariantInfo*)nvmData;
			hwVersion = (EmotiBitVersion)variantInfo->hwVersion;
			Serial.print("[NVM VARIANT INFO] HW version: "); Serial.println(EmotiBitVersionController::getHardwareVersion((EmotiBitVersion)hwVersion));
			sku = String(variantInfo->sku);
			Serial.print("[NVM VARIANT INFO] SKU version: "); Serial.println(sku);
			emotiBitNumber = variantInfo->emotiBitNumber;
			Serial.print("[NVM VARIANT INFO] EmotiBit Number: "); Serial.println(emotiBitNumber);
		}
		return true;
	}
	else if (status == (uint8_t)EmotiBitNvmController::Status::MEMORY_NOT_UPDATED)
	{
		Serial.println("NVM not updated with variant info");
		return false;
	}
	else
	{
		Serial.print("Failed to read Variant Info. Error Code: "); Serial.println(status);
		return false;
	}
}

void EmotiBitVersionController::updateVersionParameterTable(TwoWire &emotibit_i2c, EmotiBitHardwareParameterTable &hardwareParameterTable)
{
	uint8_t status;
	Serial.println("Updating HardwareParameterTable");
	emotibit_i2c.beginTransmission(SI7013_I2C_ADDR);
	status = emotibit_i2c.endTransmission();
	if (status == 0)
	{
		hardwareParameterTable.isSi7013Present = true;
		Serial.println("Si-7013: Present");
	}
	else
	{
		hardwareParameterTable.isSi7013Present = false;
		Serial.print("Si-7013: Absent."); Serial.print(" I2C Error code:"); Serial.println(status);
	}

	emotibit_i2c.beginTransmission(EEPROM_I2C_ADDR);
	status = emotibit_i2c.endTransmission();
	if (status == 0)
	{
		hardwareParameterTable.isEepromPresent = true;
		Serial.println("EEPROM: Present");
	}
	else
	{
		hardwareParameterTable.isEepromPresent = false;
		Serial.print("EEPROM: Absent."); Serial.print(" I2C Error code:"); Serial.println(status);
	}

	emotibit_i2c.beginTransmission(MLX90632_I2C_ADDR);
	status = emotibit_i2c.endTransmission();
	if (status == 0)
	{
		hardwareParameterTable.isThermopilePresent = true;
		Serial.println("Thermopile: Present");
	}
	else
	{
		hardwareParameterTable.isThermopilePresent = false;
		Serial.print("Thermopile: Absent."); Serial.print(" I2C Error code:"); Serial.println(status);
	}
}

bool EmotiBitVersionController::detectVariantFromHardware(TwoWire &emotibit_i2c, EmotiBitVersion &hwVersion, String &sku)
{
	EmotiBitHardwareParameterTable hardwareParametertable;
	updateVersionParameterTable(emotibit_i2c, hardwareParametertable);

	if (hibernatePinLogic == PinActivationLogic::ACTIVE_HIGH)
	{
		if (hardwareParametertable.isSi7013Present && hardwareParametertable.isThermopilePresent)
		{
			Serial.println("HW Version Detected: V2");
			hwVersion = EmotiBitVersion::V02H;
			sku = EmotiBitVariants::EMOTIBIT_SKU_MD;
			return true;
		}
		else
		{
			Serial.println("HW Version not detected.");
			return false;
		}
	}
	else
	{
		if (hardwareParametertable.isSi7013Present && !hardwareParametertable.isEepromPresent)
		{
			Serial.println("HW Version Detected: V3");
			hwVersion = EmotiBitVersion::V03B;
			sku = EmotiBitVariants::EMOTIBIT_SKU_MD;
			return true;
		}
		else if (hardwareParametertable.isEepromPresent && !hardwareParametertable.isSi7013Present)
		{
			Serial.println("HW Version Detected: V4");
			hwVersion = EmotiBitVersion::V04A;
			if (hardwareParametertable.isThermopilePresent)
			{
				Serial.println("SKU Version Detected: MD");
				sku = EmotiBitVariants::EMOTIBIT_SKU_MD;
				return true;
			}
			else
			{
				Serial.println("SKU Version Detected: EM");
				sku = EmotiBitVariants::EMOTIBIT_SKU_EM;
				return true;
			}
		}
		else
		{
			Serial.print("Unable to detect HW version");
			return false;
		}
	}
	
}

/*
EmotiBitVersionController::EmotiBitVersion EmotiBitVersionController::detectEmotiBitVersion(TwoWire* EmotiBit_i2c, uint8_t flashMemoryI2cAddress)
{
	_versionEst = EmotiBitVersion::UNKNOWN;
	_otpEmotiBitVersion = -1;
	// V02B, V02H and V03B all have pin 6 as hibernate, and this code supports only those versions
	pinMode(HIBERNATE_PIN, INPUT);
	if (digitalRead(HIBERNATE_PIN) == HIGH)
	{
		_versionEst = EmotiBitVersion::V02H;
		pinMode(HIBERNATE_PIN, OUTPUT);
		digitalWrite(HIBERNATE_PIN, LOW);
		delay(50);
		Serial.println("Version estimate: V02");
	}
	else
	{
		_versionEst = EmotiBitVersion::V03B;
		pinMode(HIBERNATE_PIN, OUTPUT); 
		digitalWrite(HIBERNATE_PIN, HIGH);
		delay(50);
		// check if flash module is present on the I2c line. This confirms EmotiBit V4+
		if (flashMemoryI2cAddress != 255)
		{
			EmotiBit_i2c->beginTransmission(flashMemoryI2cAddress);
			if (EmotiBit_i2c->endTransmission() == 0) // flash module detected
			{
				_versionEst = EmotiBitVersion::V04A;
				Serial.println("Version estimate: V04");
				// ToDo: in the future, we will also be reading the flash to get the EmotiBit version stored there
				versionDetectionComplete = true;
				return _versionEst;
			}
			else
			{
				Serial.println("Version estimate: V03");
			}
		}
	}
	/*
	bool status;
	Serial.println("--------------------------- DETECTING EMOTIBIT VERSION --------------------------");
	Serial.println("Making hibernate LOW");
	digitalWrite(HIBERNATE_PIN, LOW);
	// Try Setting up SD Card
	status = detectSdCard();
	if (status)
	{
		// SD-Card detected
		_versionEst = EmotiBitVersion::V02H;
	}
	else
	{
		// Sd-Card not detected in V2 startup
		Serial.println("SD-Card not detected with V2 power up Sequence.\nTrying V3 power-up sequence");
		Serial.println("Making hibernate HIGH");
		digitalWrite(HIBERNATE_PIN, HIGH);
		// Try Setting up SD Card
		status = detectSdCard();
		if (status)
		{
			_versionEst = EmotiBitVersion::V03B;
			
			
		}
		else
		{
			//pinMode(EMOTIBIT_I2C_CLK_PIN, OUTPUT);
			//digitalWrite(EMOTIBIT_I2C_CLK_PIN, LOW);
			// SD-Card not present
			_versionEst = EmotiBitVersion::UNKNOWN;
			versionDetectionComplete = false;
			return _versionEst;
		}
	}
	//*
	bool status = true;
	// Setup Temperature / Humidity Sensor
	Serial.println("\n\nReading Temperature / Humidity Sensor OTP for EmotiBit version");
	// moved the macro definition from EdaCorrection to EmotiBitVersionController
#ifdef USE_ALT_SI7013 
	status = _tempHumiditySensor.setup(*EmotiBit_i2c, 0x41);
#else
	//ToDo: Fix setup() function in Si7013 library. it returns true, irrespective of sensor being present or not
	status = _tempHumiditySensor.setup(*EmotiBit_i2c);
#endif
	if (status)
	{
		// Si-7013 detected on the EmotiBit
		if (!_tempHumiditySensor.sendCommand(0x00)) // returns false if failed to send command
		{
			versionDetectionComplete = false;
			return _versionEst;
		}
		// Sensor detected
		else
		{
			while (_tempHumiditySensor.getStatus() != Si7013::STATUS_IDLE);
			_otpEmotiBitVersion = readEmotiBitVersionFromSi7013();
			Serial.println("Si7013 detected.");
		}
	}
	else
	{
		// Si-7013 Not detected
		Serial.println("Si-7013 not Detected on EmotiBit.");
		versionDetectionComplete = false;
		return _versionEst;
	}

	if (_otpEmotiBitVersion == 255)
	{
		Serial.println("OTP has not yet been updated");
		Serial.print("using the Estimated emotibit version detected from power up sequence: "); Serial.println(getHardwareVersion((EmotiBitVersion)_versionEst));
		Serial.println("-------------------------- END DETECTING EMOTIBIT VERSION --------------------------------");
		versionDetectionComplete = true;
		return _versionEst;
	}
	else
	{
		if (_otpEmotiBitVersion == (int)_versionEst)
		{
			Serial.println("######################");
			Serial.print("The EmotiBit Version read from OTP is: ");
			Serial.println(getHardwareVersion((EmotiBitVersion)_otpEmotiBitVersion));
			//Serial.println(emotibitVersion);
			Serial.println("######################");
			Serial.println("-------------------------- END DETECTING EMOTIBIT VERSION --------------------------------");
			versionDetectionComplete = true;
			return (EmotiBitVersionController::EmotiBitVersion)_otpEmotiBitVersion;
		}
		else
		{
			// Version on EmotiBit not equal to estimated version
			// ToDO: Resolve conflict. 
			Serial.println("Mismatch between estimated Version and Version found on Emotibit");
		}
	}
}

int EmotiBitVersionController::readEmotiBitVersionFromSi7013()
{
	uint8_t emotibitVersion = 0;
	emotibitVersion = (uint8_t)_tempHumiditySensor.readRegister8(EMOTIBIT_VERSION_ADDR_SI7013_OTP, true);
	return emotibitVersion;
}
*/
