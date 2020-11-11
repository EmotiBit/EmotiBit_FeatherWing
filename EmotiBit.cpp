#include "EmotiBit.h"

FlashStorage(samdFlashStorage, SamdStorageAdcValues);

EmotiBit* myEmotiBit = nullptr;
void(*onInterruptCallback)(void);

EmotiBit::EmotiBit() 
{
	
}

bool EmotiBit::setSamplingRates(SamplingRates s) 
{
	_samplingRates = s;
	_samplingRates.ppg = ppgSettings.sampleRate / ppgSettings.sampleAverage;
	_samplingRates.accelerometer = 25.f * pow(2.f, ((float)imuSettings.acc_odr - 6.f));	// See lookup table in BMI160 datasheet
	_samplingRates.gyroscope = 25.f * pow(2.f, ((float)imuSettings.gyr_odr - 6.f));		// See lookup table in BMI160 datasheet
	_samplingRates.magnetometer = 25.f * pow(2.f, ((float)imuSettings.mag_odr - 6.f));	// See lookup table in BMI160 datasheet
}

bool EmotiBit::setSamplesAveraged(SamplesAveraged s) 
{
	_samplesAveraged = s;
}

bool EmotiBit::getBit(uint8_t num, uint8_t bit) 
{
	uint8_t mask = 1 << bit;
	return mask == (num & mask);
}
/*!
 * @brief This API reads the data from the given register address of the sensor.
 https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L579
 */
void EmotiBit::bmm150GetRegs(uint8_t address, uint8_t* dest, uint16_t len) 
{
	for (uint16_t i = 0; i < len; i++) {
		BMI160.setRegister(BMI160_MAG_IF_2, address); //tell BMI160 to read BMM150 Address, automatically flipping the MAN_OP bit to 1
		delay(BMI160_AUX_COM_DELAY);
		//add poll
		uint8_t p= BMI160.getRegister(BMI160_RA_STATUS);
		delay(BMI160_READ_WRITE_DELAY);
		while (EmotiBit::getBit(p,BMI160_STATUS_MAG_MAN_OP) != 0) { //wait for MAN_OP to switch back to 0
			p= BMI160.getRegister(BMI160_RA_STATUS);
			delay(BMI160_READ_WRITE_DELAY);
		}
		*dest = BMI160.getRegister(BMI160_RA_MAG_X_L);		//read in from MAG_[X-Z]
		delay(BMI160_READ_WRITE_DELAY);
		dest++;
		address++;
	}
}

/*!
 * @brief This internal API reads the trim registers of the BMM150 magnetometer and stores
 * the trim values in bmm150TrimData
 *
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L99
 */
void EmotiBit::bmm150ReadTrimRegisters() 
{
	int8_t rslt;
	uint8_t trim_x1y1[2] = { 0 };
	uint8_t trim_xyz_data[4] = { 0 };
	uint8_t trim_xy1xy2[10] = { 0 };
	uint16_t temp_msb = 0;

	/* Trim register value is read */
	EmotiBit::bmm150GetRegs(BMM150_DIG_X1, trim_x1y1, 2);
	EmotiBit::bmm150GetRegs(BMM150_DIG_Z4_LSB, trim_xyz_data, 4);
	EmotiBit::bmm150GetRegs(BMM150_DIG_Z2_LSB, trim_xy1xy2, 10);
	bmm150TrimData.dig_x1 = (int8_t)trim_x1y1[0];
	bmm150TrimData.dig_y1 = (int8_t)trim_x1y1[1];
	bmm150TrimData.dig_x2 = (int8_t)trim_xyz_data[2];
	bmm150TrimData.dig_y2 = (int8_t)trim_xyz_data[3];
	temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
	bmm150TrimData.dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);
	temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
	bmm150TrimData.dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);
	temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
	bmm150TrimData.dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);
	temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
	bmm150TrimData.dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);
	bmm150TrimData.dig_xy1 = trim_xy1xy2[9];
	bmm150TrimData.dig_xy2 = (int8_t)trim_xy1xy2[8];
	temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
	bmm150TrimData.dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);
}

uint8_t EmotiBit::setup(Version version, size_t bufferCapacity)
{
	_version = version;

	bool status = true;

	SamdStorageAdcValues samdStorageAdcValues;

	String fwVersionModifier = "";
	if (testingMode == TestingMode::ACUTE)
	{
		fwVersionModifier = "-TA";
	}
	else if (testingMode == TestingMode::CHRONIC)
	{
		fwVersionModifier = "-TC";
	}

	firmware_version += fwVersionModifier;

	Serial.print("\n\nEmotiBit version: ");
	Serial.println(getHardwareVersion());
	Serial.print("Firmware version: ");
	Serial.println(firmware_version);

	if (!_outDataPackets.reserve(OUT_MESSAGE_RESERVE_SIZE)) {
		Serial.println("Failed to reserve memory for output");
		while (true) {
			hibernate();
		}
	}
	
	uint32_t now = millis();
	while (!Serial.available() && millis() - now < 2000)
	{
	}
	while (Serial.available())
	{
		char input;
		input = Serial.read();
		
		if (input == 'A')
		{
			Serial.println("entered ADC Correction Mode");
			digitalWrite(LED_BUILTIN, LOW);
			Serial.println("If you are not a tester, please exit this mode by pressing Q");
			Serial.println("If you are a tester, make sure the correction rig is in place and press A");
			while (!Serial.available());
			if (Serial.read() == 'Q')
			{
				break;
			}
			else
			{	
				_debugMode = false;
				analogReadResolution(12); // Setting ADC resolution to 12 bits
				samdStorageAdcValues = samdFlashStorage.read();
				if (!samdStorageAdcValues.valid)
				{
#ifdef ADC_CORRECTION_VERBOSE
					Serial.println("Using the rig to generate correction values");
#endif
					Serial.println("Enter any character. Then remove the USB cable and plug after when the EmotiBit LED blinks");
					while (!Serial.available()); Serial.read();
					uint32_t timeCurrent = millis();
					while (millis() - timeCurrent < 2000);
					digitalWrite(LED_BUILTIN, HIGH);
					AdcCorrection adcCorrection(AdcCorrection::AdcCorrectionRigVersion::VER_0, AdcCorrection::DataFormatVersion::DATA_FORMAT_0);
					adcCorrection.begin();
#ifdef ADC_CORRECTION_VERBOSE
					Serial.println("atwincDataArray");
					for (int i = 0; i < adcCorrection.ATWINC_DATA_ARRAY_SIZE; i++)
					{
						Serial.print("  "); Serial.print(adcCorrection.atwincDataArray[i]);
					}
#endif
				
#ifdef ADC_CORRECTION_VERBOSE
					Serial.println("\nStoring on the SAMD flash for the first time");
#endif
					samdStorageAdcValues._gainCorrection = adcCorrection.getGainCorrection();
					samdStorageAdcValues._offsetCorrection = adcCorrection.getOffsetCorrection();
					samdStorageAdcValues.valid = true;
					samdFlashStorage.write(samdStorageAdcValues);
#ifdef ADC_CORRECTION_VERBOSE
					Serial.println("Enabling the ADC to use the correction values");
#endif
					uint16_t adcBeforeCorrection[3], adcAfterCorrection[3];
					adcBeforeCorrection[0] = analogRead(A0);
					adcBeforeCorrection[1] = analogRead(A1);
					adcBeforeCorrection[2] = analogRead(A2);
					analogReadCorrection(samdStorageAdcValues._offsetCorrection, samdStorageAdcValues._gainCorrection);
#ifdef ADC_CORRECTION_VERBOSE
					Serial.println("Comparing results before and after correction");
#endif
					adcAfterCorrection[0] = analogRead(A0);
					adcAfterCorrection[1] = analogRead(A1);
					adcAfterCorrection[2] = analogRead(A2);
					
					digitalWrite(LED_BUILTIN, LOW);
					while (!Serial.available())
					{
						if (millis() - timeCurrent > 2000)
						{
							Serial.println("Enter a character to continue to record results");
							timeCurrent = millis();
						}
						digitalWrite(LED_BUILTIN, HIGH);
						delay(200);
						digitalWrite(LED_BUILTIN, LOW);
						delay(200);
						digitalWrite(LED_BUILTIN, HIGH);
						delay(200);
						digitalWrite(LED_BUILTIN, LOW);
						delay(1000);
					}
					Serial.read();// pop from the buffer
					Serial.println("COPY ANS PASTE the folowing into the feather records");
					Serial.println("==============================================");
					//Serial.print("Gain correction:"); Serial.println(samdStorageAdcValues._gainCorrection);
					//Serial.print("offset correction:"); Serial.println(samdStorageAdcValues._offsetCorrection);
					//Serial.print("True Value"); Serial.print("\tBefore Correction"); Serial.println("\tAfter Correction");
					//Serial.print(round((float)(1.f / 11.f) * 4096.f)); Serial.print("\t\t"); Serial.print(adcBeforeCorrection[0]); Serial.print("\t\t"); Serial.println(adcAfterCorrection[0]);
					//Serial.print(round((float)(1.f / 2.f) * 4096.f)); Serial.print("\t\t"); Serial.print(adcBeforeCorrection[1]); Serial.print("\t\t"); Serial.println(adcAfterCorrection[1]);
					//Serial.print(round((float)(10.f / 11.f) * 4096.f)); Serial.print("\t\t"); Serial.print(adcBeforeCorrection[2]); Serial.print("\t\t"); Serial.println(adcAfterCorrection[2]);
					Serial.print(samdStorageAdcValues._gainCorrection); Serial.print(",");
					Serial.print(samdStorageAdcValues._offsetCorrection); Serial.print(",");
					Serial.print(adcBeforeCorrection[0]); Serial.print(",");
					Serial.print(adcBeforeCorrection[1]); Serial.print(",");
					Serial.print(adcBeforeCorrection[2]); Serial.print(",");
					Serial.print(adcAfterCorrection[0]); Serial.print(",");
					Serial.print(adcAfterCorrection[1]); Serial.print(",");
					Serial.print(adcAfterCorrection[2]); 
					uint8_t tempData[12], tempdata2[3];
					adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_PRIMARY_DATA, 12, tempData);
					for (int i = 0; i < adcCorrection.ATWINC_DATA_ARRAY_SIZE; i++)
					{
						Serial.print(","); Serial.print(tempData[i]);
					}
					Serial.println("\n==============================================");
#ifdef ADC_CORRECTION_VERBOSE
					Serial.println("\ntesting AT-WINC flash read");
					adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_PRIMARY_DATA, 12, tempData);
					adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_DUPLICATE_DATA, 12, tempData);
					adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_METADATA_LOC, 3, tempData, 0);
#endif
				}
				else
				{
					Serial.println("data exists on the samd flash");
					Serial.println("No R/W actions performed on the AT-WINC flash");
					Serial.println("reading from the samd flash");
					Serial.print("Gain correction:"); Serial.println(samdStorageAdcValues._gainCorrection);
					Serial.print("offset correction:"); Serial.println(samdStorageAdcValues._offsetCorrection);
				}
			}
		}
		else if (input == 'E')
		{
			Serial.println("EDA Correction Mode");
			Serial.println("If you are not a tester, please exit this mode by pressing Q");
			Serial.println("If you are a tester, press A to proceed");
			while (!Serial.available());
			char choice = Serial.read();
			if (choice == 'Q')
			{
				break;
			}
			else if (choice == 'A')
			{
				_debugMode = false;
				EdaCorrection::Status status;
				status = edaCorrection.enterUpdateMode();
			}

		}
		else
		{
			_debugMode = true;
			Serial.println("\nENTERING DEBUG MODE\n");
		}

	}


	// Setup data buffers
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDA] = &eda;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDL] = &edl;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDR] = &edr;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = &ppgInfrared;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_RED] = &ppgRed;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_GREEN] = &ppgGreen;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = &temp0;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::THERMOPILE] = &therm0;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = &humidity0;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = &accelX;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = &accelY;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = &accelZ;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = &gyroX;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = &gyroY;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = &gyroZ;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = &magX;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = &magY;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = &magZ;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = &batteryVoltage;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = &batteryPercent;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW] = &dataOverflow;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DATA_CLIPPING] = &dataClipping;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DEBUG] = &debugBuffer;

	_vcc = 3.3f;						// Vcc voltage

	if (_version == Version::V01B || _version == Version::V01C)
	{
		Serial.println("This code is not compatible with V01 (Alpha) boards");
		Serial.println("Download v0.6.0 for Alpha boards");
		while (true);
	}

	// Set board-specific pins and constants
#if defined(ADAFRUIT_FEATHER_M0)
	_batteryReadPin = A7;
	_adcBits = 12;
	adcRes = pow(2, _adcBits);	// adc bit resolution
	if (_version == Version::V01B || _version == Version::V01C)
	{
		_hibernatePin = 5; // gpio pin assigned to the mosfet
		buttonPin = 13;
		_edlPin = A3;
		_edrPin = A4;
		_sdCardChipSelectPin = 6;
	}
	else if (_version == Version::V02H)
	{
		_hibernatePin = 6; // gpio pin assigned to the mosfet
		buttonPin = 12;
		_edlPin = A4;
		_edrPin = A3;
		_sdCardChipSelectPin = 19;
		edrAmplification = 100.f / 3.3f;
		edaFeedbackAmpR = 5070000.f; // empirically derived average edaFeedbackAmpR in Ohms (theoretical 4990000.f)
		vRef1 = 0.426f; // empirically derived minimum voltage divider value [theoretical 15/(15 + 100)]
		vRef2 = 1.634591173; // empirically derived average voltage divider value [theoretical _vcc * (100.f / (100.f + 100.f))]
	}
	else if (_version == Version::V02B)
	{
		_hibernatePin = 6; // gpio pin assigned to the mosfet
		buttonPin = 12;
		_edlPin = A4;
		_edrPin = A3;
		_sdCardChipSelectPin = 19;
		edrAmplification = 100.f / 1.2f;
		edaFeedbackAmpR = 4990000.f;; // edaFeedbackAmpR in Ohms
	}
#elif defined(ADAFRUIT_BLUEFRUIT_NRF52_FEATHER)
	if (_version == Version::V01B || _version == Version::V01C)
	{
		_hibernatePin = 27;//gpio pin assigned ot the mosfet
		buttonPin = 16;
		_gsrLowPin = A3;
		_gsrHighPin = A4;
	}
#endif

	// Print board-specific settings
	Serial.println("\nHW version-specific settings:");
	Serial.print("buttonPin = "); Serial.println(buttonPin);
	Serial.print("_batteryReadPin = "); Serial.println(_batteryReadPin);
	Serial.print("_hibernatePin = "); Serial.println(_hibernatePin);
	Serial.print("_edlPin = "); Serial.println(_edlPin);
	Serial.print("_edrPin = "); Serial.println(_edrPin);
	Serial.print("_vcc = "); Serial.println(_vcc);
	Serial.print("adcRes = "); Serial.println(adcRes);
	Serial.print("edaFeedbackAmpR = "); Serial.println(edaFeedbackAmpR);
	Serial.print("edrAmplification = "); Serial.println(edrAmplification);

	// Setup switch
	if (buttonPin != LED_BUILTIN) 
	{
		// If the LED_BUILTIN and buttonPin are the same leave it as it was
		// Otherwise setup the input
		pinMode(buttonPin, INPUT);
	}

	// Setup battery Reading
	pinMode(_batteryReadPin, INPUT);

	// Enable analog circuitry
	Serial.print("\nCycling EmotiBit power...");
	pinMode(_hibernatePin, OUTPUT);
	digitalWrite(_hibernatePin, HIGH);

	delay(250);
	
	// Enable analog circuitry
	Serial.println("Enabling EmotiBit power...");
	pinMode(_hibernatePin, OUTPUT);
	digitalWrite(_hibernatePin, LOW);

	Serial.println("\nSensor setup:");
	// Setup EDA
	Serial.println("Configuring EDA...");
	pinMode(_edlPin, INPUT);
	pinMode(_edrPin, INPUT);
	samdStorageAdcValues = samdFlashStorage.read(); // reading from samd flash storage into local struct
	if (!samdStorageAdcValues.valid)
	{
#ifdef ADC_CORRECTION_VERBOSE
		Serial.println("No correction found on SAMD. Calculating correction by reading ATWINC");
#endif
		AdcCorrection adcCorrection(AdcCorrection::AdcCorrectionRigVersion::UNKNOWN, AdcCorrection::DataFormatVersion::UNKNOWN);
		if (adcCorrection.atwincAdcDataCorruptionTest == AdcCorrection::Status::FAILURE || adcCorrection.atwincAdcMetaDataCorruptionTest == AdcCorrection::Status::FAILURE)
		{
			Serial.println("data on atwinc corrupted or not present");
			Serial.println("Using the ADC withut any correction");
		}
		else 
		{
			Serial.println("Reading correction data from the flash");
			Serial.println("Calculating correction values");
			adcCorrection.calcCorrectionValues();
			// ToDo: check if above function actually worked
			// Store the values on the flash
			Serial.println("Storing correction values on the SAMD flash");
			samdStorageAdcValues._gainCorrection = adcCorrection.getGainCorrection();
			samdStorageAdcValues._offsetCorrection = adcCorrection.getOffsetCorrection();
			samdStorageAdcValues.valid = true;
			samdFlashStorage.write(samdStorageAdcValues);// Writing it to the SAMD flash storage
			// reinitializing the wifi module
			/*WiFi.setPins(8, 7, 4, 2);
			nm_bsp_init();*/
			Serial.print("Gain Correction:"); Serial.print(samdStorageAdcValues._gainCorrection); Serial.print("\toffset correction:"); Serial.println(samdStorageAdcValues._offsetCorrection);
			Serial.println("Enabling the ADC with the correction values");
			analogReadCorrection(samdStorageAdcValues._offsetCorrection, samdStorageAdcValues._gainCorrection);
		}
	}
	else
	{
		Serial.println("Correction data exists on the samd flash");
		Serial.print("Gain Correction:"); Serial.print(samdStorageAdcValues._gainCorrection); Serial.print("\toffset correction:"); Serial.println(samdStorageAdcValues._offsetCorrection);
		Serial.println("Enabling the ADC with the correction values");
		analogReadCorrection(samdStorageAdcValues._offsetCorrection, samdStorageAdcValues._gainCorrection);
	}
	analogReadResolution(_adcBits);

	if (_EmotiBit_i2c != nullptr)
	{
		delete(_EmotiBit_i2c);
	}
	_EmotiBit_i2c = new TwoWire(&sercom1, 11, 13);
	// Flush the I2C
	Serial.print("Setting up I2C....");
	_EmotiBit_i2c->begin();
	uint32_t i2cRate = 100000;
	Serial.print("setting clock to");
	Serial.print(i2cRate);
	_EmotiBit_i2c->setClock(i2cRate);
	Serial.print("...setting PIO_SERCOM");
	pinPeripheral(11, PIO_SERCOM);
	pinPeripheral(13, PIO_SERCOM);
	Serial.print("...flushing");
	_EmotiBit_i2c->flush();
	Serial.print("\n");
	//_EmotiBit_i2c->endTransmission();
	//_EmotiBit_i2c->clearWriteError();
	//_EmotiBit_i2c->end();
	
	// setup LED DRIVER
	Serial.println("Initializing NCP5623....");
	led.begin(*_EmotiBit_i2c);
	led.setCurrent(26);
	led.setLEDpwm((uint8_t)Led::RED, 8);
	led.setLEDpwm((uint8_t)Led::BLUE, 8);
	led.setLEDpwm((uint8_t)Led::YELLOW, 8);
	led.setLED(uint8_t(EmotiBit::Led::RED), false);
	led.setLED(uint8_t(EmotiBit::Led::BLUE), false);
	led.setLED(uint8_t(EmotiBit::Led::YELLOW), false);
	chipBegun.NCP5623 = true;

	//// Setup PPG sensor
	Serial.println("Initializing MAX30101....");
	// Initialize sensor
	while (ppgSensor.begin(*_EmotiBit_i2c) == false) // reads the part number to confirm device
	{
		Serial.println("MAX30101 was not found. Please check wiring/power. ");
		_EmotiBit_i2c->flush();
		_EmotiBit_i2c->endTransmission();
		_EmotiBit_i2c->clearWriteError();
		_EmotiBit_i2c->end();
		static uint32_t hibernateTimer = millis();
		if (millis() - hibernateTimer > 2000)
		{
			hibernate();
		}
	}
	ppgSensor.wakeUp();
	ppgSensor.softReset();

	ppgSensor.setup(
		ppgSettings.ledPowerLevel, 
		ppgSettings.sampleAverage, 
		ppgSettings.ledMode, 
		ppgSettings.sampleRate, 
		ppgSettings.pulseWidth, 
		ppgSettings.adcRange
	); 
	ppgSensor.check();
	chipBegun.MAX30101 = true;

	// Setup IMU
	Serial.println("Initializing IMU device....");
	status = BMI160.begin(BMI160GenClass::I2C_MODE, *_EmotiBit_i2c);
	if (status)
	{
		uint8_t dev_id = BMI160.getDeviceID();
		Serial.print("DEVICE ID: ");
		Serial.println(dev_id, HEX);

		// Accelerometer
		_accelerometerRange = 8;
		BMI160.setAccelerometerRange(_accelerometerRange);
		BMI160.setAccelDLPFMode(BMI160DLPFMode::BMI160_DLPF_MODE_NORM);
		//BMI160.setAccelRate(BMI160AccelRate::BMI160_ACCEL_RATE_25HZ);
		BMI160.setAccelRate(BMI160AccelRate::BMI160_ACCEL_RATE_100HZ);

		// Gyroscope
		_gyroRange = 1000;
		BMI160.setGyroRange(_gyroRange);
		BMI160.setGyroDLPFMode(BMI160DLPFMode::BMI160_DLPF_MODE_NORM);
		//BMI160.setGyroRate(BMI160GyroRate::BMI160_GYRO_RATE_25HZ);
		BMI160.setGyroRate(BMI160GyroRate::BMI160_GYRO_RATE_100HZ);

		// Magnetometer
		BMI160.setRegister(BMI160_MAG_IF_0, BMM150_BASED_I2C_ADDR, BMM150_BASED_I2C_MASK); // I2C MAG
		delay(BMI160_AUX_COM_DELAY);
		//initially load into setup mode to read trim values
		BMI160.setRegister(BMI160_MAG_IF_1, BMI160_MANUAL_MODE_EN_MSK, BMI160_MANUAL_MODE_EN_MSK);
		delay(BMI160_AUX_COM_DELAY);
		EmotiBit::bmm150ReadTrimRegisters();

		BMI160.setRegister(BMI160_MAG_IF_2, BMM150_DATA_REG); // ADD_BMM_DATA
		delay(BMI160_AUX_COM_DELAY);
		//BMI160.setRegister(BMI160_MAG_IF_3, BMM150_OPMODE_REG); // ADD_BMM_MEASURE
		//delay(BMI160_AUX_COM_DELAY);

		// Following example at https://github.com/BoschSensortec/BMI160_driver#auxiliary-fifo-data-parsing 

		// Put the BMM150 in normal mode (may or may not be necessary if putting in force mode later)
		// BMI160.setRegister(BMM150_OPMODE_REG, BMM150_DATA_RATE_10HZ | BMM150_NORMAL_MODE);
		//BMI160.setRegister(BMI160_MAG_IF_4, BMM150_DATA_RATE_10HZ | BMM150_NORMAL_MODE);

		//BMI160.setRegister(BMI160_MAG_IF_4, BMM150_NORMAL_MODE);
		BMI160.reg_write_bits(BMI160_MAG_IF_4, BMM150_NORMAL_MODE, BMM150_OP_MODE_BIT, BMM150_OP_MODE_LEN);
		BMI160.setRegister(BMI160_MAG_IF_3, BMM150_OP_MODE_ADDR);
		delay(BMI160_AUX_COM_DELAY);

		// Already done in setup
		///* Set BMM150 repetitions for X/Y-Axis */
		//BMI160.setRegister(BMI160_MAG_IF_4, BMM150_LOWPOWER_REPXY);             //Added for BMM150 Support
		//BMI160.setRegister(BMI160_MAG_IF_3, BMM150_XY_REP_REG);                 //Added for BMM150 Support
		//delay(BMI160_AUX_COM_DELAY);

		///* Set BMM150 repetitions for Z-Axis */
		//BMI160.setRegister(BMI160_MAG_IF_4, BMM150_LOWPOWER_REPXY);              //Added for BMM150 Support
		//BMI160.setRegister(BMI160_MAG_IF_3, BMM150_Z_REP_REG);                  //Added for BMM150 Support
		//delay(BMI160_AUX_COM_DELAY);

		//BMI160.setRegister(BMI160_MAG_IF_4, BMM150_DATA_RATE_25HZ);
		BMI160.reg_write_bits(BMI160_MAG_IF_4, BMM150_DATA_RATE_10HZ, BMM150_DATA_RATE_BIT, BMM150_DATA_RATE_LEN);
		BMI160.setRegister(BMI160_MAG_IF_3, BMM150_OP_MODE_ADDR);
		delay(BMI160_AUX_COM_DELAY);

		//BMI160.setRegister(BMI160_MAG_IF_4, BMM150_FORCED_MODE);
		BMI160.reg_write_bits(BMI160_MAG_IF_4, BMM150_FORCED_MODE, BMM150_OP_MODE_BIT, BMM150_OP_MODE_LEN);
		BMI160.setRegister(BMI160_MAG_IF_3, BMM150_OP_MODE_ADDR);
		delay(BMI160_AUX_COM_DELAY);


		// Setup the BMI160 AUX
		// Set the auto mode address
		BMI160.setRegister(BMI160_MAG_IF_2, BMM150_DATA_X_LSB);
		delay(BMI160_AUX_COM_DELAY);

		// Set the AUX ODR
		BMI160.setMagRate(BMI160MagRate::BMI160_MAG_RATE_100HZ);

		// Disable manual mode (i.e. enable auto mode)
		BMI160.setRegister(BMI160_MAG_IF_1, BMI160_DISABLE, BMI160_MANUAL_MODE_EN_MSK);

		// Set the burst length
		BMI160.setRegister(BMI160_MAG_IF_1, BMI160_AUX_READ_BURST_MSK, BMI160_AUX_READ_BURST_MSK); // MAG data mode 8 byte burst
		delay(BMI160_AUX_COM_DELAY);

		// Bosch code sets the I2C register again here for an unknown reason
		BMI160.setRegister(BMI160_MAG_IF_0, BMM150_BASED_I2C_ADDR, BMM150_BASED_I2C_MASK); // I2C MAG
		delay(BMI160_AUX_COM_DELAY);

		// Setup the FIFO
		BMI160.setAccelFIFOEnabled(true);
		_imuFifoFrameLen += 6;
		BMI160.setGyroFIFOEnabled(true);
		_imuFifoFrameLen += 6;
		BMI160.setMagFIFOEnabled(true);
		_imuFifoFrameLen += 8;
		BMI160.setFIFOHeaderModeEnabled(false);
		if (_imuFifoFrameLen > _maxImuFifoFrameLen)
		{
			// ToDo: handle _imuFifoFrameLen > _maxImuFifoFrameLen
			Serial.println("UNHANDLED CASE: _imuFifoFrameLen > _maxImuFifoFrameLen");
			while (true);
		}
		chipBegun.BMI160 = true;
		chipBegun.BMM150 = true;
	}
	else
	{
		hibernate();
	}

	// ToDo: Add interrupts to accurately record timing of data capture

	//BMI160.detachInterrupt();
	//BMI160.setRegister()

	// Setup Temperature / Humidity Sensor
	Serial.println("Configuring Temperature / Humidity Sensor");
	status = tempHumiditySensor.setup(*_EmotiBit_i2c);
	if (status)
	{
		tempHumiditySensor.changeSetting(Si7013::Settings::RESOLUTION_H11_T11);
		tempHumiditySensor.changeSetting(Si7013::Settings::ADC_NORMAL);
		tempHumiditySensor.changeSetting(Si7013::Settings::VIN_UNBUFFERED);
		tempHumiditySensor.changeSetting(Si7013::Settings::VREFP_VDDA);
		tempHumiditySensor.changeSetting(Si7013::Settings::ADC_NO_HOLD);

		tempHumiditySensor.readSerialNumber();
		Serial.print("Si7013 Electronic Serial Number: ");
		Serial.print(tempHumiditySensor.sernum_a);
		Serial.print(", ");
		Serial.print(tempHumiditySensor.sernum_b);
		Serial.print("\n");
		Serial.print("Model: ");
		Serial.println(tempHumiditySensor._model);
		chipBegun.SI7013 = true;

		tempHumiditySensor.startHumidityTempMeasurement();
	}
	else
	{
		hibernate();
	}
	
	// Thermopile
	Serial.println("Configuring MLX90632");
	MLX90632::status returnError; // Required as a parameter for begin() function in the MLX library 
	status = thermopile.begin(deviceAddress.MLX, *_EmotiBit_i2c, returnError);
	if (status)
	{
		thermopile.setMeasurementRate(thermopileFs);
		thermopile.setMode(thermopileMode);
		uint8_t thermMode = thermopile.getMode();
		if (thermMode == MODE_CONTINUOUS)
		{
			Serial.println("MODE_CONTINUOUS");
		}
		if (thermMode == MODE_STEP)
		{
			Serial.println ("MODE_STEP");
		}
		if (thermMode == MODE_SLEEP)
		{
			Serial.println("MODE_SLEEP");
		}
		chipBegun.MLX90632 = true;
	}
	else
	{
		hibernate();
	}
	
	led.setLED(uint8_t(EmotiBit::Led::YELLOW), true);


	// setup sampling rates
	Serial.println("Setting up sampling rates...");
	EmotiBit::SamplingRates samplingRates;
	samplingRates.accelerometer = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
	samplingRates.gyroscope = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
	samplingRates.magnetometer = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
	samplingRates.eda = BASE_SAMPLING_FREQ / EDA_SAMPLING_DIV;
	samplingRates.humidity = BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2;
	samplingRates.temperature = BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2;
	samplingRates.thermopile = (float) BASE_SAMPLING_FREQ / (float) THERMOPILE_SAMPLING_DIV;
	setSamplingRates(samplingRates);

	// ToDo: make target down-sampled rates more transparent
	EmotiBit::SamplesAveraged samplesAveraged;
	samplesAveraged.eda = samplingRates.eda / 15;
	samplesAveraged.humidity = (float)samplingRates.humidity / 7.5f;
	samplesAveraged.temperature = (float)samplingRates.temperature / 7.5f;
	if (thermopileMode == MODE_CONTINUOUS)
	{
		samplesAveraged.thermopile = 1;
	}
	else
	{
		samplesAveraged.thermopile = (float)samplingRates.thermopile / 7.5f;
	}
	samplesAveraged.battery = BASE_SAMPLING_FREQ / BATTERY_SAMPLING_DIV / 1;
	setSamplesAveraged(samplesAveraged);

	// EDL Filter Parameters
	if (_version == Version::V02H)
	{
		edaCrossoverFilterFreq = 1.f / (2.f * PI * 200000.f * 0.0000047f);
		_edlDigFiltAlpha = pow(M_E, -2.f * PI * edaCrossoverFilterFreq / (_samplingRates.eda / _samplesAveraged.eda));
	}

	delay(500);

	// Setup SD Card
	setupSdCard();
	led.setLED(uint8_t(EmotiBit::Led::RED), true);

	//WiFi Setup;
	Serial.println("\nSetting up WiFi");
#if defined(ADAFRUIT_FEATHER_M0)
	WiFi.setPins(8, 7, 4, 2);
#endif
	WiFi.lowPowerMode();
	_emotiBitWiFi.begin();
	led.setLED(uint8_t(EmotiBit::Led::BLUE), true);

	setPowerMode(PowerMode::NORMAL_POWER);

	typeTags[(uint8_t)EmotiBit::DataType::EDA] = EmotiBitPacket::TypeTag::EDA;
	typeTags[(uint8_t)EmotiBit::DataType::EDL] = EmotiBitPacket::TypeTag::EDL;
	typeTags[(uint8_t)EmotiBit::DataType::EDR] = EmotiBitPacket::TypeTag::EDR;
	typeTags[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = EmotiBitPacket::TypeTag::PPG_INFRARED;
	typeTags[(uint8_t)EmotiBit::DataType::PPG_RED] = EmotiBitPacket::TypeTag::PPG_RED;
	typeTags[(uint8_t)EmotiBit::DataType::PPG_GREEN] = EmotiBitPacket::TypeTag::PPG_GREEN;
	typeTags[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = EmotiBitPacket::TypeTag::TEMPERATURE_0;
	typeTags[(uint8_t)EmotiBit::DataType::THERMOPILE] = EmotiBitPacket::TypeTag::THERMOPILE;
	typeTags[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = EmotiBitPacket::TypeTag::HUMIDITY_0;
	typeTags[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = EmotiBitPacket::TypeTag::ACCELEROMETER_X;
	typeTags[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = EmotiBitPacket::TypeTag::ACCELEROMETER_Y;
	typeTags[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = EmotiBitPacket::TypeTag::ACCELEROMETER_Z;
	typeTags[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = EmotiBitPacket::TypeTag::GYROSCOPE_X;
	typeTags[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = EmotiBitPacket::TypeTag::GYROSCOPE_Y;
	typeTags[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = EmotiBitPacket::TypeTag::GYROSCOPE_Z;
	typeTags[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = EmotiBitPacket::TypeTag::MAGNETOMETER_X;
	typeTags[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = EmotiBitPacket::TypeTag::MAGNETOMETER_Y;
	typeTags[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = EmotiBitPacket::TypeTag::MAGNETOMETER_Z;
	typeTags[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = EmotiBitPacket::TypeTag::BATTERY_VOLTAGE;
	typeTags[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = EmotiBitPacket::TypeTag::BATTERY_PERCENT;
	typeTags[(uint8_t)EmotiBit::DataType::DATA_CLIPPING] = EmotiBitPacket::TypeTag::DATA_CLIPPING;
	typeTags[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW] = EmotiBitPacket::TypeTag::DATA_OVERFLOW;
	typeTags[(uint8_t)EmotiBit::DataType::DEBUG] = EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG;

	printLen[(uint8_t)EmotiBit::DataType::EDA] = 6;
	printLen[(uint8_t)EmotiBit::DataType::EDL] = 6;
	printLen[(uint8_t)EmotiBit::DataType::EDR] = 6;
	printLen[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = 0;
	printLen[(uint8_t)EmotiBit::DataType::PPG_RED] = 0;
	printLen[(uint8_t)EmotiBit::DataType::PPG_GREEN] = 0;
	printLen[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = 3;
	printLen[(uint8_t)EmotiBit::DataType::THERMOPILE] = 3;
	printLen[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = 3;
	printLen[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = 3;
	printLen[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = 3;
	printLen[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = 3;
	printLen[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = 3;
	printLen[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = 3;
	printLen[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = 3;
	printLen[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = 0;
	printLen[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = 0;
	printLen[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = 0;
	printLen[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = 2;
	printLen[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = 0;
	printLen[(uint8_t)EmotiBit::DataType::DEBUG] = 0;

	sendData[(uint8_t)EmotiBit::DataType::EDA] = true;
	sendData[(uint8_t)EmotiBit::DataType::EDL] = true;
	sendData[(uint8_t)EmotiBit::DataType::EDR] = true;
	sendData[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = true;
	sendData[(uint8_t)EmotiBit::DataType::PPG_RED] = true;
	sendData[(uint8_t)EmotiBit::DataType::PPG_GREEN] = true;
	sendData[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = true;
	sendData[(uint8_t)EmotiBit::DataType::THERMOPILE] = true;
	sendData[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = true;
	sendData[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = true;
	sendData[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = true;
	sendData[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = true;
	sendData[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = true;
	sendData[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = true;
	sendData[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = true;
	sendData[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = true;
	sendData[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = true;
	sendData[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = true;
	sendData[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = true;
	sendData[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = true;
	sendData[(uint8_t)EmotiBit::DataType::DATA_CLIPPING] = true;
	sendData[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW] = true;
	sendData[(uint8_t)EmotiBit::DataType::DEBUG] = true;

	_newDataAvailable[(uint8_t)EmotiBit::DataType::EDA] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::EDL] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::EDR] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::PPG_RED] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::PPG_GREEN] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::THERMOPILE] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::DATA_CLIPPING] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW] = false;
	_newDataAvailable[(uint8_t)EmotiBit::DataType::DEBUG] = false;

	Serial.println("\nEmotiBit Setup complete");

	Serial.print("\nEmotiBit version: ");
	Serial.println(getHardwareVersion());
	Serial.print("Firmware version: ");
	Serial.println(firmware_version);
	Serial.println("Free Ram :" + String(freeMemory(), DEC) + " bytes");
	Serial.println("Electronic Serial Number:");
	Serial.print("Si7013_SNA),");
	Serial.print(tempHumiditySensor.sernum_a);
	Serial.print(", (Si7013_SNB), ");
	Serial.print(tempHumiditySensor.sernum_b);
	Serial.print("\n");


	uint8_t ledOffdelay = 100;	// Aesthetic delay
	led.setLED(uint8_t(EmotiBit::Led::RED), false);
	delay(ledOffdelay);
	led.setLED(uint8_t(EmotiBit::Led::BLUE), false);
	delay(ledOffdelay);
	led.setLED(uint8_t(EmotiBit::Led::YELLOW), false);

	if (!_sendTestData)
	{
		attachToInterruptTC3(&ReadSensors, this);
		Serial.println("Starting interrupts");
		startTimer(BASE_SAMPLING_FREQ);
	}

	// Debugging scope pins
	if (DIGITAL_WRITE_DEBUG)
	{
		pinMode(14, OUTPUT);
		digitalWrite(14, LOW);
		pinMode(16, OUTPUT);
		digitalWrite(16, LOW);
		pinMode(10, OUTPUT);
		digitalWrite(10, LOW);
	}

	if (testingMode == TestingMode::ACUTE)
	{
		Serial.println("TestingMode::ACUTE");
	}
	else if (testingMode == TestingMode::CHRONIC)
	{
		Serial.println("TestingMode::CHRONIC");
	}

	if (_debugMode) 
	{
		Serial.println("\nDEBUG MODE");
		Serial.println("Press ? to know more about available options in DEBUG MODE");
	}
} // Setup

void EmotiBit::setupSdCard()
{
	Serial.print("\nInitializing SD card...");
	// see if the card is present and can be initialized:
	bool success = false;
	for (int i = 0; i < 3; i++)
	{
		Serial.print(i);
		Serial.print(",");
		if (SD.begin(_sdCardChipSelectPin))
		{
			success = true;
			break;
		}
		delay(100);
	}
	if (!success) {
		Serial.print("...Card failed, or not present on chip select ");
		Serial.println(_sdCardChipSelectPin);
		// don't do anything more:
		// ToDo: Handle case where we still want to send network data
		while (true) {
			hibernate();
		}
	}
	Serial.println("card initialized.");
	SD.ls(LS_R);

	Serial.print(F("\nLoading configuration file: "));
	Serial.println(_configFilename);
	if (!loadConfigFile(_configFilename)) {
		Serial.println("SD card configuration file parsing failed.");
		Serial.println("Create a file 'config.txt' with the following JSON:");
		Serial.println("{\"WifiCredentials\": [{\"ssid\":\"SSSS\",\"password\" : \"PPPP\"}]}");
		while (true) {
			hibernate();
		}
	}

}

bool EmotiBit::addPacket(uint32_t timestamp, EmotiBit::DataType t, float * data, size_t dataLen, uint8_t precision) {
	static EmotiBitPacket::Header header;
	if (dataLen > 0) {
		// ToDo: Consider faster ways to populate the _outDataPackets

		//if (t == EmotiBit::DataType::DATA_OVERFLOW){
		//	addDebugPacket((uint8_t)EmotiBit::DebugTags::WIFI_CONNHISTORY, timestamp);  // addDebugPacket(case, timestamp) 
		//}

		header = EmotiBitPacket::createHeader(typeTags[(uint8_t)t], timestamp, _outDataPacketCounter++, dataLen);
		_outDataPackets += EmotiBitPacket::headerToString(header);
		for (uint16_t i = 0; i < dataLen; i++) {
			_outDataPackets += ",";
			if (t == EmotiBit::DataType::DATA_CLIPPING || t == EmotiBit::DataType::DATA_OVERFLOW) {
				// If it's a clipping/overflow type, write the data as a string rather than float
				// ToDo: consider how to better keep track of clipping and overflows
				_outDataPackets += typeTags[(uint8_t)data[i]];
			}
			else {
				_outDataPackets += String(data[i], precision);
			}
		}
		_outDataPackets += "\n";

		return true;
	}
	return false;
}

bool EmotiBit::addPacket(EmotiBit::DataType t) {
#ifdef DEBUG_GET_DATA
	Serial.print("addPacket: ");
	Serial.println((uint8_t)t);
#endif // DEBUG
	float * data;
	uint32_t timestamp;
	size_t dataLen;

	dataLen = getData(t, &data, &timestamp);

	if (dataLen > 0)
	{
		_newDataAvailable[(uint8_t)t] = true;	// set new data is available in the outputBuffer
	}

	if (sendData[(uint8_t)t]) {
		return addPacket(timestamp, t, data, dataLen, printLen[(uint8_t)t]);
	}
	return false;
}

void EmotiBit::parseIncomingControlPackets(String &controlPackets, uint16_t &packetNumber) {
	static String packet;
	static EmotiBitPacket::Header header;
	int16_t dataStartChar = 0;
	while (_emotiBitWiFi.readControl(packet) > 0)
	{
		Serial.println(packet);
		dataStartChar = EmotiBitPacket::getHeader(packet, header);
		if (dataStartChar > 0 || dataStartChar == EmotiBitPacket::NO_PACKET_DATA)
		{
			if (header.typeTag.equals(EmotiBitPacket::TypeTag::RECORD_BEGIN)) 
			{
				stopTimer();	// stop the data sampling timer
				String datetimeString = packet.substring(dataStartChar, packet.length());
				// Write the configuration info to json file
				String infoFilename = datetimeString + "_info.json";
				_dataFile = SD.open(infoFilename, FILE_WRITE);
				if (_dataFile) {
					if (!printConfigInfo(_dataFile, datetimeString)) {
						Serial.println(F("Failed to write to info file"));
					}
					_dataFile.close();
				}
				Serial.println("Creating new file to write data");
				// Try to open the data file to be sure we can write
				_sdCardFilename = datetimeString + ".csv";
				uint32_t start_timeOpenFile = millis();
				_dataFile = SD.open(_sdCardFilename, O_CREAT | O_WRITE | O_AT_END);
				if (_dataFile) {
					_sdWrite = true;
					Serial.print("** Recording Begin: ");
					Serial.print(_sdCardFilename);
					Serial.println(" **");
					// ToDo: need to communicate back to Visualizer if we were able to open a file
				}
				else {
					Serial.println("Failed to open data file for writing");
				}
				if (!_sendTestData)
				{
					startTimer(BASE_SAMPLING_FREQ); // start up the data sampling timer
				}
			}
			else if (header.typeTag.equals(EmotiBitPacket::TypeTag::RECORD_END)) { // Recording end
				if (_dataFile) {
					_dataFile.close();
				}
				_sdWrite = false;
				Serial.println("** Recording End **");
			}
			else if (header.typeTag.equals(EmotiBitPacket::TypeTag::MODE_NORMAL_POWER)) {
				setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
			}
			else if (header.typeTag.equals(EmotiBitPacket::TypeTag::MODE_LOW_POWER)) {
				setPowerMode(EmotiBit::PowerMode::LOW_POWER);
			}
			else if (header.typeTag.equals(EmotiBitPacket::TypeTag::MODE_MAX_LOW_POWER)) {
				setPowerMode(EmotiBit::PowerMode::MAX_LOW_POWER);
			}
			else if (header.typeTag.equals(EmotiBitPacket::TypeTag::MODE_WIRELESS_OFF)) {
				setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
			}
			else if (header.typeTag.equals(EmotiBitPacket::TypeTag::MODE_HIBERNATE)) {
				setPowerMode(EmotiBit::PowerMode::HIBERNATE);
			}
			else if (header.typeTag.equals(EmotiBitPacket::TypeTag::EMOTIBIT_DISCONNECT)) {
				_emotiBitWiFi.disconnect();
			}

			// Create a packet that can be logged with incrementing EmotiBit packet number
			controlPackets += EmotiBitPacket::createPacket(header.typeTag, packetNumber++, packet.substring(dataStartChar, packet.length()), header.dataLength);
		}
	}
}

void EmotiBit::updateButtonPress()
{
	uint16_t minShortButtonPress = 150;
	uint16_t minLongButtonPress = 3000;
	static uint32_t buttonPressedTimer = millis();
	static bool buttonPreviouslyPressed = false;

	// ToDo: create a mechanism

	buttonPressed = digitalRead(buttonPin);
	if (buttonPressed)
	{
		buttonPreviouslyPressed = true;

		if (millis() - buttonPressedTimer > minLongButtonPress)
		{
			Serial.print("onLongPress: ");
			Serial.println(millis() - buttonPressedTimer);
			// ToDo: Send BL packet
			(onLongPressCallback());
		}
	}
	else
	{
		if (buttonPreviouslyPressed) // Make sure button was actually pressed (not just a loop lag)
		{
			if (millis() - buttonPressedTimer > minShortButtonPress && millis() - buttonPressedTimer < minLongButtonPress)
			{
				Serial.print("onShortPress: ");
				Serial.println(millis() - buttonPressedTimer);
				// ToDo: Send BS packet
				(onShortPressCallback());
			}
		}
		buttonPreviouslyPressed = false;
		buttonPressedTimer = millis();	// reset the timer until the button is pressed
	}
}

uint8_t EmotiBit::update()
{



		static uint16_t serialPrevAvailable = Serial.available();
		if (Serial.available() > serialPrevAvailable)
		{
			// There's new data available on serial
			// Print to show we're alive
			//Serial.print(Serial.available());
			//Serial.print(':');
			//Serial.print(serialPrevAvailable);
			//Serial.print(">");
			//Serial.println(Serial.peek());
			Serial.println("hi");
		}
		serialPrevAvailable = Serial.available();

	if (_debugMode)
	{
		static String debugPackets;
		processDebugInputs(debugPackets, _outDataPacketCounter);
		_outDataPackets += debugPackets;
		debugPackets = "";
		if (_serialData != DataType::length)
		{
			writeSerialData(_serialData);
		}
	}



	// Handle updating WiFi connction + syncing
	static String inSyncPackets;
	_emotiBitWiFi.update(inSyncPackets, _outDataPacketCounter);
	_outDataPackets += inSyncPackets;
	inSyncPackets = "";
	
	// Process incoming controll packets
	static String inControlPackets;
	parseIncomingControlPackets(inControlPackets, _outDataPacketCounter);
	Serial.print(inControlPackets);
	_outDataPackets += inControlPackets;
	inControlPackets = "";


	
	// Update the button status and handle callbacks
	updateButtonPress();

	// Handle sending mode packets
	static uint32_t modePacketTimer = millis();
	if (millis() - modePacketTimer > modePacketInterval)
	{
		modePacketTimer = millis();
		String sentModePacket;
		sendModePacket(sentModePacket, _outDataPacketCounter);
	}
	if (edaCorrection.getMode() == EdaCorrection::Mode::UPDATE)
	{
		if (edaCorrection.progress == EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA || edaCorrection.progress == EdaCorrection::Progress::WAITING_USER_APPROVAL)
		{
			edaCorrection.readFloatFromSerial();
		}
		
	}
	else if (edaCorrection.getMode() == EdaCorrection::Mode::NORMAL)
	{
		// reading the OTP is done in the ISR
		// the correction values should be generated in the update function outside the ISR,
		// to reduce the load on the ISR
		// call calcCorrection
		if (edaCorrection.readOtpValues && !edaCorrection.calculationPerformed)
		{
			edaCorrection.calcEdaCorrection(_EmotiBit_i2c);
		}

		if (edaCorrection.triedRegOverwrite)
		{
			Serial.println("You are trying to overwrite a register, which is not allowed.");
			Serial.println("Please verify write operations");
			edaCorrection.triedRegOverwrite = false;// out of UPDATE mode, so this will not affect write Operations
		}

		if (edaCorrection.dummyDataReady)
		{
			vRef1 = edaCorrection.testVref1;
			vRef2 = edaCorrection.testVref2;
			edaFeedbackAmpR = edaCorrection.testRskin;
			Serial.print("updated emotibit class with these values");
			edaCorrection.dummyDataReady = false; // once the values are updated, we can set it to false to not enter this case again
		}

		
	}
	
	// Handle data buffer reading and sending
	static uint32_t dataSendTimer = millis();
	if (millis() - dataSendTimer > DATA_SEND_INTERVAL)
	{
		dataSendTimer = millis();



		if (_sendTestData)
		{
			appendTestData(_outDataPackets, _outDataPacketCounter);
			if (getPowerMode() == PowerMode::NORMAL_POWER)
			{
				_emotiBitWiFi.sendData(_outDataPackets);
			}
			writeSdCardMessage(_outDataPackets);
			_outDataPackets = "";
		}
		else
		{
			bool newData = false;

			if (DIGITAL_WRITE_DEBUG) digitalWrite(14, HIGH);

			for (int16_t i = 0; i < (uint8_t)EmotiBit::DataType::length; i++)
			{
				addPacket((EmotiBit::DataType) i);
				if (_outDataPackets.length() > OUT_MESSAGE_RESERVE_SIZE - OUT_PACKET_MAX_SIZE)
				{
					// Avoid overrunning our reserve memory
					if (DIGITAL_WRITE_DEBUG) digitalWrite(16, HIGH);

					if (getPowerMode() == PowerMode::NORMAL_POWER)
					{
						_emotiBitWiFi.sendData(_outDataPackets);
					}
					writeSdCardMessage(_outDataPackets);
					_outDataPackets = "";

					if (DIGITAL_WRITE_DEBUG) digitalWrite(16, LOW);
				}
			}
			if (_outDataPackets.length() > 0)
			{
				if (getPowerMode() == PowerMode::NORMAL_POWER)
				{
					_emotiBitWiFi.sendData(_outDataPackets);
				}
				writeSdCardMessage(_outDataPackets);
				_outDataPackets = "";
			}

			if (DIGITAL_WRITE_DEBUG) digitalWrite(14, LOW);

		}
	}

	// Hibernate after writing data
	if (getPowerMode() == PowerMode::HIBERNATE) {
		Serial.println("Hibernate");

		// Clear the remaining data buffer
		if (getPowerMode() == PowerMode::NORMAL_POWER)
		{
			_emotiBitWiFi.sendData(_outDataPackets);
		}
		writeSdCardMessage(_outDataPackets);
		_outDataPackets = "";

		hibernate();
	}
}

int8_t EmotiBit::updateEDAData() 
{
#ifdef DEBUG
	Serial.println("updateEDAData()");
#endif // DEBUG
	int8_t status = 0;
	static float edlTemp;	// Electrodermal Level in Volts
	static float edrTemp;	// Electrodermal Response in Volts
	static float edaTemp;	// Electrodermal Activity in Volts
  static float sclTemp;	// Skin Conductance Level in uSeimens
	static float scrTemp;	// Skin Conductance Response in uSeimens
	static bool edlClipped = false;
	static bool edrClipped = false;

	// ToDo: Optimize calculations for EDA

	// Check EDL and EDR voltages for saturation
	edlTemp = analogRead(_edlPin);
	edrTemp = analogRead(_edrPin);

	// Add data to buffer for sample averaging (oversampling)
	edlBuffer.push_back(edlTemp);
	edrBuffer.push_back(edrTemp);

	// ToDo: move adc clipping limits to setup()
	static const int adcClippingLowerLim = 20;
	static const int adcClippingUpperLim = adcRes - 20;

	// Check for data clipping
	if (edlTemp < adcClippingLowerLim	|| edlTemp > adcClippingUpperLim
		) 
	{
		edlClipped = true;
		status = status | (int8_t) Error::DATA_CLIPPING;
	}
	// Check for data clipping
	if (edrTemp < adcClippingLowerLim	|| edrTemp > adcClippingUpperLim
		) 
	{
		edrClipped = true;
		status = status | (int8_t)Error::DATA_CLIPPING;
	}
	
	if (edlBuffer.size() == _samplesAveraged.eda) {
		//static uint32_t timestamp;

		// Perform data averaging
		edlTemp = average(edlBuffer);
		edrTemp = average(edrBuffer);

		// Perform data conversion
		edlTemp = edlTemp * _vcc / adcRes;	// Convert ADC to Volts
		edrTemp = edrTemp * _vcc / adcRes;	// Convert ADC to Volts

		pushData(EmotiBit::DataType::EDL, edlTemp, &edlBuffer.timestamp);
		if (edlClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDL, &edlBuffer.timestamp);
		}

		pushData(EmotiBit::DataType::EDR, edrTemp, &edrBuffer.timestamp);
		if (edrClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDR, &edrBuffer.timestamp);
		}

		// EDL Digital Filter
		if (_edlDigFilteredVal < 0)
		{
			// initialize filter
			_edlDigFilteredVal = edlTemp;
		}
		_edlDigFilteredVal = edlTemp * (1. - _edlDigFiltAlpha) + _edlDigFilteredVal * _edlDigFiltAlpha;
		edlTemp = _edlDigFilteredVal;

		// Link to diff amp biasing: https://ocw.mit.edu/courses/media-arts-and-sciences/mas-836-sensor-technologies-for-interactive-environments-spring-2011/readings/MITMAS_836S11_read02_bias.pdf
		edaTemp = (edrTemp - vRef2) / edrAmplification;	// Remove VGND bias and amplification from EDR measurement
		edaTemp = edaTemp + edlTemp;                     // Add EDR to EDL in Volts

		//edaTemp = (_vcc - edaTemp) / edaVDivR * 1000000.f;						// Convert EDA voltage to uSeimens
		if (edaTemp - vRef1 < 0.000086f) // only track eda down to 1K Ohm
		{
			edaTemp = 0.001f; // Clamp the EDA measurement at 1K Ohm (0.001 Siemens)
		}
		else
		{
			edaTemp = vRef1 / (edaFeedbackAmpR * (edaTemp - vRef1)); // Conductance in Siemens
		}
		edaTemp = edaTemp * 1000000.f; // Convert to uSiemens


		// Add to data double buffer
		status = status | pushData(EmotiBit::DataType::EDA, edaTemp, &edrBuffer.timestamp);
		if (edlClipped || edrClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDA, &edrBuffer.timestamp);
		}

		// Clear the averaging buffers
		edlBuffer.clear();
		edrBuffer.clear();
		edlClipped = false;
		edrClipped = false;
	}

	// ToDo: Consider moving calculation into getData

	// EDA -> Iskin
	//tempEda = edaLow + (edaHigh / edaHPAmplification);
	//tempEda = (1.f - tempEda / _adcRes) * _vcc / _edaVDivR; // Iskin calculation
	////tempEda = tempEda * _edaVDivR / (_adcRes - _edaVDivR); // Rskin calculation
	//if (eda.push_back(tempEda) == BufferFloat::ERROR_BUFFER_OVERFLOW) {
	//	status = (int8_t)Error::BUFFER_OVERFLOW;
	//}

	////// EDL, EDR -> Rskin
	//tempEda = edaLow;
	////tempEda = tempEda * _edaVDivR / (_adcRes - _edaVDivR); // Rskin calculation
	//tempEda = tempEda * _vcc / _adcRes;
	//if (edl.push_back(tempEda) == BufferFloat::ERROR_BUFFER_OVERFLOW) {
	//	status = (int8_t)Error::BUFFER_OVERFLOW;
	//}
	//tempEda = edaHigh;
	////tempEda = tempEda * _edaVDivR / (_adcRes - _edaVDivR); // Rskin calculation
	//tempEda = tempEda * _vcc / _adcRes;
	//if (edr.push_back(tempEda) == BufferFloat::ERROR_BUFFER_OVERFLOW) {
	//	status = (int8_t)Error::BUFFER_OVERFLOW;
	//}

	return status;
}
int8_t EmotiBit::updatePPGData() {
#ifdef DEBUG
	Serial.println("updatePPGData()");
#endif // DEBUG

	int8_t status = 0;
	uint16_t numsamples = ppgSensor.check();

	while (ppgSensor.available()) {
		status = status | pushData(EmotiBit::DataType::PPG_INFRARED, ppgSensor.getFIFOIR());
		status = status | pushData(EmotiBit::DataType::PPG_RED, ppgSensor.getFIFORed());
		status = status | pushData(EmotiBit::DataType::PPG_GREEN, ppgSensor.getFIFOGreen());
		ppgSensor.nextSample();
		//available--;
	}
	return status;
}

int8_t EmotiBit::updateTempHumidityData() {
#ifdef DEBUG
	Serial.println("updateTempHumidityData()");
#endif // DEBUG
	_EmotiBit_i2c->setClock(100000);
	int8_t status = 0;
	if (tempHumiditySensor.getStatus() == Si7013::STATUS_IDLE) {
		if (tempHumiditySensor.isHumidityNew() == true) {
			humidityBuffer.push_back(tempHumiditySensor.getHumidity());
			if (humidityBuffer.size() >= _samplesAveraged.humidity) {
				status = status | pushData(EmotiBit::DataType::HUMIDITY_0, average(humidityBuffer), &(humidityBuffer.timestamp));
				humidityBuffer.clear();
			}
			temperatureBuffer.push_back(tempHumiditySensor.getPreviousTemperature());
			if (temperatureBuffer.size() >= _samplesAveraged.temperature) {
				status = status | pushData(EmotiBit::DataType::TEMPERATURE_0, average(temperatureBuffer), &(temperatureBuffer.timestamp));
				temperatureBuffer.clear();
			}
			tempHumiditySensor.startAdcMeasurement();
		}
		else {
			tempHumiditySensor.startHumidityTempMeasurement();
		}
		//else if (tempHumiditySensor.isAdcNew() == true) {
		//	static float thermTemp;
		//	thermTemp = tempHumiditySensor.getAdc();
		//	thermistorBuffer.push_back(thermTemp);
		//	if (thermistorBuffer.size() >= _samplesAveraged.thermistor) {
		//		// ToDo: Convert  to degrees
		//		//thermTemp = thermTemp * _vcc / _thermistorAdcResolution;	// Convert ADC to Volts
		//		//thermTemp = (thermTemp - vGnd) / thermistorAmplification;	// Remove VGND bias and amplification from thermistor measurement
		//		//thermTemp = thermTemp / (1 - thermTemp) / thermistorToDividerRatio;	// Convert Volts to Ohms
		//		// rDivVal
		//		// rThermVal
		//		// ohmToC

		//		// ToDo: Add clipping check

		//		// Steps:
		//		// Convert adc to volts: adcVal 
		//		// Remove VGND bias and amplification from EDR measurement
		//		// Convert to Ohms
		//		// Convert to degrees
		//		// B constant 3380
		//		// 20C = 12.09kOhm
		//		// 25C = 10KOhm
		//		// 30C = 8.31kOhm
		//		// Amplification 10x
		//		status = status | pushData(EmotiBit::DataType::TEMPERATURE_HP0, average(thermistorBuffer), &(thermistorBuffer.timestamp));
		//		thermistorBuffer.clear();
		//	}
		//	tempHumiditySensor.startHumidityTempMeasurement();
		//}
			
	}
	_EmotiBit_i2c->setClock(400000);
	return status;
}


int8_t EmotiBit::updateThermopileData() {
	// Thermopile
	int8_t status = 0;
	uint32_t timestamp;
	float AMB;// = 22991.97;  // raw data from thermopile needed for post processing
	float Sto;// = -88.44;  // raw data from thermopile needed for post processing
	MLX90632::status thermStatus;
	if (thermopileMode == MODE_STEP) {
		// Step mode manually triggers sensor reading
		if (!thermopileBegun) {
			// First time through step mode just starts a measurement
			MLX90632::status returnError;
			thermopile.startRawSensorValues(returnError);
			thermopileBegun = true;
		}
		else {

			thermStatus = MLX90632::status::SENSOR_NO_NEW_DATA;
			//while (thermStatus != MLX90632::status::SENSOR_SUCCESS)
			//{
			thermopile.getRawSensorValues(thermStatus, AMB, Sto); //Get the temperature of the object we're looking at in C
			timestamp = millis();
			//}
			status = status | therm0AMB.push_back(AMB, &(timestamp));
			// if DataOverflow
			if (status & BufferFloat::ERROR_BUFFER_OVERFLOW == BufferFloat::ERROR_BUFFER_OVERFLOW) 
			{
				// store the buffer overflow type and time
				dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW]->push_back((uint8_t)DataType::THERMOPILE, &timestamp);
			}
			status = status | therm0Sto.push_back(Sto, &(timestamp));
			thermopile.startRawSensorValues(thermStatus);
			return (int8_t)EmotiBit::Error::BUFFER_OVERFLOW;
		}
	}
	else if (thermopileMode == MODE_CONTINUOUS) {
		// Continuouts mode reads at the set rate and returns data if ready
		thermopile.getRawSensorValues(thermStatus, AMB, Sto);
		timestamp = millis();

		if (thermStatus == MLX90632::status::SENSOR_SUCCESS)
		{
			status = status | therm0AMB.push_back(AMB, &(timestamp));
			if (status & BufferFloat::ERROR_BUFFER_OVERFLOW == BufferFloat::ERROR_BUFFER_OVERFLOW)
			{
				// store the buffer overflow type and time
				dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW]->push_back((uint8_t)DataType::THERMOPILE, &timestamp);
			}
			status = status | therm0Sto.push_back(Sto, &(timestamp));
		}
	}
	
	return status;
}

int8_t EmotiBit::updateIMUData() {
#ifdef DEBUG
		Serial.println("updateIMUData()");
#endif // DEBUG
	static uint32_t timestamp;
	// ToDo: Add status return
	//static int8_t status;
	timestamp = millis();
	static int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
	static uint16_t rh;

	bool imuBufferFull = false;
	uint16_t nFrames = 1;
	if (_imuFifoFrameLen > 0) {
		// Using FIFO, get available frame count
		uint16_t nBytes = BMI160.getFIFOCount();
		nFrames = nBytes / _imuFifoFrameLen;
		if (nBytes > MAX_FIFO_BYTES - _imuFifoFrameLen*2) {
			// Possible data overflow on the IMU buffer
			// ToDo: assess IMU buffer overflow more accurately
			for (uint8_t j = (uint8_t)EmotiBit::DataType::ACCELEROMETER_X; j <= (uint8_t)EmotiBit::DataType::MAGNETOMETER_Z; j++) {
				// Note: this for loop usage relies on all IMU data types being grouped from AX to MZ
				dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW]->push_back(j, &timestamp);
				imuBufferFull = true;
			}
		}
		//Serial.print("FIFO Len: ");
		//Serial.print(nFrames);
		//Serial.print(" / ");
		//Serial.println(nBytes);
	}

	for (uint16_t j = 0; j < nFrames; j++) {
		if (_imuFifoFrameLen > 0) {
			// Using FIFO

			//Serial.print(",");
			//Serial.print(j);

			// Check for near-overflow of IMU double buffer and if so let data stay on IMU FIFO
			bool bufferMaxed = false;
			for (uint8_t k = (uint8_t)EmotiBit::DataType::ACCELEROMETER_X; k <= (uint8_t)EmotiBit::DataType::MAGNETOMETER_Z; k++) {
				// Note: this for loop usage relies on all IMU data types being grouped from AX to MZ
				if (dataDoubleBuffers[k]->inSize() == dataDoubleBuffers[k]->inCapacity()) {
					bufferMaxed = true;
				}
			}
			if (bufferMaxed && !imuBufferFull) {
				// data is about to overflow... leave it on the FIFO unless FIFO is also full
				break;
			}
			BMI160.getFIFOBytes(_imuBuffer, _imuFifoFrameLen);
			if (_imuFifoFrameLen == 20) {
				BMI160.extractMotion9(_imuBuffer, &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &rh);
			}
			else {
				Serial.println("UNHANDLED CASE: _imuFifoFrameLen != 20");
				while (true);
				// ToDo: Handle case when _imuFifoFrameLen < 20
			}
		}
		else {
			// Not using FIFO
			BMI160.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &rh);
			//BMI160.getMotion9Bosch(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &rh);
			//BMI160.getMotion9Check(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &rh);

			Serial.println("getMotion9");
		}

		// ToDo: Utilize IMU 1024 buffer to help mitigate buffer overruns

		checkIMUClipping(EmotiBit::DataType::ACCELEROMETER_X, ax, timestamp);
		checkIMUClipping(EmotiBit::DataType::ACCELEROMETER_Y, ay, timestamp);
		checkIMUClipping(EmotiBit::DataType::ACCELEROMETER_Z, az, timestamp);
		// Convert raw accelerometer to g's
		pushData(EmotiBit::DataType::ACCELEROMETER_X, convertRawAcc(ax), &timestamp);
		pushData(EmotiBit::DataType::ACCELEROMETER_Y, convertRawAcc(ay), &timestamp);
		pushData(EmotiBit::DataType::ACCELEROMETER_Z, convertRawAcc(az), &timestamp);

		checkIMUClipping(EmotiBit::DataType::GYROSCOPE_X, gx, timestamp);
		checkIMUClipping(EmotiBit::DataType::GYROSCOPE_Y, gy, timestamp);
		checkIMUClipping(EmotiBit::DataType::GYROSCOPE_Z, gz, timestamp);
		// convert the raw gyro data to degrees/second
		pushData(EmotiBit::DataType::GYROSCOPE_X, convertRawGyro(gx), &timestamp);
		pushData(EmotiBit::DataType::GYROSCOPE_Y, convertRawGyro(gy), &timestamp);
		pushData(EmotiBit::DataType::GYROSCOPE_Z, convertRawGyro(gz), &timestamp);

		// ToDo: determine correct magnetometer clipping
		pushData(EmotiBit::DataType::MAGNETOMETER_X, convertMagnetoX(mx, rh), &timestamp);
		pushData(EmotiBit::DataType::MAGNETOMETER_Y, convertMagnetoY(my, rh), &timestamp);
		pushData(EmotiBit::DataType::MAGNETOMETER_Z, convertMagnetoZ(mz, rh), &timestamp);
		if (bmm150XYClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::MAGNETOMETER_X, &timestamp);
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::MAGNETOMETER_Y, &timestamp);
			bmm150XYClipped = false;
		}
		if (bmm150ZHallClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::MAGNETOMETER_Z, &timestamp);
			bmm150ZHallClipped = false;
		}
	}
}

float EmotiBit::convertRawAcc(int16_t aRaw) {
	// ToDo: Precompute multiplier
	return ((float)aRaw * _accelerometerRange) / 32768.0f;
}

float EmotiBit::convertRawGyro(int16_t gRaw) {
	// Mapt to degrees/seconds

	// ToDo: Precompute multiplier
	return ((float)gRaw * _gyroRange) / 32768.0f;
}

int8_t EmotiBit::checkIMUClipping(EmotiBit::DataType type, int16_t data, uint32_t timestamp) {
	if (data == 32767 || data == -32768) {
		pushData(EmotiBit::DataType::DATA_CLIPPING, (int)type, &timestamp);
		return (int8_t) EmotiBit::Error::DATA_CLIPPING;
	} 
	else {
		return (int8_t)EmotiBit::Error::NONE;
	}
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer X axis data in float.
 *
 * @param[in] mag_data_x     : The value of raw X data
 * @param[in] data_rhall     : The value of raw RHALL data
 *
 * @return Result of compensated X data value in float'

 https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L331
 */
float EmotiBit::convertMagnetoX(int16_t mag_data_x, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_x0;
	float process_comp_x1;
	float process_comp_x2;
	float process_comp_x3;
	float process_comp_x4;

	/* Overflow condition check */
	if ((mag_data_x != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) &&
		(data_rhall != 0) && (bmm150TrimData.dig_xyz1 != 0)) {
		/*Processing compensation equations*/
		process_comp_x0 = (((float)bmm150TrimData.dig_xyz1) * 16384.0f / data_rhall);
		retval = (process_comp_x0 - 16384.0f);
		process_comp_x1 = ((float)bmm150TrimData.dig_xy2) * (retval * retval / 268435456.0f);
		process_comp_x2 = process_comp_x1 + retval * ((float)bmm150TrimData.dig_xy1) / 16384.0f;
		process_comp_x3 = ((float)bmm150TrimData.dig_x2) + 160.0f;
		process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
		retval = ((process_comp_x4 / 8192.0f) + (((float)bmm150TrimData.dig_x1) * 8.0f)) / 16.0f;
	}
	else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
		bmm150XYClipped = true;
	}

	return retval;
}
/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer y axis data(micro-tesla) in float.
 *
 * @param[in] mag_data_y     : The value of raw Y data
 * @param[in] data_rhall     : The value of raw RHALL data
 *
 * @return Result of compensated Y data value in float'
https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L1476
 */
float EmotiBit::convertMagnetoY(int16_t mag_data_y, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_y0;
	float process_comp_y1;
	float process_comp_y2;
	float process_comp_y3;
	float process_comp_y4;

	/* Overflow condition check */
	if ((mag_data_y != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL)
		&& (data_rhall != 0) && (bmm150TrimData.dig_xyz1 != 0)) {
		/*Processing compensation equations*/
		process_comp_y0 = ((float)bmm150TrimData.dig_xyz1) * 16384.0f / data_rhall;
		retval = process_comp_y0 - 16384.0f;
		process_comp_y1 = ((float)bmm150TrimData.dig_xy2) * (retval * retval / 268435456.0f);
		process_comp_y2 = process_comp_y1 + retval * ((float)bmm150TrimData.dig_xy1) / 16384.0f;
		process_comp_y3 = ((float)bmm150TrimData.dig_y2) + 160.0f;
		process_comp_y4 = mag_data_y * (((process_comp_y2)+256.0f) * process_comp_y3);
		retval = ((process_comp_y4 / 8192.0f) + (((float)bmm150TrimData.dig_y1) * 8.0f)) / 16.0f;
	}
	else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
		bmm150XYClipped = true;
	}

	return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer z axis data(micro-tesla) in float.
 *
 * @param[in] mag_data_z     : The value of raw Z data
 * @param[in] data_rhall     : The value of raw RHALL data
 *
 * @return Result of compensated Z data value in float'
 https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L1508
 */
float EmotiBit::convertMagnetoZ(int16_t mag_data_z, uint16_t data_rhall)
{
	float retval = 0;
	float process_comp_z0;
	float process_comp_z1;
	float process_comp_z2;
	float process_comp_z3;
	float process_comp_z4;
	float process_comp_z5;

	/* Overflow condition check */
	if ((mag_data_z != BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL) &&
		(bmm150TrimData.dig_z2 != 0) && (bmm150TrimData.dig_z1 != 0)
		&& (bmm150TrimData.dig_xyz1 != 0) && (data_rhall != 0)) {
		/* Processing compensation equations */
		process_comp_z0 = ((float)mag_data_z) - ((float)bmm150TrimData.dig_z4);
		process_comp_z1 = ((float)data_rhall) - ((float)bmm150TrimData.dig_xyz1);
		process_comp_z2 = (((float)bmm150TrimData.dig_z3) * process_comp_z1);
		process_comp_z3 = ((float)bmm150TrimData.dig_z1) * ((float)data_rhall) / 32768.0f;
		process_comp_z4 = ((float)bmm150TrimData.dig_z2) + process_comp_z3;
		process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
		retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;
	}
	else {
		/* overflow, set output to 0.0f */
		retval = BMM150_OVERFLOW_OUTPUT_FLOAT;
		bmm150ZHallClipped = true;
	}

	return retval;
}

int8_t EmotiBit::pushData(EmotiBit::DataType type, float data, uint32_t * timestamp) {
	if ((uint8_t)type < (uint8_t)EmotiBit::DataType::length) {
		if (timestamp == nullptr) {
			*timestamp = millis();
		}
		uint8_t status = dataDoubleBuffers[(uint8_t)type]->push_back(data, timestamp);
		if (status & BufferFloat::ERROR_BUFFER_OVERFLOW == BufferFloat::ERROR_BUFFER_OVERFLOW) {
			// store the buffer overflow type and time
			dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW]->push_back((uint8_t)type, timestamp);
			return (int8_t)EmotiBit::Error::BUFFER_OVERFLOW;
		}
		//else if (status & BufferFloat::PUSH_WHILE_GETTING == BufferFloat::PUSH_WHILE_GETTING) {
		//	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PUSH_WHILE_GETTING]->push_back((uint8_t)type, timestamp);
		//	return (int8_t)EmotiBit::Error::NONE;
		//}
		else {
			return (int8_t)EmotiBit::Error::NONE;
		}
	} 
	else {
		return (int8_t)EmotiBit::Error::INDEX_OUT_OF_BOUNDS;
	}
}

size_t EmotiBit::getDataThermopile(float **data, uint32_t *timestamp)
{
	size_t sizeAMB;
	size_t sizeSto;
	float* dataAMB;
	float* dataSto;
	uint32_t* timestampSto;
	sizeAMB = therm0AMB.getData(&dataAMB, timestamp);
	sizeSto = therm0Sto.getData(&dataSto, timestampSto);
	if (sizeAMB != sizeSto) // interrupt hit between therm0AMB.getdata and therm0Sto.getdata
	{
		// sizeAMB = k
		// SizeSto = k+s ; where s= #sampepls added per interrupt
		for (uint8_t i = sizeAMB; i < sizeSto; i++)
		{
			therm0Sto.push_back(dataSto[i], timestampSto);
		}
		sizeSto = sizeAMB;
	}
	else
	{
		for (uint8_t i = 0; i < sizeAMB; i++)
		{
			// if dummy data was stored
			if (dataAMB[i] == -2 && dataSto[i] == -2)
			{
				return dataDoubleBuffers[(uint8_t)EmotiBit::DataType::THERMOPILE]->getData(data, timestamp);
			}
			float objectTemp;
			// toggle to true to create forced nan values on the thermopile data
			bool createNan = false;
			if (createNan)
			{
				// if createNan is true generate nan values with a 0.1 probablity. This is to test the MLX90632 library functionatlity that prevents one nan to recursively create only nan values.
				int randNum = rand() % 10 + 1;
				if (randNum <= 1)
				{
					// getObjectTemp(-3, -3) generates nan Temp value.
					objectTemp = thermopile.getObjectTemp(-3, -3);
					Serial.print("AMB for nan: -2");
					Serial.println("\t Sto for nan: -2");
				}
				else
				{
					objectTemp = thermopile.getObjectTemp(dataAMB[i], dataSto[i]);
				}
			}
			else
			{
				objectTemp = thermopile.getObjectTemp(dataAMB[i], dataSto[i]);
			}

			if (isnan(objectTemp))
			{
				static String debugPacket;
				static String payloadAMB;
				static String payloadSto;
				payloadAMB = "AMB val for nan:";
				payloadSto = "Sto val fro nan:";
				if (createNan)
				{
					payloadAMB += "-2";
					payloadSto += "-2";
				}
				else
				{
					payloadAMB += String(dataAMB[i], 4);
					payloadSto += String(dataSto[i], 4);
				}
				debugPacket += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, _outDataPacketCounter++, payloadAMB, 1);
				debugPacket += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, _outDataPacketCounter++, payloadSto, 1);
				_outDataPackets += debugPacket;
				debugPacket = "";
				payloadAMB = "";
				payloadSto = "";
			}
			pushData(EmotiBit::DataType::THERMOPILE, objectTemp, timestamp);
		}
		return dataDoubleBuffers[(uint8_t)EmotiBit::DataType::THERMOPILE]->getData(data, timestamp);
	}
	
}


size_t EmotiBit::getData(DataType type, float** data, uint32_t * timestamp) {
#ifdef DEBUG
	Serial.print("getData: type=");
	Serial.println((uint8_t) t);
#endif // DEBUG
	if ((uint8_t)type < (uint8_t)EmotiBit::DataType::length) {
		
		if ((uint8_t)type == (uint8_t)EmotiBit::DataType::THERMOPILE)
		{
			return getDataThermopile(data, timestamp);
		}
		else
		{
			return dataDoubleBuffers[(uint8_t)type]->getData(data, timestamp);
		}
	}
	else {
		return (int8_t)EmotiBit::Error::NONE;
	}
}

int8_t EmotiBit::updateBatteryVoltageData() {
	batteryVoltageBuffer.push_back(readBatteryVoltage());
	if (batteryVoltageBuffer.size() >= _samplesAveraged.battery) {
		batteryVoltage.push_back(average(batteryVoltageBuffer));
		batteryVoltageBuffer.clear();
	}
}


int8_t EmotiBit::updateBatteryPercentData() {
	batteryPercentBuffer.push_back(readBatteryPercent());
	if (batteryPercentBuffer.size() >= _samplesAveraged.battery) {
		batteryPercent.push_back(average(batteryPercentBuffer));
		batteryPercentBuffer.clear();
	}
}

float EmotiBit::readBatteryVoltage() {
	float batRead = analogRead(_batteryReadPin);
	//float batRead = 10000.f;
	batRead *= 2.f;
	batRead *= _vcc;
	batRead /= adcRes; // ToDo: precalculate multiplier
	return batRead;
}

int8_t EmotiBit::readBatteryPercent() {
	// Thresholded bi-linear approximation
	// See battery discharge profile here:
	// https://www.richtek.com/Design%20Support/Technical%20Document/AN024
	float bv = readBatteryVoltage();
	if (bv > 4.15f) {
		return 100;
	}
	else if (bv > 3.65f) {
		float temp;
		temp = (bv - 3.65f);
		temp /= (4.15f - 3.65f);
		temp *= 93.f;
		temp += 7.f;
		return (int8_t)temp;
	}
	else if (bv > 3.3f) {
		float temp;
		temp = (bv - 3.3f);
		temp /= (4.65f - 3.3f);
		temp *= 7.f;
		temp += 0.f;
		return (int8_t)temp;
	}
	//else if (bv > 4.1f) {
	//	return 95;
	//}
	//else if (bv > 4.f) {
	//	return 85;
	//}
	//else if (bv > 3.95f) {
	//	return 80;
	//}
	//else if (bv > 3.9f) {
	//	return 75;
	//}
	//else if (bv > 3.85f) {
	//	return 70;
	//}
	//else if (bv > 3.8f) {
	//	return 60;
	//}
	//else if (bv > 3.75f) {
	//	return 50;
	//}
	//else if (bv > 3.725f) {
	//	return 40;
	//}
	//else if (bv > 3.7f) {
	//	return 30;
	//}
	//else if (bv > 3.675f) {
	//	return 17;
	//}
	//else if (bv >3.6f) {
	//	return 5;
	//}
	//else if (bv > 3.5) {
	//	return 2;
	//}
	//else return 1;
	//else if (temp > 3.45) {
	//	return 1;
	//}
	//else if (temp > 3.4) {
	//	return 0;
	//}
}

bool EmotiBit::setSensorTimer(SensorTimer t) {
	_sensorTimer = t;
	// ToDo: add timer handling
}



bool EmotiBit::printConfigInfo(File &file, const String &datetimeString) {
#ifdef DEBUG
	Serial.println("printConfigInfo");
#endif
	//bool EmotiBit::printConfigInfo(File file, String datetimeString) {
	String source_id = "EmotiBit FeatherWing";
	String hardware_version = getHardwareVersion();
	String feather_version = "Adafruit Feather M0 WiFi";

	const uint16_t bufferSize = 1024;

	// Open file for writing
	//File file = SD.open("TEST.TXT", FILE_WRITE);
	if (!file) {
#ifdef DEBUG
		Serial.println(F("Failed to create file"));
#endif
		return false;
	}

	file.print("["); // Doing some manual printing to chunk JSON and save RAM

	{
		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		//JsonArray& root = jsonBuffer.createArray();
		const uint8_t nInfo = 1;
		//JsonObject* indices[nInfo];
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));

		// ToDo: Use EmotiBitPacket::TypeTag rather than fallable constants to set typeTags

		// Accelerometer
		//indices[i] = &(root.createNestedObject());
		//infos[i] = &(indices[i]->createNestedObject("info"));
		infos[i]->set("name", "Accelerometer");
		infos[i]->set("type", "Accelerometer");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("AX");
		typeTags[i]->add("AY");
		typeTags[i]->add("AZ");
		infos[i]->set("channel_count", 3);
		infos[i]->set("nominal_srate", _samplingRates.accelerometer);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "g");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("range", _accelerometerRange);
		setups[i]->set("acc_bwp", imuSettings.acc_bwp);
		setups[i]->set("acc_us", imuSettings.acc_us);

		if (root.printTo(file) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

	file.print(","); // Doing some manual printing to chunk JSON and save RAM

	{
		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		//JsonArray& root = jsonBuffer.createArray();
		const uint8_t nInfo = 1;
		//JsonObject* indices[nInfo];
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));

		// Gyroscope
		//i++;
		//indices[i] = &(root.createNestedObject());
		//infos[i] = &(indices[i]->createNestedObject("info"));
		infos[i]->set("name", "Gyroscope");
		infos[i]->set("type", "Gyroscope");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("GX");
		typeTags[i]->add("GY");
		typeTags[i]->add("GZ");
		infos[i]->set("channel_count", 3);
		infos[i]->set("nominal_srate", _samplingRates.gyroscope);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "degrees/second");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("range", _gyroRange);
		setups[i]->set("gyr_bwp", imuSettings.gyr_bwp);
		setups[i]->set("gyr_us", imuSettings.gyr_us);
		if (root.printTo(file) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

	file.print(","); // Doing some manual printing to chunk JSON and save RAM

	{
		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		//JsonArray& root = jsonBuffer.createArray();
		const uint8_t nInfo = 1;
		//JsonObject* indices[nInfo];
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));

		// Magnetometer
		//i++;
		//indices[i] = &(root.createNestedObject());
		//infos[i] = &(indices[i]->createNestedObject("info"));
		infos[i]->set("name", "Magnetometer");
		infos[i]->set("type", "Magnetometer");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("MX");
		typeTags[i]->add("MY");
		typeTags[i]->add("MZ");
		infos[i]->set("channel_count", 3);
		infos[i]->set("nominal_srate", _samplingRates.magnetometer);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "microhenries");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		if (root.printTo(file) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

	file.print(","); // Doing some manual printing to chunk JSON and save RAM

	{
		// EDA

		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		const uint8_t nInfo = 1;
		//JsonObject* indices[nInfo];
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));
		//i++;
		infos[i]->set("name", "ElectrodermalActivity");
		infos[i]->set("type", "ElectrodermalActivity");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("EA");
		infos[i]->set("channel_count", 1);
		infos[i]->set("nominal_srate", _samplingRates.eda / _samplesAveraged.eda);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "microsiemens");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("adc_bits", _adcBits);
		setups[i]->set("voltage_reference_1", vRef1);
		setups[i]->set("voltage_reference_2", vRef2);
		setups[i]->set("EDA_feedback_amp_resistance", edaFeedbackAmpR);
		setups[i]->set("EDR_amplification", edrAmplification);
		setups[i]->set("EDL_digital_filter_alpha", _edlDigFiltAlpha);
		setups[i]->set("EDA_crossover_filter_frequency", edaCrossoverFilterFreq);
		setups[i]->set("samples_averaged", _samplesAveraged.eda);
		setups[i]->set("oversampling_rate", _samplingRates.eda);		
		if (root.printTo(file) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

	file.print(","); // Doing some manual printing to chunk JSON and save RAM

	{
		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		//JsonArray& root = jsonBuffer.createArray();
		const uint8_t nInfo = 1;
		//JsonObject* indices[nInfo];
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));
		// Humidity0
		//i++;
		//indices[i] = &(root.createNestedObject());
		//infos[i] = &(indices[i]->createNestedObject("info"));
		infos[i]->set("name", "Humidity0");
		infos[i]->set("type", "Humidity");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("H0");
		infos[i]->set("channel_count", 1);
		infos[i]->set("nominal_srate", _samplingRates.humidity / _samplesAveraged.humidity);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "Percent");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("resolution", "RESOLUTION_H11_T11");
		setups[i]->set("samples_averaged", _samplesAveraged.humidity);
		setups[i]->set("oversampling_rate", _samplingRates.humidity);
		if (root.printTo(file) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

	file.print(","); // Doing some manual printing to chunk JSON and save RAM

	{
		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		//JsonArray& root = jsonBuffer.createArray();
		const uint8_t nInfo = 1;
		//JsonObject* indices[nInfo];
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));
		// Temperature0
		//i++;
		//indices[i] = &(root.createNestedObject());
		//infos[i] = &(indices[i]->createNestedObject("info"));
		infos[i]->set("name", "Temperature0");
		infos[i]->set("type", "Temperature");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("T0");
		infos[i]->set("channel_count", 1);
		infos[i]->set("nominal_srate", _samplingRates.temperature / _samplesAveraged.temperature);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "degrees celcius");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("sensor_part_number", "Si7013");
		infos[i]->set("sensor_serial_number_a", tempHumiditySensor.sernum_a);
		infos[i]->set("sensor_serial_number_b", tempHumiditySensor.sernum_b);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("resolution", "RESOLUTION_H11_T11");
		setups[i]->set("samples_averaged", _samplesAveraged.temperature);
		setups[i]->set("oversampling_rate", _samplingRates.temperature);
		if (root.printTo(file) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

	file.print(","); // Doing some manual printing to chunk JSON and save RAM

	{
		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		//JsonArray& root = jsonBuffer.createArray();
		const uint8_t nInfo = 1;
		//JsonObject* indices[nInfo];
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));

		// thermopile
		//i++;
		//indices[i] = &(root.createNestedObject());
		//infos[i] = &(indices[i]->createNestedObject("info"));
		infos[i]->set("name", "Thermopile");
		infos[i]->set("type", "Temperature");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("TH");
		infos[i]->set("channel_count", 1);
		if (thermopileMode == MODE_CONTINUOUS)
		{
			infos[i]->set("nominal_srate", thermopileFs);
		}
		else
		{
			infos[i]->set("nominal_srate", _samplingRates.thermopile / _samplesAveraged.thermopile);
		}
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "degrees celcius");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("samples_averaged", _samplesAveraged.thermopile);
		if (thermopileMode == MODE_CONTINUOUS)
		{
			infos[i]->set("oversampling_rate", thermopileFs);
		}
		else
		{
			setups[i]->set("oversampling_rate", _samplingRates.thermopile);
		}
		if (root.printTo(file) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

	file.print(","); // Doing some manual printing to chunk JSON and save RAM

	{
		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		//JsonArray& root = jsonBuffer.createArray();
		const uint8_t nInfo = 1;
		//JsonObject* indices[nInfo];
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));

		// PPG
		//i++;
		//indices[i] = &(root.createNestedObject());
		//infos[i] = &(indices[i]->createNestedObject("info"));
		infos[i]->set("name", "PPG");
		infos[i]->set("type", "PPG");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("PI");
		typeTags[i]->add("PR");
		typeTags[i]->add("PG");
		infos[i]->set("channel_count", 3);
		infos[i]->set("nominal_srate", ppgSettings.sampleRate / ppgSettings.sampleAverage);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "raw units");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("LED_power_level", ppgSettings.ledPowerLevel);
		setups[i]->set("samples_averaged", ppgSettings.sampleAverage);
		setups[i]->set("LED_mode", ppgSettings.ledMode);
		setups[i]->set("oversampling_rate", ppgSettings.sampleRate);
		setups[i]->set("pulse_width", ppgSettings.pulseWidth);
		setups[i]->set("ADC_range", ppgSettings.adcRange);
		if (root.printTo(file) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

	file.print("]"); // Doing some manual printing to chunk JSON and save RAM

	return true;
}

float EmotiBit::average(BufferFloat &b) {
	float f = 0;
	for (int i = 0; i < b.size(); i++) {
		f += b.data[i];
	}
	f /= b.size();
	return f;
}

void EmotiBit::scopeTimingTest() {
	pinMode(SCOPE_TEST_PIN, OUTPUT);
	if (scopeTestPinOn) {
		digitalWrite(SCOPE_TEST_PIN, LOW);
		scopeTestPinOn = false;
	}
	else {
		digitalWrite(SCOPE_TEST_PIN, HIGH);
		scopeTestPinOn = true;
	}
}

// Function to attach callback to short press
void EmotiBit::attachShortButtonPress(void(*shortButtonPressFunction)(void))
{
	onShortPressCallback = shortButtonPressFunction;
}

// Function to attach callback to long press
void EmotiBit::attachLongButtonPress(void(*longButtonPressFunction)(void)) {
	onLongPressCallback = longButtonPressFunction;
}

void EmotiBit::readSensors() 
{
#ifdef DEBUG_GET_DATA
	Serial.println("readSensors()");
#endif // DEBUG
	if (DIGITAL_WRITE_DEBUG) digitalWrite(10, HIGH);
	
	uint32_t readSensorsBegin = micros();
	if (edaCorrection.getMode() == EdaCorrection::Mode::UPDATE)
	{
		if (edaCorrection.progress == EdaCorrection::Progress::WRITING_TO_OTP)
		{
			
			edaCorrection.writeToOtp(_EmotiBit_i2c);
			
		}
	}
	else if (edaCorrection.getMode() == EdaCorrection::Mode::NORMAL && !edaCorrection.readOtpValues)
	{
		
		edaCorrection.readFromOtp(_EmotiBit_i2c);
		
	}

	// Battery (all analog reads must be in the ISR)
	// TODO: use the stored/averaged Battery value instead of calling readBatteryPercent again
	if (acquireData.battery)
	{
		static uint16_t batteryCounter = timerLoopOffset.battery;
		if (batteryCounter == BATTERY_SAMPLING_DIV) {
			battLevel = readBatteryPercent();
			updateBatteryIndication();
			updateBatteryPercentData();
			batteryCounter = 0;
		}
		batteryCounter++;
	}
	
	if (dummyIsrWithDelay)
	{
		// Generate dummy data to test ISR without I2C

		static uint16_t dummyCounter = 0;
		if (dummyCounter == DUMMY_ISR_DIV) {
			static int dummyData = 0;
			for (uint8_t t = (uint8_t)DataType::PPG_INFRARED; t < (uint8_t)DataType::BATTERY_VOLTAGE; t++)
			{
				if (t == (uint8_t)DataType::THERMOPILE)
				{
					pushData((DataType)t, (float)dummyData);
					therm0AMB.push_back(-2);
					therm0Sto.push_back(-2);
				}
				else
				{
					pushData((DataType)t, (float)dummyData);
				}
			}
			dummyData++;
			if (dummyData >= 25) dummyData = 0;
			dummyCounter = 0;
		}
		dummyCounter++;
		// Delay a bit more than half the sampling period (2 msec for 300Hz)
		float delay = 1000.f / BASE_SAMPLING_FREQ / 2.f;
		delay = ceil(delay) * 1000;
		delayMicroseconds(delay);
	}
	else
	{

		// EDA
		if (acquireData.eda) {
			static uint16_t edaCounter = timerLoopOffset.eda;
			if (edaCounter == EDA_SAMPLING_DIV) {
				int8_t tempStatus = updateEDAData();
				edaCounter = 0;
			}
			edaCounter++;
		}

		// Temperature / Humidity Sensor
		if (chipBegun.SI7013 && acquireData.tempHumidity) {
			static uint16_t temperatureCounter = timerLoopOffset.tempHumidity;
			if (temperatureCounter == TEMPERATURE_SAMPLING_DIV) {
				// Note: Temperature/humidity and the thermistor are alternately sampled 
				// on every other call of updateTempHumidityData()
				// I.e. you must call updateTempHumidityData() 2x with a sufficient measurement 
				// delay between calls to sample both temperature/humidity and the thermistor
				int8_t tempStatus = updateTempHumidityData();
				//if (dataStatus.tempHumidity == 0) {
				//	dataStatus.tempHumidity = tempStatus;
				//}
				temperatureCounter = 0;
			}
			temperatureCounter++;
		}

		// Thermopile
		if (chipBegun.MLX90632 && acquireData.thermopile) {
			static uint16_t thermopileCounter = timerLoopOffset.thermopile;
			if (thermopileCounter == THERMOPILE_SAMPLING_DIV) {
				int8_t tempStatus = updateThermopileData();
				thermopileCounter = 0;
			}
			thermopileCounter++;
		}

		// PPG
		if (chipBegun.MAX30101 && acquireData.ppg) {
			static uint16_t ppgCounter = timerLoopOffset.ppg;
			if (ppgCounter == PPG_SAMPLING_DIV) {
				int8_t tempStatus = updatePPGData();
				ppgCounter = 0;
			}
			ppgCounter++;
		}

		// IMU
		if (chipBegun.BMI160 && chipBegun.BMM150 && acquireData.imu) {
			static uint16_t imuCounter = timerLoopOffset.imu;
			if (imuCounter == IMU_SAMPLING_DIV) {
				int8_t tempStatus = updateIMUData();
				imuCounter = 0;
			}
			imuCounter++;
		}

		// LED STATUS CHANGE SEGMENT
		if (chipBegun.NCP5623)
		{
			static uint16_t ledCounter = timerLoopOffset.led;
			if (ledCounter == LED_REFRESH_DIV)
			{
				ledCounter = 0;

				if (buttonPressed)
				{
					// Turn on the LEDs when the button is pressed
					led.setLED(uint8_t(EmotiBit::Led::RED), true);
					led.setLED(uint8_t(EmotiBit::Led::BLUE), true);
					led.setLED(uint8_t(EmotiBit::Led::YELLOW), true);
				}
				else
				{

					// WiFi connected status LED
					if (_emotiBitWiFi.isConnected())
					{
						led.setLED(uint8_t(EmotiBit::Led::BLUE), true);
					}
					else
					{
						led.setLED(uint8_t(EmotiBit::Led::BLUE), false);
					}

					// Battery LED
					if (battIndicationSeq)
					{
						led.setLED(uint8_t(EmotiBit::Led::YELLOW), true);
					}
					else
					{
						led.setLED(uint8_t(EmotiBit::Led::YELLOW), false);
					}

					// Recording status LED
					if (_sdWrite)
					{
						static uint32_t recordBlinkDuration = millis();
						if (millis() - recordBlinkDuration >= 500)
						{
							led.setLED(uint8_t(EmotiBit::Led::RED), !led.getLED(uint8_t(EmotiBit::Led::RED)));
							recordBlinkDuration = millis();
						}
					}
					else if (!_sdWrite && led.getLED(uint8_t(EmotiBit::Led::RED)) == true)
					{
						led.setLED(uint8_t(EmotiBit::Led::RED), false);
					}
				}
			}
			ledCounter++;
		}
	}
	
	if (acquireData.debug) pushData(EmotiBit::DataType::DEBUG, micros() - readSensorsBegin); // Add readSensors processing duration to debugBuffer

	if (DIGITAL_WRITE_DEBUG) digitalWrite(10, LOW);
}

String EmotiBit::getHardwareVersion()
{
	if (_version == Version::V02H) {
		return "V02h";
	}
}



#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int EmotiBit::freeMemory() {
	char top;
#ifdef __arm__
	return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
	return &top - __brkval;
#else  // __arm__
	return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}


// NEW Hibernate
void EmotiBit::hibernate() {
	Serial.println("hibernate()");
	Serial.println("Stopping timer...");
	stopTimer();

	// Delay to ensure the timer has finished
	delay((1000 * 5) / BASE_SAMPLING_FREQ);

	// Turn on all the LEDs to indicate hibernate started
	led.setLED(uint8_t(EmotiBit::Led::RED), true);
	led.setLED(uint8_t(EmotiBit::Led::BLUE), true);
	led.setLED(uint8_t(EmotiBit::Led::YELLOW), true);

	chipBegun.MAX30101 = false;
	chipBegun.BMM150 = false;
	chipBegun.BMI160 = false;
	chipBegun.SI7013 = false;
	chipBegun.NCP5623 = false;
	chipBegun.MLX90632 = false;

	Serial.println("Shutting down WiFi...");
	_emotiBitWiFi.end();

	Serial.println("Shutting down serial interfaces...");
	SPI.end(); 
	Wire.end();
	_EmotiBit_i2c->end();

	if (_EmotiBit_i2c != nullptr)
	{
		delete(_EmotiBit_i2c);
	}
	
	// Setup all pins (digital and analog) in INPUT mode (default is nothing)  
	for (uint32_t ul = 0; ul < PINS_COUNT; ul++)
	{
		if (ul != _hibernatePin) {
			pinMode(ul, OUTPUT);
			digitalWrite(ul, LOW);
			pinMode(ul, INPUT);
			Serial.print("Turning off pin: ");
			Serial.println(ul);
		}
	}

	// Turn off EmotiBit power
	Serial.println("Disabling EmotiBit power");
	pinMode(_hibernatePin, OUTPUT);
	digitalWrite(_hibernatePin, HIGH);
	pinMode(_hibernatePin, INPUT);

	//deepSleep();
	Serial.println("Entering deep sleep...");
	LowPower.deepSleep();
}


// Loads the configuration from a file
bool EmotiBit::loadConfigFile(const String &filename) {
	// Open file for reading
	File file = SD.open(filename);

	if (!file) {
		Serial.print("File ");
		Serial.print(filename);
		Serial.println(" not found");
		return false;
	}

	Serial.print("Parsing: ");
	Serial.println(filename);

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
		_emotiBitWiFi.addCredential( ssid, pass);
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


bool EmotiBit::writeSdCardMessage(const String & s) {
	// Break up the message in to bite-size chunks to avoid over running the UDP or SD card write buffers
	// UDP buffer seems to be about 1400 char. SD card writes should be 512 char.

	if (_sdWrite && s.length() > 0) {
		if (_dataFile) {
			static int16_t firstIndex;
			firstIndex = 0;
			while (firstIndex < s.length()) {
				static int16_t lastIndex;
				if (s.length() - firstIndex > MAX_SD_WRITE_LEN) {
					lastIndex = firstIndex + MAX_SD_WRITE_LEN;
				}
				else {
					lastIndex = s.length();
				}
#ifdef DEBUG_GET_DATA
				Serial.println("writing to SD card");
#endif // DEBUG
				_dataFile.print(s.substring(firstIndex, lastIndex));
				firstIndex = lastIndex;
			}

			static uint32_t syncTimer = millis();
			if (millis() - syncTimer > targetFileSyncDelay)
			{
				syncTimer = millis();
				_dataFile.sync();
			}
		}
		else {
			Serial.print("Data file didn't open properly: ");
			Serial.println(_sdCardFilename);
			_sdWrite = false;
		}
	}
}

EmotiBit::PowerMode EmotiBit::getPowerMode()
{
	return _powerMode;
}

void EmotiBit::setPowerMode(PowerMode mode)
{
	_powerMode = mode;
	String modePacket;
	sendModePacket(modePacket, _outDataPacketCounter);
	if (getPowerMode() == PowerMode::NORMAL_POWER)
	{
		Serial.println("PowerMode::NORMAL_POWER");
		if (_emotiBitWiFi.isOff())
		{
			_emotiBitWiFi.begin(100, 100);	// ToDo: create a async begin option
		}
		WiFi.lowPowerMode();
		modePacketInterval = NORMAL_POWER_MODE_PACKET_INTERVAL;
	}
	else if (getPowerMode() == PowerMode::LOW_POWER)
	{
		Serial.println("PowerMode::LOW_POWER");
		if (_emotiBitWiFi.isOff())
		{
			_emotiBitWiFi.begin(100, 100);	// ToDo: create a async begin option
		}
		WiFi.lowPowerMode();
		modePacketInterval = LOW_POWER_MODE_PACKET_INTERVAL;
	}
	else if (getPowerMode() == PowerMode::MAX_LOW_POWER)
	{
		Serial.println("PowerMode::MAX_LOW_POWER");
		if (_emotiBitWiFi.isOff())
		{
			_emotiBitWiFi.begin(100, 100);	// ToDo: create a async begin option
		}
		WiFi.maxLowPowerMode();
		modePacketInterval = LOW_POWER_MODE_PACKET_INTERVAL;
	}
	else if (getPowerMode() == PowerMode::WIRELESS_OFF)
	{
		Serial.println("PowerMode::WIRELESS_OFF");
		_emotiBitWiFi.end();
	}
	else if (getPowerMode() == PowerMode::HIBERNATE)
	{
		Serial.println("PowerMode::HIBERNATE");
	}
	else
	{
		Serial.println("PowerMode Not Recognized");
	}
}

void EmotiBit::writeSerialData(EmotiBit::DataType t)
{
	float * data;
	uint32_t timestamp;
	size_t dataAvailable = dataDoubleBuffers[(uint8_t)t]->getData(&data, &timestamp, false);	// read data without swapping buffers
	for (size_t i = 0; i < dataAvailable; i++)
	{
		//Serial.print((uint8_t)t);
		//Serial.print(", ");
		//Serial.print(dataAvailable);
		//Serial.print(", ");
		Serial.println(String(data[i], printLen[(uint8_t)t]));
	}
}


size_t EmotiBit::readData(EmotiBit::DataType t, float *data, size_t dataSize)
{
	uint32_t timestamp;
	return readData(t, data, dataSize, timestamp);
}

size_t EmotiBit::readData(EmotiBit::DataType t, float *data, size_t dataSize, uint32_t &timestamp)
{
	float * dataBuffer;
	size_t bufferSize = 0;

	bool testing = false;
	if (testing)
	{
		if (_newDataAvailable[(uint8_t)t]) // if there is new data available on the outputBuffer
		{
			_newDataAvailable[(uint8_t)t] = false;
			bufferSize = 10;
			for (size_t i = 0; i < bufferSize && i < dataSize; i++)
			{
				data[i] = i;
			}
		}
	}
	else
	{
		bufferSize = readData(t, &dataBuffer, timestamp);
		// ToDo: optimize with memcpy
		for (size_t i = 0; i < bufferSize && i < dataSize; i++)
		{
			data[i] = dataBuffer[i];
		}
	}

	return bufferSize; // Return size of available buffer even if we're only able to copy some of it
}


size_t EmotiBit::readData(EmotiBit::DataType t, float **data)
{
	uint32_t timestamp;
	return readData(t, data, timestamp);
}

 
size_t EmotiBit::readData(EmotiBit::DataType t, float **data, uint32_t &timestamp)
{
	if ((uint8_t)t < (uint8_t)EmotiBit::DataType::length) {
		if (_newDataAvailable[(uint8_t)t]) // if there is new data available on the outputBuffer
		{
			_newDataAvailable[(uint8_t)t] = false;	
			return dataDoubleBuffers[(uint8_t)t]->getData(data, &timestamp, false);	// read data without swapping buffers
		}
	}
	return 0;
}

void EmotiBit::updateBatteryIndication()
{
	if (battIndicationSeq)
	{
		// Low Batt Indication is ON
		if (battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_HIGH))
		{
			// Wait until we hit he high threshold to avoid flickering
			battIndicationSeq = 0;
		}
	}
	else
	{
		// Low Batt Indication is OFF
		if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_LOW))
		{
			battIndicationSeq = 1;
		}
	}


	//// To update Battery level variable for LED indication
	//if (battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_HIGH))
	//	battIndicationSeq = 0;
	//else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_HIGH) && battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_MED)) {
	//	battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_HIGH);
	//	BattLedDuration = 1000;
	//}
	//else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_MED) && battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_LOW)) {
	//	battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_MED);
	//	BattLedDuration = 500;
	//}
	//else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_LOW)) {
	//	battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_LOW);
	//	BattLedDuration = 1;
	//}
}

void EmotiBit::appendTestData(String &dataMessage, uint16_t &packetNumber)
{
	for (uint8_t t = ((uint8_t)EmotiBit::DataType::DATA_CLIPPING) + 1; t < (uint8_t)DataType::length; t++)
	{
		String payload;
		int m = 5;
		int n = 2;
		for (int i = 0; i < m; i++)
		{
			int k;
			for (int j = 0; j < n; j++)
			{
				if (i > 0 || j > 0)
				{
					payload += ",";
				}
				k = i * 250 / m + random(50);
				payload += k;
			}
		}
		dataMessage += EmotiBitPacket::createPacket(typeTags[t], packetNumber++, payload, m * n);
	}
}

bool EmotiBit::createModePacket(String &modePacket, uint16_t &packetNumber)
{
	String payload;
	uint8_t dataCount = 0;
	payload += EmotiBitPacket::PayloadLabel::RECORDING_STATUS;
	dataCount++;
	payload += ',';
	if (_sdWrite)
	{
		payload += EmotiBitPacket::TypeTag::RECORD_BEGIN;
		// ToDo: Consider if we want to send the sdCardFilename in a separate key-value pair
		payload += ',';
		payload += _sdCardFilename;
	}
	else
	{
		payload += EmotiBitPacket::TypeTag::RECORD_END;
	}
	dataCount++;
	payload += ',';
	payload += EmotiBitPacket::PayloadLabel::POWER_STATUS;
	dataCount++;
	payload += ',';
	if (getPowerMode() == PowerMode::NORMAL_POWER)
	{
		payload += EmotiBitPacket::TypeTag::MODE_NORMAL_POWER;
	}
	else if (getPowerMode() == PowerMode::LOW_POWER)
	{
		payload += EmotiBitPacket::TypeTag::MODE_LOW_POWER;
	}
	else if(getPowerMode() == PowerMode::MAX_LOW_POWER)
	{
		payload += EmotiBitPacket::TypeTag::MODE_MAX_LOW_POWER;
	}
	else if(getPowerMode() == PowerMode::WIRELESS_OFF)
	{
		payload += EmotiBitPacket::TypeTag::MODE_WIRELESS_OFF;
	}
	else if (getPowerMode() == PowerMode::HIBERNATE)
	{
		payload += EmotiBitPacket::TypeTag::MODE_HIBERNATE;
	}
	dataCount++;

	modePacket += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_MODE, packetNumber++, payload, dataCount);

	return true;
}

void EmotiBit::sendModePacket(String &sentModePacket, uint16_t &packetNumber)
{
	createModePacket(sentModePacket, packetNumber);
	// ToDo: This should probably be over TCP in response to specific messages from Host (but will require writing TCP ingest)
	_emotiBitWiFi.sendData(sentModePacket);	// Send packet immediately to update host
	_outDataPackets += sentModePacket;			// Add packet to slower data logging bucket
}

void EmotiBit::processDebugInputs(String &debugPackets, uint16_t &packetNumber)
{
	if (Serial.available())
	{
		uint8_t dataCount = 1;
		String payload;

		char c = Serial.read();
		if (c == '?')
		{
			Serial.println("Debug tool options:");
			Serial.println("Press : for printing sensor data on Serial");
			Serial.println("Press / for initiating dummy isr with dummy values");
			Serial.println("Press R to print freeRam available");
			Serial.println("Press r to allocate free memory to simulate RAM needs");
			Serial.println("Press l to toggle OFF the NCP5623 LED driver");
			Serial.println("Press L to toggle ON the NCP5623 LED driver");
			Serial.println("Press t to togle OFF the Thermopile");
			Serial.println("Press T to togle ON the Thermopile");
			Serial.println("Press e to togle OFF the GSR");
			Serial.println("Press E to togle ON the GSR");
			Serial.println("Press h to togle OFF the Temp/Humidity Sensor");
			Serial.println("Press H to togle ON the Temp/Humidity Sensor");
			Serial.println("Press i to toggle OFF the IMU");
			Serial.println("Press I to toggle ON the IMU");
			Serial.println("Press p to toggle OFF the PPG sensor");
			Serial.println("Press P to toggle ON the PPG sensor");
			Serial.println("Press d to toggle OFF recording ISR loop time");
			Serial.println("Press D to toggle ON recording ISR loop time");
			Serial.println("Press b to toggle OFF Battry update");
			Serial.println("Press B to toggle ON Battery update");
			//Serial.println("Press 0 to simulate nan events in the thermopile");

		}
		else if (c == ':')
		{
			if (_serialData == DataType::length)
			{
				payload = "_serialData = DataType::DEBUG";
				Serial.println(payload);
				_serialData = DataType::DEBUG;
				debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			}
			else
			{
				payload = "_serialData OFF";
				Serial.println(payload);
				_serialData = DataType::length;
				debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			}
		}
		else if (c == '/')
		{
			dummyIsrWithDelay = !dummyIsrWithDelay;
			payload = "dummyIsrWithDelay = ";
			payload += dummyIsrWithDelay;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'R')
		{
			payload = "Free RAM: ";
			payload += String(freeMemory(), DEC);
			payload += " bytes";
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'r')
		{
			const int nFloats = 25;
			static int n = 0;
			n++;
			payload = "Wasted RAM: ";
			payload += nFloats * 4 * n;
			payload += " bytes";
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			float * data = new float[nFloats];
			payload = "Free RAM: ";
			payload += String(freeMemory(), DEC);
			payload += " bytes";
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'l')
		{
			chipBegun.NCP5623 = false;
			payload = "chipBegun.NCP5623 = ";
			payload += chipBegun.NCP5623;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'L')
		{
			chipBegun.NCP5623 = true;
			payload = "chipBegun.NCP5623 = ";
			payload += chipBegun.NCP5623;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 't')
		{
			acquireData.thermopile = false;
			payload = "acquireData.thermopile = ";
			payload += acquireData.thermopile;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'T')
		{
			acquireData.thermopile = true;
			payload = "acquireData.thermopile = ";
			payload += acquireData.thermopile;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			if (_serialData != DataType::length) _serialData = DataType::THERMOPILE;
		}
		else if (c == 'e')
		{
			acquireData.eda = false;
			payload = "acquireData.eda = ";
			payload += acquireData.eda;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'E')
		{
			acquireData.eda = true;
			payload = "acquireData.eda = ";
			payload += acquireData.eda;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			if (_serialData != DataType::length) _serialData = DataType::EDA;
		}
		else if (c == 'h')
		{
			acquireData.tempHumidity = false;
			payload = "acquireData.tempHumidity = ";
			payload += acquireData.tempHumidity;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'H')
		{
			acquireData.tempHumidity = true;
			payload = "acquireData.tempHumidity = ";
			payload += acquireData.tempHumidity;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			if (_serialData != DataType::length) _serialData = DataType::HUMIDITY_0;
		}
		else if (c == 'i')
		{
			acquireData.imu = false;
			payload = "acquireData.imu = ";
			payload += acquireData.imu;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'I')
		{
			acquireData.imu = true;
			payload = "acquireData.imu = ";
			payload += acquireData.imu;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			if (_serialData != DataType::length) _serialData = DataType::ACCELEROMETER_X;
		}
		else if (c == 'p')
		{
			acquireData.ppg = false;
			payload = "acquireData.ppg = ";
			payload += acquireData.ppg;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'P')
		{
			acquireData.ppg = true;
			payload = "acquireData.ppg = ";
			payload += acquireData.ppg;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			if (_serialData != DataType::length) _serialData = DataType::PPG_RED;
		}
		else if (c == 'd')
		{
			acquireData.debug = false;
			payload = "acquireData.debug = ";
			payload += acquireData.debug;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'D')
		{
			acquireData.debug = true;
			payload = "acquireData.debug = ";
			payload += acquireData.debug;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			if (_serialData != DataType::length) _serialData = DataType::DEBUG;
		}
		else if (c == 'b')
		{
			acquireData.battery = false;
			payload = "acquireData.battery = ";
			payload += acquireData.battery;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}
		else if (c == 'B')
		{
			acquireData.battery = true;
			payload = "acquireData.battery = ";
			payload += acquireData.battery;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
			if (_serialData != DataType::length) _serialData = DataType::BATTERY_PERCENT;
		}
		/*else if (c == '0')
		{
			catchDataException.catchNan = !catchDataException.catchNan;
			payload = "catchDataException.catchNan = ";
			payload += catchDataException.catchNan;
			Serial.println(payload);
			debugPackets += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::EMOTIBIT_DEBUG, packetNumber++, payload, dataCount);
		}*/
	}
}

void EmotiBit::setTimerFrequency(int frequencyHz) {
	int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
	TcCount16* TC = (TcCount16*)TC3;
	// Make sure the count is in a proportional position to where it was
	// to prevent any jitter or disconnect when changing the compare value.
	TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
	TC->CC[0].reg = compareValue;
	//Serial.println(TC->COUNT.reg);
	//Serial.println(TC->CC[0].reg);
	while (TC->STATUS.bit.SYNCBUSY == 1);
}

void EmotiBit::startTimer(int frequencyHz) {
	REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);
	while (GCLK->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TcCount16* TC = (TcCount16*)TC3;

	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Use the 16-bit timer
	TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Use match mode so that the timer counter resets when the count matches the compare register
	TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Set prescaler to 1024
	TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	setTimerFrequency(frequencyHz);

	// Enable the compare interrupt
	TC->INTENSET.reg = 0;
	TC->INTENSET.bit.MC0 = 1;

	NVIC_EnableIRQ(TC3_IRQn);

	TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void EmotiBit::stopTimer() {
	// ToDo: Verify implementation
	TcCount16* TC = (TcCount16*)TC3;
	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
}

void TC3_Handler() {

	TcCount16* TC = (TcCount16*)TC3;
	// If this interrupt is due to the compare register matching the timer count
	// we toggle the LED.
	if (TC->INTFLAG.bit.MC0 == 1) {
		TC->INTFLAG.bit.MC0 = 1;
		//scopeTimingTest();
		// Write callback here!!!
#ifdef TIMER_TEST
		toggleLED();
		Serial.println("LED Testing Routine");
#else
		onInterruptCallback();
#endif
	}
}

void attachToInterruptTC3(void(*readFunction)(void), EmotiBit* e)
{
	attachEmotiBit(e);
	onInterruptCallback = readFunction;
}

void attachEmotiBit(EmotiBit*e)
{
	myEmotiBit = e;
}

void ReadSensors()
{
	if (myEmotiBit != nullptr)
	{
		myEmotiBit->readSensors();

	}
}
