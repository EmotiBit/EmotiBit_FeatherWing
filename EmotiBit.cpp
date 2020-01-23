#include "EmotiBit.h"

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
	Serial.print("EmotiBit version: ");
	Serial.println((int)version);
	if (!_outDataPackets.reserve(OUT_MESSAGE_RESERVE_SIZE)) {
		Serial.println("Failed to reserve memory for output");
		while (true) {
			hibernate();
		}
	}

	delay(500);

	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDA] = &eda;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDL] = &edl;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDR] = &edr;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = &ppgInfrared;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_RED] = &ppgRed;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_GREEN] = &ppgGreen;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = &temp0;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::THERMOPILE] = &tempHP0;
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
	//dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PUSH_WHILE_GETTING] = &pushWhileGetting;

	//bufferCapacity = 32;
	//eda = DoubleBufferFloat(bufferCapacity);
	//ppgInfrared = DoubleBufferFloat(bufferCapacity);
	//ppgRed = DoubleBufferFloat(bufferCapacity);
	//ppgGreen = DoubleBufferFloat(bufferCapacity);
	//temp0 = DoubleBufferFloat(bufferCapacity);
	//tempHP0 = DoubleBufferFloat(bufferCapacity);
	//humidity0 = DoubleBufferFloat(bufferCapacity);
	//accelX = DoubleBufferFloat(bufferCapacity);
	//accelY = DoubleBufferFloat(bufferCapacity);
	//accelZ = DoubleBufferFloat(bufferCapacity);
	//gyroX = DoubleBufferFloat(bufferCapacity);
	//gyroY = DoubleBufferFloat(bufferCapacity);
	//gyroZ = DoubleBufferFloat(bufferCapacity);
	//magX = DoubleBufferFloat(bufferCapacity);
	//magY = DoubleBufferFloat(bufferCapacity);
	//magZ = DoubleBufferFloat(bufferCapacity);
	//edl.resize(bufferCapacity);
	//edr.resize(bufferCapacity);
	//ppgInfrared.resize(bufferCapacity);
	//ppgRed.resize(bufferCapacity);
	//ppgGreen.resize(bufferCapacity);
	//temp0.resize(bufferCapacity); 
	//tempHP0.resize(bufferCapacity);
	//humidity0.resize(bufferCapacity);
	//accelX.resize(bufferCapacity);
	//accelY.resize(bufferCapacity);
	//accelZ.resize(bufferCapacity);
	//gyroX.resize(bufferCapacity);
	//gyroY.resize(bufferCapacity);
	//gyroZ.resize(bufferCapacity);
	//magX.resize(bufferCapacity);
	//magY.resize(bufferCapacity);
	//magZ.resize(bufferCapacity);
	//batteryVoltage.resize(1);
	//batteryPercent.resize(1);

	_version = version;
	_vcc = 3.3f;						// Vcc voltage
	// vGnd = _vcc / 2.f;	// Virtual GND Voltage for eda
  //vRef1 = 0.44372341;
	vRef1 = _vcc * (15.f / (15.f + 100.f)); // First Voltage divider refernce
	vRef2 = _vcc * (100.f / (100.f + 100.f)); // Second voltage Divider reference

	_adcBits = 12;
	adcRes = pow(2, _adcBits);	// adc bit resolution

	if (version == Version::V01B || version == Version::V01C)
	{
		Serial.println("This code is not compatible with V01 (Alpha) boards");
		Serial.println("Download v0.6.0 for Alpha boards");
		while (true);
	}

	// Set board specific pins and constants
#if defined(ADAFRUIT_FEATHER_M0)
	_batteryReadPin = A7;
	if (version == Version::V01B || version == Version::V01C)
	{
		_hibernatePin = 5; // gpio pin assigned to the mosfet
		buttonPin = 13;
		_edlPin = A3;
		_edrPin = A4;
		_sdCardChipSelectPin = 6;
	}
	else if (version == Version::V02H)
	{
		_hibernatePin = 6; // gpio pin assigned to the mosfet
		buttonPin = 12;
		_edlPin = A4;
		_edrPin = A3;
		_sdCardChipSelectPin = 19;
		edrAmplification = 100.f / 3.3f;
		edaFeedbackAmpR = 4990000.f; // edaFeedbackAmpR in Ohms
	}
	else if (version == Version::V02B)
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
	if (version == Version::V01B || version == Version::V01C)
	{
		_hibernatePin = 27;//gpio pin assigned ot the mosfet
		buttonPin = 16;
		_gsrLowPin = A3;
		_gsrHighPin = A4;
	}
	analogReadResolution(_adcBits);
#endif

	// Print board-specific settings
	Serial.println("Board-specific settings:");
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
	Serial.println("Enabling analog circuitry...");
	pinMode(_hibernatePin, OUTPUT);
	digitalWrite(_hibernatePin, LOW);

	// Setup GSR
	Serial.println("Configuring EDA...");
	pinMode(_edlPin, INPUT);
	pinMode(_edrPin, INPUT);
	analogReadResolution(_adcBits);

	if (_EmotiBit_i2c != nullptr)
	{
		delete(_EmotiBit_i2c);
	}
	_EmotiBit_i2c = new TwoWire(&sercom1, 11, 13);
	// Flush the I2C
	Serial.println("Flushing I2C....");
	_EmotiBit_i2c->begin();
	_EmotiBit_i2c->setClock(100000);
	pinPeripheral(11, PIO_SERCOM);
	pinPeripheral(13, PIO_SERCOM);
	_EmotiBit_i2c->flush();
	//_EmotiBit_i2c->endTransmission();
	//_EmotiBit_i2c->clearWriteError();
	//_EmotiBit_i2c->end();
	
	// setup LED DRIVER
	led.begin(*_EmotiBit_i2c);
	led.setCurrent(26);
	led.setLEDpwm((uint8_t)Led::RED, 8);
	led.setLEDpwm((uint8_t)Led::BLUE, 8);
	led.setLEDpwm((uint8_t)Led::YELLOW, 8);

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
		//while (1);
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


	// Setup IMU
	Serial.println("Initializing IMU device....");
	BMI160.begin(BMI160GenClass::I2C_MODE, *_EmotiBit_i2c);
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

	// ToDo: Add interrupts to accurately record timing of data capture

	//BMI160.detachInterrupt();
	//BMI160.setRegister()

	// Setup Temperature / Humidity Sensor
	Serial.println("Configuring Temperature / Humidity Sensor");
	tempHumiditySensor.setup(*_EmotiBit_i2c);
	tempHumiditySensor.changeSetting(Si7013::Settings::RESOLUTION_H11_T11);
	tempHumiditySensor.changeSetting(Si7013::Settings::ADC_NORMAL);
	tempHumiditySensor.changeSetting(Si7013::Settings::VIN_UNBUFFERED);
	tempHumiditySensor.changeSetting(Si7013::Settings::VREFP_VDDA);
	tempHumiditySensor.changeSetting(Si7013::Settings::ADC_NO_HOLD);
	tempHumiditySensor.startHumidityTempMeasurement();
	
	// Thermopile
	MLX90632::status returnError; // Required as a parameter for begin() function in the MLX library 
	thermopile.begin(deviceAddress.MLX, *_EmotiBit_i2c, returnError);
	thermopile.setMeasurementRate(8);
	//lastThermopileBegin = millis();

	// ------------ BEGIN ino refactoring ------------//
	// ToDo: move into separate functions

	// setup sampling rates
	EmotiBit::SamplingRates samplingRates;
	samplingRates.accelerometer = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
	samplingRates.gyroscope = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
	samplingRates.magnetometer = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
	samplingRates.eda = BASE_SAMPLING_FREQ / EDA_SAMPLING_DIV;
	samplingRates.humidity = BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2;
	samplingRates.temperature = BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2;
	samplingRates.thermistor = BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2;
	setSamplingRates(samplingRates);

	// ToDo: make target down-sampled rates more transparent
	EmotiBit::SamplesAveraged samplesAveraged;
	samplesAveraged.eda = BASE_SAMPLING_FREQ / EDA_SAMPLING_DIV / 15;
	samplesAveraged.humidity = (float)BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2 / 7.5f;
	samplesAveraged.temperature = (float)BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2 / 7.5f;
	samplesAveraged.thermistor = (float)BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2 / 7.5f;
	samplesAveraged.battery = BASE_SAMPLING_FREQ / BATTERY_SAMPLING_DIV / 1;
	setSamplesAveraged(samplesAveraged);

	delay(500);

	setupSdCard();
	//_emotiBitWiFi.setup();
#if defined(ADAFRUIT_FEATHER_M0)
	WiFi.setPins(8, 7, 4, 2);
#endif
	WiFi.lowPowerMode();
	_emotiBitWiFi.begin();

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

	Serial.println("Free Ram :" + String(freeMemory(), DEC) + " bytes");
	Serial.println("EmotiBit Setup complete");

	if (!_sendTestData)
	{
		attachToInterruptTC3(&ReadSensors, this);
		Serial.println("Starting interrupts");
		startTimer(BASE_SAMPLING_FREQ);
	}

} // Setup

void EmotiBit::setupSdCard()
{
	Serial.print("Initializing SD card...");
	// see if the card is present and can be initialized:
	if (!SD.begin(_sdCardChipSelectPin)) {
		Serial.print("Card failed, or not present on chip select ");
		Serial.println(_sdCardChipSelectPin);
		// don't do anything more:
		// ToDo: Handle case where we still want to send network data
		while (true) {
			hibernate();
		}
	}
	Serial.println("card initialized.");
	SD.ls(LS_R);

	Serial.print(F("Loading configuration file: "));
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
#ifdef DEBUG_GET_DATA
	Serial.print("addPacket: ");
	Serial.println(typeTag);
#endif // DEBUGs
	static EmotiBitPacket::Header header;
	if (dataLen > 0) {
		// ToDo: Consider faster ways to populate the _outDataPackets

		//if (t == EmotiBit::DataType::DATA_OVERFLOW){
		//	addDebugPacket((uint8_t)EmotiBit::DebugTags::WIFI_CONNHISTORY, timestamp);  // addDebugPacket(case, timestamp) 
		//}

		header = EmotiBitPacket::createHeader(typeTags[(uint8_t)t], millis(), _outDataPacketCounter++, dataLen);
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
		if (dataStartChar > 0)
		{
			if (header.typeTag.equals(EmotiBitPacket::TypeTag::RECORD_BEGIN)) 
			{
				stopTimer();	// stop the data sampling timer
				String datetimeString = packet.substring(dataStartChar, packet.length() - 1);
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
			controlPackets += EmotiBitPacket::createPacket(header.typeTag, packetNumber++, packet.substring(dataStartChar, packet.length() - 1), header.dataLength);
		}
	}
}

bool EmotiBit::readButton()
{
	// ToDo: Consider reading pin mode https://arduino.stackexchange.com/questions/13165/how-to-read-pinmode-for-digital-pin
	return digitalRead(buttonPin);
}

void EmotiBit::updateButtonPress()
{
	uint16_t minShortButtonPress = 500;
	uint16_t minLongButtonPress = 3000;
	static uint32_t buttonPressTimer = millis();
	static bool buttonPreviouslyPressed = false;

	// ToDo: create a mechanism

	bool buttonPressed = readButton();
	if (buttonPressed)
	{
		buttonPreviouslyPressed = true;
	}
	else
	{
		if (buttonPreviouslyPressed) // Make sure button was actually pressed (not just a loop lag)
		{
			if (millis() - buttonPressTimer > minShortButtonPress && millis() - buttonPressTimer < minLongButtonPress)
			{
				Serial.print("onShortPress: ");
				Serial.println(millis() - buttonPressTimer);
				// ToDo: Send BS packet
				(onShortPressCallback());
			}
			if (millis() - buttonPressTimer > minLongButtonPress)
			{
				Serial.print("onLongPress: ");
				Serial.println(millis() - buttonPressTimer);
				// ToDo: Send BL packet
				(onLongPressCallback());
			}
		}
		buttonPreviouslyPressed = false;
		buttonPressTimer = millis();	// reset the timer until the button is pressed
	}
}

uint8_t EmotiBit::update()
{
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
			for (int16_t i = 0; i < (uint8_t)EmotiBit::DataType::length; i++)
			{
				addPacket((EmotiBit::DataType) i);
				if (_outDataPackets.length() > OUT_MESSAGE_RESERVE_SIZE - OUT_PACKET_MAX_SIZE)
				{
					// Avoid overrunning our reserve memory
					if (getPowerMode() == PowerMode::NORMAL_POWER)
					{
						_emotiBitWiFi.sendData(_outDataPackets);
					}
					writeSdCardMessage(_outDataPackets);
					_outDataPackets = "";
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
			// TODO: modify for all sensor calibration compatibility
			if (emotibitcalibration.gsrcalibration.isCalibrated()/*add other sensors .iscalibrated in this if*/)
			{
				Serial.println("Entered to TX onver WIfi");
				sendCalibrationPacket();
			}
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

void EmotiBit::sendCalibrationPacket()
{
	uint8_t gsr_precision = 10;
	for (int s = 0; s < (uint8_t)EmotiBitCalibration::SensorType::length; s++)
	{
		if (emotibitcalibration.getSensorToCalibrate((EmotiBitCalibration::SensorType)s))
		{
			EmotiBitPacket::Header header;
			header = EmotiBitPacket::createHeader(emotibitcalibration.calibrationTag[s], millis(), _outDataPacketCounter++, emotibitcalibration.calibrationPointsPerSensor[s]);
			_outDataPackets += EmotiBitPacket::headerToString(header);
			_outDataPackets += ',';
			_outDataPackets += String(emotibitcalibration.gsrcalibration.getCalibratedValue(), gsr_precision);
			_outDataPackets += "\n";
		}
	}
	_emotiBitWiFi.sendData(_outDataPackets);
	// TODO: Do a check on availability on buffer space of _outDataPackets
	Serial.println(_outDataPackets);
	_outDataPackets = "";
	Serial.println("Sending the data:");
	Serial.println(emotibitcalibration.gsrcalibration.getCalibratedValue(), gsr_precision);
	Serial.println("Ending Execution");
	while (1);
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
	if (edlTemp < adcClippingLowerLim
		|| edlTemp > adcClippingUpperLim
		) {
		edlClipped = true;
		status = status | (int8_t) Error::DATA_CLIPPING;
	}
	// Check for data clipping
	if (edrTemp < adcClippingLowerLim
		|| edrTemp > adcClippingUpperLim
		) {
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
		
		// CALIBRATION
		if (emotibitcalibration.getSensorToCalibrate(EmotiBitCalibration::SensorType::GSR) && !emotibitcalibration.gsrcalibration.isCalibrated())
		{
			emotibitcalibration.gsrcalibration.performCalibration(edlTemp);
		}

		pushData(EmotiBit::DataType::EDL, edlTemp, &edlBuffer.timestamp);
		if (edlClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDL, &edlBuffer.timestamp);
		}

		pushData(EmotiBit::DataType::EDR, edrTemp, &edrBuffer.timestamp);
		if (edrClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDR, &edrBuffer.timestamp);
		}
		// Link to diff amp biasing: https://ocw.mit.edu/courses/media-arts-and-sciences/mas-836-sensor-technologies-for-interactive-environments-spring-2011/readings/MITMAS_836S11_read02_bias.pdf
		edaTemp = (edrTemp - vRef2) / edrAmplification;	// Remove VGND bias and amplification from EDR measurement
		edaTemp = edaTemp + edlTemp;                     // Add EDR to EDL in Volts

		//edaTemp = (_vcc - edaTemp) / edaVDivR * 1000000.f;						// Convert EDA voltage to uSeimens
		if (edaTemp <= vRef1) {
			edaTemp = 1; // Clamp the EDA measurement at 1 Ohm (1M uSiemens)
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
	return status;
}


int8_t EmotiBit::updateThermopileData() {
	int8_t status = 0;
	// Thermopile
	if (!thermopileBegun) {// for the first time
		thermopile.start_getObjectTemp();
		thermopileBegun = true;
		//lastThermopileBegin = millis();
	}
	else {
		/*Serial.print("Thermopile:");
		Serial.println(thermopile.end_getObjectTemp());*/
		uint32_t time_stamp = millis();
		status = status | pushData(EmotiBit::DataType::THERMOPILE, thermopile.end_getObjectTemp(), &(time_stamp));
		thermopile.start_getObjectTemp();
		//lastThermopileBegin = millis();
	}
	return status;
}
//rslt = bmi160_get_fifo_data(dev);

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

size_t EmotiBit::getData(DataType type, float** data, uint32_t * timestamp) {
#ifdef DEBUG
	Serial.print("getData: type=");
	Serial.println((uint8_t) t);
#endif // DEBUG
	if ((uint8_t)type < (uint8_t)EmotiBit::DataType::length) {
		return dataDoubleBuffers[(uint8_t)type]->getData(data, timestamp);
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
	batRead /= adcRes;
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
	int hardware_version = (int)_version;
	String feather_version = "Adafruit Feather M0 WiFi";
	String firmware_version = "1.0.0";

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
		infos[i]->set("acc_bwp", imuSettings.acc_bwp);
		infos[i]->set("acc_us", imuSettings.acc_us);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "G/second");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("range", _accelerometerRange);

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
		infos[i]->set("gyr_bwp", imuSettings.gyr_bwp);
		infos[i]->set("gyr_us", imuSettings.gyr_us);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "degrees/second");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("range", _gyroRange);
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
		infos[i]->set("units", "raw samples");
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
		setups[i]->set("voltage_divider_resistance", edaVDivR);
		setups[i]->set("EDR_amplification", edrAmplification);
		setups[i]->set("low_pass_filter_frequency", "15.91Hz");
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

		// Thermistor
		//i++;
		//indices[i] = &(root.createNestedObject());
		//infos[i] = &(indices[i]->createNestedObject("info"));
		infos[i]->set("name", "Thermistor");
		infos[i]->set("type", "Thermistor");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("TH");
		infos[i]->set("channel_count", 1);
		infos[i]->set("nominal_srate", _samplingRates.thermistor / _samplesAveraged.thermistor);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "raw adc units");
		infos[i]->set("source_id", source_id);
		infos[i]->set("hardware_version", hardware_version);
		infos[i]->set("feather_version", feather_version);
		infos[i]->set("firmware_version", firmware_version);
		infos[i]->set("created_at", datetimeString);
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("ADC_speed", "ADC_NORMAL");
		setups[i]->set("Vin_buffering", "VIN_UNBUFFERED");
		setups[i]->set("VREFP", "VREFP_VDDA");
		setups[i]->set("voltage_divider_resistance", 10000);
		setups[i]->set("thermistor_resistance", 10000);
		setups[i]->set("low_pass_filter_frequency", "15.91Hz");
		setups[i]->set("low_pass_filter_frequency", "0.1591Hz");
		setups[i]->set("amplification", 10);
		setups[i]->set("samples_averaged", _samplesAveraged.thermistor);
		setups[i]->set("oversampling_rate", _samplingRates.thermistor);
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
	static float f;
	f = 0;
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

	// LED STATUS CHANGE SEGMENT
	static uint16_t ledCounter = 0;
	ledCounter++;
	/*Serial.print("ledCounter:");
	Serial.println(ledCounter);*/
	if (ledCounter == LED_REFRESH_DIV) 
	{
		ledCounter = 0;
		// Serial.println("Time to update LED");
		if (_emotiBitWiFi.isConnected())
		{
			led.setLED(uint8_t(EmotiBit::Led::BLUE), true);
		}
		else
		{
			led.setLED(uint8_t(EmotiBit::Led::BLUE), false);
		}

		//static bool battLed = false;
		if (battIndicationSeq) 
		{
			static uint32_t BattLedstatusChangeTime = millis();
			if (millis() - BattLedstatusChangeTime > BattLedDuration)
			{
				led.setLED(uint8_t(EmotiBit::Led::YELLOW), !led.getLED(uint8_t(EmotiBit::Led::YELLOW)));
			}
			//if (battLed && millis() - BattLedstatusChangeTime > BattLedDuration) 
			//{
			//	led.setLED(uint8_t(EmotiBit::Led::YELLOW), false);
			//	battLed = false;
			//	BattLedstatusChangeTime = millis();
			//}
			//else if (!battLed && millis() - BattLedstatusChangeTime > BattLedDuration) 
			//{
			//	led.setLED(uint8_t(EmotiBit::Led::YELLOW), true);
			//	battLed = true;
			//	BattLedstatusChangeTime = millis();
			//}
		}
		else 
		{
			led.setLED(uint8_t(EmotiBit::Led::YELLOW), false);
			//battLed = false;
		}

		//static bool recordLedStatus = false;
		if (_sdWrite) 
		{
			static uint32_t recordBlinkDuration = millis();
			if (millis() - recordBlinkDuration >= 500) 
			{
				//if (recordLedStatus == true) 
				//{
				//	led.setLED(uint8_t(EmotiBit::Led::RED), false);
				//	recordLedStatus = false;
				//}
				//else 
				//{
				//	led.setLED(uint8_t(EmotiBit::Led::RED), true);
				//	recordLedStatus = true;
				//}
				led.setLED(uint8_t(EmotiBit::Led::RED), !led.getLED(uint8_t(EmotiBit::Led::RED)));
				recordBlinkDuration = millis();
			}
		}
		else if (!_sdWrite && led.getLED(uint8_t(EmotiBit::Led::RED)) == true)
		{
			led.setLED(uint8_t(EmotiBit::Led::RED), false);
			// recordLedStatus = false;
		}
	}

	// EDA
	if (acquireData.eda) {
		static uint16_t edaCounter = 0;
		edaCounter++;
		if (edaCounter == EDA_SAMPLING_DIV) {
			int8_t tempStatus = updateEDAData();
			edaCounter = 0;
		}
	}

	// Temperature / Humidity Sensor
	if (acquireData.tempHumidity) {
		static uint16_t temperatureCounter = 0;
		temperatureCounter++;
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
	}

	// Thermopile
	if (acquireData.tempHumidity) {
		static uint16_t thermopileCounter = 0;
		thermopileCounter++;
		if (thermopileCounter == THERMOPILE_SAMPLING_DIV) {
			int8_t tempStatus = updateThermopileData();
			thermopileCounter = 0;
		}
	}

	// IMU
	if (acquireData.imu) {
		static uint16_t imuCounter = 0;
		imuCounter++;
		if (imuCounter == IMU_SAMPLING_DIV) {
			int8_t tempStatus = updateIMUData();
			imuCounter = 0;
		}
	}

	// PPG
	if (acquireData.ppg) {
		static uint16_t ppgCounter = 0;
		ppgCounter++;
		if (ppgCounter == PPG_SAMPLING_DIV) {
			int8_t tempStatus = updatePPGData();
			ppgCounter = 0;
		}
	}

	// Battery (all analog reads must be in the ISR)
	// TODO: use the stored BAtt value instead of calling readBatteryPercent again
	static uint16_t batteryCounter = 0;
	batteryCounter++;
	if (batteryCounter == BATTERY_SAMPLING_DIV) {
		battLevel = readBatteryPercent();
		updateBatteryIndication();
		updateBatteryPercentData();
		batteryCounter = 0;
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

	//PPGToggle
	// For an unknown reason, this need to be before WiFi diconnect/end
	Serial.println("Shutting down ppg...");
	ppgSensor.shutDown();

	Serial.println("Ending WiFi...");
	_emotiBitWiFi.end();

	//IMU Suspend Mode
	Serial.println("Suspending IMU...");
	BMI160.suspendIMU();

	SPI.end(); // shutdown the SPI interface
	Wire.end();
	_EmotiBit_i2c->end();

	//pinMode(_sdCardChipSelectPin, OUTPUT);//cs
	//digitalWrite(_sdCardChipSelectPin, LOW);
	//pinMode(PIN_SPI_MISO, OUTPUT);
	//digitalWrite(PIN_SPI_MISO, LOW);
	//pinMode(PIN_SPI_MOSI, OUTPUT);
	//digitalWrite(PIN_SPI_MOSI, LOW);
	//pinMode(PIN_SPI_SCK, OUTPUT);
	//digitalWrite(PIN_SPI_SCK, LOW);

	//pinMode(PIN_WIRE_SCL, OUTPUT);
	//digitalWrite(PIN_WIRE_SCL, LOW);
	//pinMode(PIN_WIRE_SDA, OUTPUT);
	//digitalWrite(PIN_WIRE_SDA, LOW);

	//pinMode(PIN_UART, OUTPUT);
	//digitalWrite(PIN_WIRE_SCL, LOW);
	//pinMode(PIN_WIRE_SDA, OUTPUT);
	//digitalWrite(PIN_WIRE_SDA, LOW);

	/*while (ledPinBusy)*/

		// Setup all pins (digital and analog) in INPUT mode (default is nothing)  
		//for (uint32_t ul = 0; ul < NUM_DIGITAL_PINS; ul++)
	for (uint32_t ul = 0; ul < PINS_COUNT; ul++)
	{
		if (ul != _hibernatePin) {
			pinMode(ul, OUTPUT);
			digitalWrite(ul, LOW);
			pinMode(ul, INPUT);
		}
	}

	//GSRToggle, write 1 to the PMO
	Serial.println("Disabling MOSFET ");
	pinMode(_hibernatePin, OUTPUT);
	digitalWrite(_hibernatePin, HIGH);

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
			_dataFile.sync();
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
		size_t bufferSize = readData(t, &dataBuffer, timestamp);
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
	// To update Battery level variable for LED indication
	if (battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_HIGH))
		battIndicationSeq = 0;
	else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_HIGH) && battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_MED)) {
		battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_HIGH);
		BattLedDuration = 1000;
	}
	else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_MED) && battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_LOW)) {
		battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_MED);
		BattLedDuration = 500;
	}
	else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_LOW)) {
		battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_LOW);
		BattLedDuration = 100;
	}
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
