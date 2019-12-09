#include "EmotiBit.h"

TwoWire EmotiBit_i2c(&sercom1, 11, 13);

EmotiBit::EmotiBit() {
	
}

bool EmotiBit::setSamplingRates(SamplingRates s) {
	_samplingRates = s;
	_samplingRates.ppg = ppgSettings.sampleRate / ppgSettings.sampleAverage;
	_samplingRates.accelerometer = 25.f * pow(2.f, ((float)imuSettings.acc_odr - 6.f));	// See lookup table in BMI160 datasheet
	_samplingRates.gyroscope = 25.f * pow(2.f, ((float)imuSettings.gyr_odr - 6.f));		// See lookup table in BMI160 datasheet
	_samplingRates.magnetometer = 25.f * pow(2.f, ((float)imuSettings.mag_odr - 6.f));	// See lookup table in BMI160 datasheet
}

bool EmotiBit::setSamplesAveraged(SamplesAveraged s) {
	_samplesAveraged = s;
}

bool EmotiBit::getBit(uint8_t num, uint8_t bit) {
	uint8_t mask = 1 << bit;
	return mask == (num & mask);
}
/*!
 * @brief This API reads the data from the given register address of the sensor.
 https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L579
 */
void EmotiBit::bmm150GetRegs(uint8_t address, uint8_t* dest, uint16_t len) {
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
void EmotiBit::bmm150ReadTrimRegisters() {
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

uint8_t EmotiBit::setup(Version version, size_t bufferCapacity) {
#ifdef DEBUG
	Serial.print("setup: version="); 
	Serial.println((int)version);
#endif // DEBUG

	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDA] = &eda;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDL] = &edl;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDR] = &edr;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = &ppgInfrared;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_RED] = &ppgRed;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_GREEN] = &ppgGreen;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = &temp0;
	dataDoubleBuffers[(uint8_t)EmotiBit::DataType::TEMPERATURE_HP0] = &tempHP0;
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
	vRef1 = _vcc * (15.f / 115.f); // First Voltage divider refernce
	vRef2 = _vcc / 2.f; // Second voltage Divider reference

	// Set board specific pins and constants
#if defined(ADAFRUIT_FEATHER_M0)
	_analogEnablePin = 6; // gpio pin assigned to the mosfet
	switchPin = 12;
	_batteryReadPin = A7;
	_edlPin = A4;
	_edrPin = A3;
	_sdCardChipSelectPin = 19;
	_adcBits = 12;
	adcRes = pow(2, _adcBits);	// adc bit resolution
#elif defined(ADAFRUIT_BLUEFRUIT_NRF52_FEATHER)
	_analogEnablePin = 27;//gpio pin assigned ot the mosfet
	switchPin = 16;
	_gsrLowPin = A3;
	_gsrHighPin = A4;
	analogReadResolution(12);
	adcRes = 4096.f;	// adc bit resolution
#endif

	if (version == Version::V01B) {
		edaVDivR = 5000000.f;         // R17 5MOhm
		edrAmplification = 20; // 20x
	}
	else if (version == Version::V01C) {
		edaVDivR = 5000000.f;           //R17 5MOhm
		edrAmplification = 83.33; // 83.33x
		rSkinAmp = 4.99; // RskinAmp in Mega Ohms
	}
	// Print board-specific settings
	Serial.println("Board-specific settings:");
	Serial.print("switchPin = "); Serial.println(switchPin);
	Serial.print("_batteryReadPin = "); Serial.println(_batteryReadPin);
	Serial.print("_analogEnablePin = "); Serial.println(_analogEnablePin);
	Serial.print("_edlPin = "); Serial.println(_edlPin);
	Serial.print("_edrPin = "); Serial.println(_edrPin);
	Serial.print("_vcc = "); Serial.println(_vcc);
	Serial.print("adcRes = "); Serial.println(adcRes);
	Serial.print("edaVDivR = "); Serial.println(edaVDivR);
	Serial.print("edrAmplification = "); Serial.println(edrAmplification);

	// Setup switch
	if (switchPin != LED_BUILTIN) {
		// If the LED_BUILTIN and switchpin are the same leave it as it was
		// Otherwise setup the input
		pinMode(switchPin, INPUT);
	}

	// Setup battery Reading
	pinMode(_batteryReadPin, INPUT);
	
	// Enable analog circuitry
	Serial.println("Enabling analog circuitry...");
	pinMode(_analogEnablePin, OUTPUT);
	digitalWrite(_analogEnablePin, LOW);

	// Setup GSR
	Serial.println("Configuring EDA...");
	pinMode(_edlPin, INPUT);
	pinMode(_edrPin, INPUT);
	analogReadResolution(_adcBits);

	// Flush the I2C
	Serial.println("Flushing I2C....");
	EmotiBit_i2c.begin();
	EmotiBit_i2c.setClock(100000);
	pinPeripheral(11, PIO_SERCOM);
	pinPeripheral(13, PIO_SERCOM);
	EmotiBit_i2c.flush();
	//EmotiBit_i2c.endTransmission();
	//EmotiBit_i2c.clearWriteError();
	//EmotiBit_i2c.end();
	
	// setup LED DRIVER
	led.begin(EmotiBit_i2c);

	//// Setup PPG sensor
	Serial.println("Initializing MAX30101....");
	// Initialize sensor
	while (ppgSensor.begin(EmotiBit_i2c) == false) // reads the part number to confirm device
	{
		Serial.println("MAX30101 was not found. Please check wiring/power. ");
		EmotiBit_i2c.flush();
		EmotiBit_i2c.endTransmission();
		EmotiBit_i2c.clearWriteError();
		EmotiBit_i2c.end();
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
	BMI160.begin(BMI160GenClass::I2C_MODE, EmotiBit_i2c);
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
	if (_imuFifoFrameLen > _maxImuFifoFrameLen) {
		// ToDo: handle _imuFifoFrameLen > _maxImuFifoFrameLen
		Serial.println("UNHANDLED CASE: _imuFifoFrameLen > _maxImuFifoFrameLen");
		while (true);
	}


	// ToDo: Add interrupts to accurately record timing of data capture

	//BMI160.detachInterrupt();
	//BMI160.setRegister()

	// Setup Temperature / Humidity Sensor
	Serial.println("Configuring Temperature / Humidity Sensor");
	tempHumiditySensor.setup(EmotiBit_i2c);
	tempHumiditySensor.changeSetting(Si7013::Settings::RESOLUTION_H11_T11);
	tempHumiditySensor.changeSetting(Si7013::Settings::ADC_NORMAL);
	tempHumiditySensor.changeSetting(Si7013::Settings::VIN_UNBUFFERED);
	tempHumiditySensor.changeSetting(Si7013::Settings::VREFP_VDDA);
	tempHumiditySensor.changeSetting(Si7013::Settings::ADC_NO_HOLD);
	tempHumiditySensor.startHumidityTempMeasurement();



	//delay(1000);

}

int8_t EmotiBit::updateEDAData() {
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

		pushData(EmotiBit::DataType::EDL, edlTemp, &edlBuffer.timestamp);
		if (edlClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDL, &edlBuffer.timestamp);
		}

		pushData(EmotiBit::DataType::EDR, edrTemp, &edrBuffer.timestamp);
		if (edrClipped) {
			pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDR, &edrBuffer.timestamp);
		}

		edaTemp = (edrTemp - vRef2) / edrAmplification;	// Remove VGND bias and amplification from EDR measurement
		edaTemp = edaTemp + edlTemp;											// Add EDR to EDL in Volts

		//edaTemp = (_vcc - edaTemp) / edaVDivR * 1000000.f;						// Convert EDA voltage to uSeimens
		edaTemp = vRef1 / (rSkinAmp * (edaTemp - vRef1)); // Conductance in uSiemens


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
		else if (tempHumiditySensor.isAdcNew() == true) {
			static float thermTemp;
			thermTemp = tempHumiditySensor.getAdc();
			thermistorBuffer.push_back(thermTemp);
			if (thermistorBuffer.size() >= _samplesAveraged.thermistor) {
				// ToDo: Convert  to degrees
				//thermTemp = thermTemp * _vcc / _thermistorAdcResolution;	// Convert ADC to Volts
				//thermTemp = (thermTemp - vGnd) / thermistorAmplification;	// Remove VGND bias and amplification from thermistor measurement
				//thermTemp = thermTemp / (1 - thermTemp) / thermistorToDividerRatio;	// Convert Volts to Ohms
				// rDivVal
				// rThermVal
				// ohmToC

				// ToDo: Add clipping check

				// Steps:
				// Convert adc to volts: adcVal 
				// Remove VGND bias and amplification from EDR measurement
				// Convert to Ohms
				// Convert to degrees
				// B constant 3380
				// 20C = 12.09kOhm
				// 25C = 10KOhm
				// 30C = 8.31kOhm
				// Amplification 10x
				status = status | pushData(EmotiBit::DataType::TEMPERATURE_HP0, average(thermistorBuffer), &(thermistorBuffer.timestamp));
				thermistorBuffer.clear();
			}
			tempHumiditySensor.startHumidityTempMeasurement();

		}
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



bool EmotiBit::printConfigInfo(File &file, String datetimeString) {
#ifdef DEBUG
	Serial.println("printConfigInfo");
#endif
	//bool EmotiBit::printConfigInfo(File file, String datetimeString) {
	String source_id = "EmotiBit FeatherWing";
	int hardware_version = (int)_version;
	String feather_version = "Adafruit Feather M0 WiFi";
	String firmware_version = "0.5.8";

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
