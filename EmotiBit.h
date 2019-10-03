//#define DEBUG


#ifndef _EMOTIBIT_H_
#define _EMOTIBIT_H_

#include "Arduino.h"
#include <Wire.h>
#include <EmotiBit_Si7013.h>
#include <BMI160Gen.h>
#include <MAX30105.h>
#include "DoubleBufferFloat.h"
#include <ArduinoJson.h>
#include <SdFat.h>


class EmotiBit {
  
public:

	enum class SensorTimer {
		MANUAL
	};

	enum class Version {
		V01B,
		V01C
	};

  typedef struct IMUSettings {
    int gyroResolution = 250;
    int accResolution = 2;
		uint8_t IF_CONF = 0x6B; //BMI160_IF_CONF
    uint8_t FIFO_CONF = 0x47;//BMI160_RA_FIFO_CONFIG_1
    uint8_t MAG_IF_0 = 0x4B;//BMI160_MAG_IF_0
    uint8_t MAG_IF_1 = 0x4C;//BMI160_MAG_IF_1
    uint8_t MAG_IF_2 = 0x4D;//BMI160_MAG_IF_2
    uint8_t MAG_IF_3 = 0x4E;//BMI160_MAG_IF_3
    uint8_t MAG_IF_4 = 0x4F;//BMI160_MAG_IF_4
    uint8_t DATA_T_L = 0x20;//BMI160_RA_TEMP_L 
    uint8_t DATA_T_M = 0x21;//BMI160_RA_TEMP_M 
    uint8_t DATA_MAG_X_L = 0x04;//BMI160_RA_MAG_X_L
    uint8_t DATA_MAG_X_M = 0x05;//BMI160_RA_MAG_X_M
    uint8_t MAG_CONF = 0x44;
    uint8_t FIFO_L = 0x46;
    uint8_t FIFO_M = 0x47;
    uint8_t STATUS = 0x1B;
    uint8_t I2C_MAG = 0x20;
    uint8_t MAG_DATA_8BYTE = 0x03;
    uint8_t ADD_BMM_MEASURE = 0x4C; 
    uint8_t ADD_BMM_DATA = 0x42;
  };

 
  typedef struct BMM150TrimData {
	  int8_t dig_x1;
	  int8_t dig_y1;
	  int8_t dig_x2;
	  int8_t dig_y2;
	  uint16_t dig_z1;
	  int16_t dig_z2;
	  int16_t dig_z3;
	  int16_t dig_z4;
	  uint8_t dig_xy1;
	  int8_t dig_xy2;
	  uint16_t dig_xyz1;
  };
  
  typedef struct PPGSettings {
    uint8_t ledPowerLevel = 0x2F; //Options: 0=Off to 255=50mA
    uint16_t sampleAverage = 16;   //Options: 1, 2, 4, 8, 16, 32
    uint8_t ledMode = 3;          //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    uint16_t sampleRate = 400;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    uint16_t pulseWidth = 215;     //Options: 69, 118, 215, 411
    uint16_t adcRange = 4096;     //Options: 2048, 4096, 8192, 16384
  };

	typedef struct SamplingRates {
		uint16_t eda = 0;
		uint16_t accelerometer = 0;
		uint16_t gyroscope = 0;
		uint16_t magnetometer = 0;
		uint16_t humidity = 0;
		uint16_t temperature = 0;
		uint16_t thermistor = 0;
		uint16_t ppg = 0;
	};

	typedef struct SamplesAveraged {
		uint8_t eda = 1;
		uint8_t humidity = 1;
		uint8_t temperature = 1;
		uint8_t thermistor = 1;
	};

	enum class Error {
		NONE = 0,
		BUFFER_OVERFLOW = -1,
		DATA_CLIPPING = -2,
		INDEX_OUT_OF_BOUNDS = -4
	};

  enum class DataType{
  	DATA_OVERFLOW,
	DATA_CLIPPING,
	//TODO: Add a debug data type
	EDA,
	EDL,
	EDR,
	PPG_INFRARED,
	PPG_RED,
	PPG_GREEN,
	TEMPERATURE_0,
	TEMPERATURE_HP0,
	HUMIDITY_0,
	ACCELEROMETER_X,
	ACCELEROMETER_Y,
	ACCELEROMETER_Z,
	GYROSCOPE_X,
	GYROSCOPE_Y,
	GYROSCOPE_Z,
	MAGNETOMETER_X,
	MAGNETOMETER_Y,
	MAGNETOMETER_Z,
	BATTERY_VOLTAGE,
	BATTERY_PERCENT,
	//PUSH_WHILE_GETTING,
	length
  };
  
  Si7013 tempHumiditySensor;
	uint8_t switchPin;
	PPGSettings ppgSettings;
  MAX30105 ppgSensor;
	float edrAmplification;
	float vGnd;
	float adcRes;
	float edaVDivR;
	uint8_t _sdCardChipSelectPin;	// ToDo: create getter and make private
BMM150TrimData bmm150TrimData;
	bool bmm150XYClipped = false;
	bool bmm150ZHallClipped = false;
	
  
  EmotiBit();
  uint8_t setup(Version version = Version::V01C, size_t bufferCapacity = 64);   /**< Setup all the sensors */
  void setAnalogEnablePin(uint8_t i); /**< Power on/off the analog circuitry */
  int8_t updateIMUData();                /**< Read any available IMU data from the sensor FIFO into the inputBuffers */
	int8_t updatePPGData();                /**< Read any available PPG data from the sensor FIFO into the inputBuffers */
	int8_t updateEDAData();                /**< Take EDA reading and put into the inputBuffer */
	int8_t updateTempHumidityData();       /**< Read any available temperature and humidity data into the inputBuffers */
	int8_t updateBatteryVoltageData();     /**< Take battery voltage reading and put into the inputBuffer */
	int8_t updateBatteryPercentData();     /**< Take battery percent reading and put into the inputBuffer */
	float convertRawGyro(int16_t gRaw);
	float convertRawAcc(int16_t aRaw);
	float convertMagnetoX(int16_t mag_data_x, uint16_t data_rhall);
	float convertMagnetoY(int16_t mag_data_y, uint16_t data_rhall);
	float convertMagnetoZ(int16_t mag_data_z, uint16_t data_rhall);
	bool getBit(uint8_t num, uint8_t bit);
	void bmm150GetRegs(uint8_t address, uint8_t* dest, uint16_t len);
	void bmm150ReadTrimRegisters();
	// https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L90-L99
	// https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150_defs.h

  
  size_t getData(DataType t, float** data, uint32_t * timestamp = nullptr); /**< Swap buffers and read out the data */
	//size_t dataAvailable(DataType t);
	float readBatteryVoltage();
	int8_t readBatteryPercent();
	bool setSensorTimer(SensorTimer t);
	bool printConfigInfo(File &file, String datetimeString);
	//bool printConfigInfo(File file, String datetimeString);
	bool setSamplingRates(SamplingRates s);
	bool setSamplesAveraged(SamplesAveraged s);
	void scopeTimingTest();

private:
	float average(BufferFloat &b);
	int8_t checkIMUClipping(EmotiBit::DataType type, int16_t data, uint32_t timestamp);
	int8_t pushData(EmotiBit::DataType type, float data, uint32_t * timestamp = nullptr);

	SensorTimer _sensorTimer = SensorTimer::MANUAL;
	SamplingRates _samplingRates;
	SamplesAveraged _samplesAveraged;
	uint8_t _batteryReadPin;
  uint8_t _analogEnablePin;
  uint8_t _edlPin;
  uint8_t _edrPin;
	float _vcc;
	uint8_t _adcBits;
	float _accelerometerRange; // supported values: 2, 4, 8, 16 (G)
	float _gyroRange; // supported values: 125, 250, 500, 1000, 2000 (degrees/second)
	Version _version;


	DoubleBufferFloat eda;
	DoubleBufferFloat edl;
	DoubleBufferFloat edr;
	DoubleBufferFloat ppgInfrared;
	DoubleBufferFloat ppgRed;
	DoubleBufferFloat ppgGreen;
	DoubleBufferFloat temp0;
	DoubleBufferFloat tempHP0;
	DoubleBufferFloat humidity0;
	DoubleBufferFloat accelX;
	DoubleBufferFloat accelY;
	DoubleBufferFloat accelZ;
	DoubleBufferFloat gyroX;
	DoubleBufferFloat gyroY;
	DoubleBufferFloat gyroZ;
	DoubleBufferFloat magX;
	DoubleBufferFloat magY;
	DoubleBufferFloat magZ;
	DoubleBufferFloat batteryVoltage = DoubleBufferFloat(1);
	DoubleBufferFloat batteryPercent = DoubleBufferFloat(1);
	DoubleBufferFloat dataOverflow; //= DoubleBufferFloat(16);
	DoubleBufferFloat dataClipping; //= DoubleBufferFloat(16);
	//DoubleBufferFloat pushWhileGetting;

	DoubleBufferFloat * dataDoubleBuffers[(uint8_t)DataType::length];

	// Oversampling buffers
	// Single buffered arrays must only be accessed from ISR functions, not in the main loop
	// ToDo: add assignment for dynamic allocation;
	BufferFloat edlBuffer = BufferFloat(8);	
	BufferFloat edrBuffer = BufferFloat(8);	
	BufferFloat thermistorBuffer = BufferFloat(8);	
	BufferFloat temperatureBuffer = BufferFloat(8);	
	BufferFloat humidityBuffer = BufferFloat(8);

	const uint8_t SCOPE_TEST_PIN = A0;
	bool scopeTestPinOn = false;

};

#endif
