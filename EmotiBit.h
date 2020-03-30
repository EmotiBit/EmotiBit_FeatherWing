//#define DEBUG

#ifndef _EMOTIBIT_H_
#define _EMOTIBIT_H_

#include "Arduino.h"
#include <Wire.h>
#include <EmotiBit_Si7013.h>
#include <BMI160Gen.h>
#include <MAX30105.h>
#include <EmotiBit_NCP5623.h>
#include <SparkFun_MLX90632_Arduino_Library.h>
#include "DoubleBufferFloat.h"
#include <ArduinoJson.h>
#include <SdFat.h>
#include "wiring_private.h"
#include "EmotiBitWiFi.h"
#include <SPI.h>
#include <SdFat.h>
#include <ArduinoJson.h>
#include <ArduinoLowPower.h>

//#include <Adafruit_SleepyDog.h>


class EmotiBit {
  
public:
	enum class TestingMode
	{
		NONE,
		CHRONIC,
		ACUTE,
		length
	};

	String firmware_version = "1.0.55";
	TestingMode testingMode = TestingMode::ACUTE;
	const bool DIGITAL_WRITE_DEBUG = false;

	bool _debugMode = false;
	bool dummyIsrWithDelay = false;
	uint32_t targetFileSyncDelay = 1;


	enum class SensorTimer {
		MANUAL
	};

	enum class Version {
		V01B,
		V01C,
		V02F,
		V02H,
		V02B
	};

	//struct IMUSettings {
	//int gyroResolution = 250;
	//int accResolution = 2;
	//	uint8_t IF_CONF = 0x6B; //BMI160_IF_CONF
	//uint8_t FIFO_CONF = 0x47;//BMI160_RA_FIFO_CONFIG_1
	//uint8_t MAG_IF_0 = 0x4B;//BMI160_MAG_IF_0
	//uint8_t MAG_IF_1 = 0x4C;//BMI160_MAG_IF_1
	//uint8_t MAG_IF_2 = 0x4D;//BMI160_MAG_IF_2
	//uint8_t MAG_IF_3 = 0x4E;//BMI160_MAG_IF_3
	//uint8_t MAG_IF_4 = 0x4F;//BMI160_MAG_IF_4
	//uint8_t DATA_T_L = 0x20;//BMI160_RA_TEMP_L 
	//uint8_t DATA_T_M = 0x21;//BMI160_RA_TEMP_M 
	//uint8_t DATA_MAG_X_L = 0x04;//BMI160_RA_MAG_X_L
	//uint8_t DATA_MAG_X_M = 0x05;//BMI160_RA_MAG_X_M
	//uint8_t MAG_CONF = 0x44;
	//uint8_t FIFO_L = 0x46;
	//uint8_t FIFO_M = 0x47;
	//uint8_t STATUS = 0x1B;
	//uint8_t I2C_MAG = 0x20;
	//uint8_t MAG_DATA_8BYTE = 0x03;
	//uint8_t ADD_BMM_MEASURE = 0x4C; 
	//uint8_t ADD_BMM_DATA = 0x42;
	//};
	struct DeviceAddress {
		uint8_t MLX = 0x3A;
	};
	struct BMM150TrimData {
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
  
	struct PPGSettings {
		uint8_t ledPowerLevel = 0x2F; //Options: 0=Off to 255=50mA
		uint16_t sampleAverage = 16;   //Options: 1, 2, 4, 8, 16, 32
		uint8_t ledMode = 3;          //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
		uint16_t sampleRate = 400;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
		uint16_t pulseWidth = 215;     //Options: 69, 118, 215, 411
		uint16_t adcRange = 4096;     //Options: 2048, 4096, 8192, 16384
	};

	struct IMUSettings {
		uint8_t acc_odr = BMI160AccelRate::BMI160_ACCEL_RATE_25HZ;
		uint8_t acc_bwp = BMI160_DLPF_MODE_NORM;
		uint8_t acc_us = BMI160_DLPF_MODE_NORM;
		uint8_t gyr_odr = BMI160GyroRate::BMI160_GYRO_RATE_25HZ;
		uint8_t gyr_bwp = BMI160_DLPF_MODE_NORM;
		uint8_t gyr_us = BMI160_DLPF_MODE_NORM;
		uint8_t mag_odr = BMI160MagRate::BMI160_MAG_RATE_25HZ;
	};

	struct SamplingRates {
		float eda = 0.f;
		float accelerometer = 0.f;
		float gyroscope = 0.f;
		float magnetometer = 0.f;
		float humidity = 0.f;
		float temperature = 0.f;
		float thermopile = 0.f;
		float ppg = 0.f;
	};

	struct SamplesAveraged {
		uint8_t eda = 1;
		uint8_t humidity = 1;
		uint8_t temperature = 1;
		uint8_t thermopile = 1;
		uint8_t battery = 1;
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
		PPG_INFRARED,
		PPG_RED,
		PPG_GREEN,
		EDA,
		EDL,
		EDR,
		TEMPERATURE_0,
		//THERMOPILE_AMB,
		//THERMOPILE_STO,
		THERMOPILE,
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
		DEBUG,
		length
  };

  enum class DebugTags{
  	WIFI_CONNHISTORY = 1,  // To print wifi connection history till point of do
  	WIFI_DISCONNECT,  // Add message at wifi disconnect 
  	WIFI_TIMELOSTCONN,  // add message indicating time of disconnect
  	WIFI_CONNECT,  // add message indicating established connection
  	WIFI_UPDATECONNRECORDTIME,// NO LONGER IN USE
  	TIME_PARSEINCOMINGMSG = 6,  // time taken to parse incoming message
  	TIME_TIMESTAMPSYNC,  // time taken for time stamp syncing
  	TIME_MSGGENERATION = 8,  // time taken for message generation
  	TIME_MSGTX,  // time taken for comlpete transmission(udp_sdcard)
  	TIME_UDPTX,  // time taken for udp transmission
  	TIME_SDCARDTX,  // time taken for sdcard transmission
  	TIME_FILEOPEN,  // time taken for file opening
  	TIME_FILEWRITES,  // time taken for file write
  	TIME_FILECLOSE,  // time taken for file close
  	TIME_FILESYNC  //time taken for file syncing
  };

  //TODO: Make enum classs for led's
  enum class Led {
	  RED = 1,
	  BLUE,
	  YELLOW
  };

  enum class BattLevel {
	  THRESHOLD_HIGH = 50, // Set thresholds for changing led indication on board for battery
	  //THRESHOLD_MED  = 15,
	  THRESHOLD_LOW  = 20,
	  //INDICATION_SEQ_HIGH = 1,
	  //INDICATION_SEQ_MED,
	  //INDICATION_SEQ_LOW
  };

	enum class PowerMode {
		HIBERNATE,
		WIRELESS_OFF,				// fully shutdown wireless
		MAX_LOW_POWER,			// data not sent, time-syncing accuracy low
		LOW_POWER,					// data not sent, time-syncing accuracy high
		NORMAL_POWER,				// data sending, time-syncing accuracy high
		length
	};

	Si7013 tempHumiditySensor;
	DeviceAddress deviceAddress;
	uint8_t buttonPin;
	PPGSettings ppgSettings;
	IMUSettings imuSettings;
	MAX30105 ppgSensor;
	NCP5623 led;
	MLX90632 thermopile;
	float edrAmplification;
	float vRef1; // Reference voltage of first voltage divider(15/100)
	float vRef2; // Reference voltage from second voltage divider(100/100)
	//float rSkinAmp;
	float adcRes;
	float edaVDivR;
	float edaFeedbackAmpR;
	float edaCrossoverFilterFreq;
	uint8_t _sdCardChipSelectPin;	// ToDo: create getter and make private
	BMM150TrimData bmm150TrimData;
	bool bmm150XYClipped = false;
	bool bmm150ZHallClipped = false;
	uint8_t _hibernatePin;
	bool thermopileBegun = false;
	int thermopileFs = 8; // *** NOTE *** changing this may wear out the Melexis flash
	uint8_t thermopileMode = MODE_STEP;		// If changing to MODE_CONTINUOUS besure to adjust SAMPLING_DIV to match thermopile rate

	// ---------- BEGIN ino refactoring --------------
	static const uint16_t OUT_MESSAGE_RESERVE_SIZE = 2048;
	static const uint16_t OUT_PACKET_MAX_SIZE = 1024;
	static const uint16_t DATA_SEND_INTERVAL = 100;
	static const uint16_t MAX_SD_WRITE_LEN = 512; // 512 is the size of the sdFat buffer
	static const uint16_t MAX_DATA_BUFFER_SIZE = 32;
	static const uint16_t NORMAL_POWER_MODE_PACKET_INTERVAL = 200;
	static const uint16_t LOW_POWER_MODE_PACKET_INTERVAL = 1000;
	uint16_t modePacketInterval = NORMAL_POWER_MODE_PACKET_INTERVAL;

	// Timer constants
#define TIMER_PRESCALER_DIV 1024
	const uint32_t CPU_HZ = 48000000;

	// ToDo: Make sampling variables changeable
#define BASE_SAMPLING_FREQ 300
#define EDA_SAMPLING_DIV 1
#define IMU_SAMPLING_DIV 5
#define PPG_SAMPLING_DIV 5
#define LED_REFRESH_DIV 20
#define THERMOPILE_SAMPLING_DIV 40 	// TODO: This should change according to the rate set on the thermopile begin function 
#define TEMPERATURE_SAMPLING_DIV 10
#define BATTERY_SAMPLING_DIV 50
#define DUMMY_ISR_DIV 20

	struct TimerLoopOffset
	{
		uint8_t eda = 0;
		uint8_t imu = 0;
		uint8_t ppg = 1;
		uint8_t led = 4;
		uint8_t thermopile = 6;
		uint8_t tempHumidity = 2;
		uint8_t battery = 0;
	} timerLoopOffset;	// Sets initial value of sampling counters


//// ToDo: Make sampling variables changeable
//#define BASE_SAMPLING_FREQ 120
//#define EDA_SAMPLING_DIV 1
//#define IMU_SAMPLING_DIV 4
//#define PPG_SAMPLING_DIV 4
//#define LED_REFRESH_DIV 8
//#define THERMOPILE_SAMPLING_DIV 16	// TODO: This should change according to the rate set on the thermopile begin function 
//#define TEMPERATURE_SAMPLING_DIV 4
//#define BATTERY_SAMPLING_DIV 20

	//struct TimerLoopOffset
	//{
	//	uint8_t eda = 0;
	//	uint8_t imu = 3;
	//	uint8_t ppg = 1;
	//	uint8_t led = 2;
	//	uint8_t thermopile = 6;
	//	uint8_t tempHumidity = 0;
	//	uint8_t battery = 0;
	//} timerLoopOffset;	// Sets initial value of sampling counters

	struct AcquireData {
		bool eda = true;
		bool tempHumidity = true;
		bool thermopile = true;
		bool imu = true;
		bool ppg = true;
		bool debug = false;
		bool battery = true;
	} acquireData;

	struct ChipBegun {
		bool SI7013 = false;
		bool MAX30101 = false;
		bool BMI160 = false;
		bool BMM150 = false;
		bool NCP5623 = false;
		bool MLX90632 = false;
	} chipBegun;

	const char *typeTags[(uint8_t)EmotiBit::DataType::length];
	bool _newDataAvailable[(uint8_t)EmotiBit::DataType::length];
	uint8_t printLen[(uint8_t)EmotiBit::DataType::length];
	bool sendData[(uint8_t)EmotiBit::DataType::length];

	SdFat SD;
	volatile uint8_t battLevel = 100;
	volatile uint8_t battIndicationSeq = 0;
	volatile uint16_t BattLedDuration = 65535;

	EmotiBitWiFi _emotiBitWiFi; 
	TwoWire* _EmotiBit_i2c = nullptr;
	String _outDataPackets;		// Packets that will be sent over wireless (if enabled) and written to SD card (if recording)
	uint16_t _outDataPacketCounter = 0;
	//String _outSdPackets;		// Packts that will be written to SD card (if recording) but not sent over wireless
	//String _inControlPackets;	// Control packets recieved over wireless
	String _sdCardFilename = "datalog.csv";
	const char *_configFilename = "config.txt"; 
	File _dataFile;
	volatile bool _sdWrite;
	PowerMode _powerMode;
	bool _sendTestData = false;
	float _edlDigFiltAlpha = 0;
	float _edlDigFilteredVal = -1;
	DataType _serialData = DataType::length;
	volatile bool buttonPressed = false;

	void setupSdCard();
	void updateButtonPress();
	void hibernate();
	void startTimer(int frequencyHz);
	void setTimerFrequency(int frequencyHz);
	//void TC3_Handler();
	void stopTimer();
	void(*onShortPressCallback)(void);
	void(*onLongPressCallback)(void);
	void(*onDataReadyCallback)(void);
	void attachShortButtonPress(void(*shortButtonPressFunction)(void));
	void attachLongButtonPress(void(*longButtonPressFunction)(void));
	PowerMode getPowerMode();
	void setPowerMode(PowerMode mode);
	bool writeSdCardMessage(const String &s);
	int freeMemory();
	bool loadConfigFile(const String &filename);
	bool addPacket(uint32_t timestamp, EmotiBit::DataType t, float * data, size_t dataLen, uint8_t precision = 4);
	bool addPacket(EmotiBit::DataType t);
	void parseIncomingControlPackets(String &controlPackets, uint16_t &packetNumber);
	void readSensors();
	void writeSerialData(EmotiBit::DataType t);
	

	/**
	 * Copies data buffer of the specified DataType into the passed array
	 *
	 * @param t an EmotiBit::DataType to read
	 * @data a pre-allocated array of size dataSize to copy data into
	 * @dataSize a size_t specifying the size of the passed data array
	 * @return the size of the present data buffer
	 */
	size_t readData(EmotiBit::DataType t, float *data, size_t dataSize);
	
	/**
	 * Copies data buffer and timestamp of the specified DataType into the passed parameters
	 *
	 * @param t an EmotiBit::DataType to read
	 * @data a pre-allocated array of size dataSize to copy data into
	 * @dataSize a size_t specifying the size of the passed data array
	 * @timestamp returns with the most recent data timestamp
	 * @return the size of the present data buffer
	 */
	size_t readData(EmotiBit::DataType t, float *data, size_t dataSize, uint32_t &timestamp);		
	
	/**
	 * Points to the data buffer of the specified type into the passed parameter
	 *		CAUTION: this pointer becomes stale after calling EmotiBit::update()
	 *
	 * @param t an EmotiBit::DataType to read
	 * @data returns with a pointer to the existing (double buffered) float[]
	 * @return the size of the present data buffer 
	 */
	size_t readData(EmotiBit::DataType t, float **data);
	
	/**
	 * Points to the data buffer and reads the timestamp of the specified type into the passed parameters
	 *		CAUTION: this pointer becomes stale after calling EmotiBit::update()
	 *
	 * @param t an EmotiBit::DataType to read
	 * @data returns with a pointer to the existing (double buffered) float[]
	 * @timestamp returns with the most recent timestamp in the present data array
	 * @return the size of the present data buffer
	 */
	size_t readData(EmotiBit::DataType t, float **data, uint32_t &timestamp);	

	void updateBatteryIndication();
	void appendTestData(String &dataMessage, uint16_t &packetNumber);
	bool createModePacket(String &modePacket, uint16_t &packetNumber);
	void sendModePacket(String &sentModePacket, uint16_t &packetNumber);
	void processDebugInputs(String &debugPackets, uint16_t &packetNumber);
	String getHardwareVersion();

	// ----------- END ino refactoring ---------------

	
  EmotiBit();
  uint8_t setup(Version version = Version::V02H, size_t bufferCapacity = 64);   /**< Setup all the sensors */
	uint8_t update();
  void setAnalogEnablePin(uint8_t i); /**< Power on/off the analog circuitry */
  int8_t updateIMUData();                /**< Read any available IMU data from the sensor FIFO into the inputBuffers */
	int8_t updatePPGData();                /**< Read any available PPG data from the sensor FIFO into the inputBuffers */
	int8_t updateEDAData();                /**< Take EDA reading and put into the inputBuffer */
	int8_t updateTempHumidityData();       /**< Read any available temperature and humidity data into the inputBuffers */
	int8_t updateThermopileData();         /**< Read Thermopile data into the buffers*/
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
	bool printConfigInfo(File &file, const String &datetimeString);
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
	uint8_t _edlPin;
	uint8_t _edrPin;
	float _vcc;
	uint8_t _adcBits;
	float _accelerometerRange; // supported values: 2, 4, 8, 16 (G)
	float _gyroRange; // supported values: 125, 250, 500, 1000, 2000 (degrees/second)
	Version _version;
	uint8_t _imuFifoFrameLen = 0; // in bytes
	const uint8_t _maxImuFifoFrameLen = 40; // in bytes
	uint8_t _imuBuffer[40];


	DoubleBufferFloat eda			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat edl			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat edr			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat ppgInfrared	 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE);
	DoubleBufferFloat ppgRed		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE);
	DoubleBufferFloat ppgGreen		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE);
	DoubleBufferFloat temp0			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 4);
	// DoubleBufferFloat tempHP0 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 4);
	DoubleBufferFloat therm0		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 4); // To store the thermistor Object Temp
	DoubleBufferFloat therm0AMB		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 4); // To store the raw value AMB from the thermistor
	DoubleBufferFloat therm0Sto		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 4); // To store the raw Value Sto from the thermistor
	DoubleBufferFloat humidity0		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 4);
	DoubleBufferFloat accelX		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat accelY		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat accelZ		 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat gyroX			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat gyroY			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat gyroZ			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat magX			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat magY			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat magZ			 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE / 2);
	DoubleBufferFloat batteryVoltage = DoubleBufferFloat(1);
	DoubleBufferFloat batteryPercent = DoubleBufferFloat(1);
	DoubleBufferFloat dataOverflow	 = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE);
	DoubleBufferFloat dataClipping   = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE);
	DoubleBufferFloat debugBuffer    = DoubleBufferFloat(MAX_DATA_BUFFER_SIZE);

	DoubleBufferFloat * dataDoubleBuffers[(uint8_t)DataType::length];

	// Oversampling buffers
	// Single buffered arrays must only be accessed from ISR functions, not in the main loop
	// ToDo: add assignment for dynamic allocation;
	// 	**** WARNING **** THIS MUST MATCH THE SAMPLING DIVS ETC
	BufferFloat edlBuffer = BufferFloat(24);
	BufferFloat edrBuffer = BufferFloat(24);	
	BufferFloat temperatureBuffer = BufferFloat(4);	
	BufferFloat humidityBuffer = BufferFloat(4);
	BufferFloat batteryVoltageBuffer = BufferFloat(8);
	BufferFloat batteryPercentBuffer = BufferFloat(8);

	const uint8_t SCOPE_TEST_PIN = A0;
	bool scopeTestPinOn = false;

};

void attachEmotiBit(EmotiBit*e = nullptr);
void attachToInterruptTC3(void(*readFunction)(void), EmotiBit*e = nullptr);
void ReadSensors();

#endif
