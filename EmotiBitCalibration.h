#pragma once

//#include "DoubleBufferFloat.h"
//#include <ArduinoJson.h>
//#include <SdFat.h>
//#include "wiring_private.h"
//#include "EmotiBitWiFi.h"
//#include <SPI.h>
//#include <SdFat.h>
//#include <ArduinoJson.h>
//#include <ArduinoLowPower.h>


class EmotiBitCalibration {

public:
	EmotiBitCalibration();
	
	enum class SensorType {

		GSR,
		length
	};
	
	// An array to enable/disable sensor calibration 
	bool calibrateSensor[(uint8_t)SensorType::length];
	// typeTags to be added to the Output messages
	const char *typeTags[(uint8_t)SensorType::length];
	String _outDataPackets;

	class GsrCalibration {
		
	public:
		GsrCalibration();
		float edlCumSum;
		uint16_t edlAvgCount;
		const uint16_t MAX_EDL_AVG_COUNT = 1000;
	
	public:
		void performCalibration(float newVal);
		void sendGsrCalibration();
	} gsrcalibration;

public:

	// void calibrateSensor(EmotiBitCalibration::SensorType sensor, float edlVal);
	void setSensorToCalibrate(EmotiBitCalibration::SensorType sensor);
	bool getSensorToCalibrate(EmotiBitCalibration::SensorType sensor);
	bool sendCalibrationData();
};