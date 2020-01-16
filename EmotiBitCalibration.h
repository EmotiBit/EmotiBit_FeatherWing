#pragma once

#include "DoubleBufferFloat.h"
#include <ArduinoJson.h>
#include <SdFat.h>
#include "wiring_private.h"
#include "EmotiBitWiFi.h"
#include <SPI.h>
#include <SdFat.h>
#include <ArduinoJson.h>
#include <ArduinoLowPower.h>


class EmotiBitCalibration {

public:
	EmotiBitCalibration();
	
	enum class SensorType {

		GSR,
		length
	};
	bool calibrateSensor[SensorType::length];
	const char *typeTags[(uint8_t)SensorType::length];
	String _outDataPackets;
private:
	class GsrCalibration {
		
	public:
		GsrCalibration();
		float edlCumSum;
		uint16_t edlAvgCount;
		const uint16_t MAX_EDL_AVG_COUNT = 1000;
	
	public:
		void calibrateGsr();
		void sendGsrCalibration();
	} gsrcalibration;

public:

	void calibrateSensor(EmotiBitCalibration::SensorType sensor, float edlVal);
	void setSensorToCalibrate(EmotiBitCalibration::SensorType s);
	bool sendCalibrationData();
};