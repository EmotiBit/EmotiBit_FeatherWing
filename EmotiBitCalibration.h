#pragma once
#include "Arduino.h"

class EmotiBitCalibration {


public:

	enum class SensorType {
		GSR,
		// Add sensors for calibration below this.
		length
	};
	const char* calibrationTag[(uint8_t)SensorType::length];
	EmotiBitCalibration();

	uint8_t calibrationPointsPerSensor[(uint8_t)SensorType::length];
	
private:
	// An array to enable/disable sensor calibration 
	bool sensorsToCalibrate[(uint8_t)SensorType::length];
	
public:
	class GsrCalibration {
		
	public: 
		GsrCalibration();
	private:
		bool calibrationStatus;
		
	public:
		float edlCumSum;
		uint16_t edlAvgCount;
		const uint16_t MAX_EDL_AVG_COUNT = 1000;
	
	public:
		void performCalibration(float newVal);
		void setCalibrationStatus();
		bool isCalibrated();
		float getCalibratedValue();
	} gsrcalibration;

public:

	void setSensorToCalibrate(EmotiBitCalibration::SensorType sensor);
	bool getSensorToCalibrate(EmotiBitCalibration::SensorType sensor);
	bool sendCalibrationData();
};