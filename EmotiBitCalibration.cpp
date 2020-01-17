#include "EmotiBitCalibration.h"
//#include "EmotiBit.h"
EmotiBitCalibration::EmotiBitCalibration()
{
	sensorsToCalibrate[(uint8_t)EmotiBitCalibration::SensorType::GSR] = false;
	// TODO : set it to false for all other sensors
	// calibrationTags[(uint8_t)SensorType::GSR] = "GSR_C\0";
	calibrationTag[(uint8_t)SensorType::GSR] = "CG\0"; // Calibrate Gsr
	// TODO: for all other sensor types
	calibrationPointsPerSensor[(uint8_t)SensorType::GSR] = 1;
}
EmotiBitCalibration::GsrCalibration::GsrCalibration()
{
	edlCumSum = 0;
	edlAvgCount = 1;
	calibrationStatus = false;
}


void EmotiBitCalibration::GsrCalibration::setCalibrationStatus()
{
	calibrationStatus = true;
}

bool EmotiBitCalibration::GsrCalibration::isCalibrated()
{
	return calibrationStatus;
}


void EmotiBitCalibration::GsrCalibration::performCalibration(float newVal)
{
	if (edlAvgCount < MAX_EDL_AVG_COUNT)
	{
		// add the cummulative sum
		edlCumSum += newVal;
		edlAvgCount++;
		Serial.print("updating:");
		Serial.println(edlAvgCount);
	}
	else if (edlAvgCount == MAX_EDL_AVG_COUNT)
	{
		// take the average
		edlCumSum = edlCumSum / edlAvgCount;
		edlAvgCount++;
		Serial.println("updated");
	}
	else
	{
		setCalibrationStatus();
	}
}

float EmotiBitCalibration::GsrCalibration::getCalibratedValue()
{
	if (isCalibrated())
	{
		return edlCumSum;
	}
	else
	{
		return -1; // indicates data not ready
	}
}


void EmotiBitCalibration::setSensorToCalibrate(EmotiBitCalibration::SensorType sensor)
{
	if (sensor == EmotiBitCalibration::SensorType::GSR)
	{
		sensorsToCalibrate[(uint8_t)sensor] = true;
	}
	// TODO: other statements for other sensor calibrations
}

bool EmotiBitCalibration::getSensorToCalibrate(EmotiBitCalibration::SensorType sensor)
{
	return sensorsToCalibrate[(uint8_t)sensor];
}