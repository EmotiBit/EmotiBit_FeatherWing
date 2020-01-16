#include "EmotiBitCalibration.h"

EmotiBitCalibration::EmotiBitCalibration()
{
	calibrateSensor[(uint8_t)EmotiBitCalibration::SensorType::GSR] = false;
	// TODO : set it to false for all other sensors
	typeTags[(uint8_t)SensorType::GSR] = "GSR\0";

	// TODO: for all other sensor types
}
void EmotiBitCalibration::GsrCalibration::GsrCalibration()
{
	edlCumSum = 0;
	edlAvgCount = 1;
}

void EmotiBitCalibration::GsrCalibration::calibrateGsr()
{
	if (edlAvgCount < MAX_EDL_AVG_COUNT)
	{
		// add the cummulative sum
		edlCumSum += newVal;
		edlAvgCount++;
		Serial.println("updating");
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
		// send this data over to the computer
		// set "calibrated tag"
		Serial.println("done with data");
		//sendGsrCalibration();
	}
}

void EmotiBitCalibration::GsrCalibration::sendGsrCalibration()
{

	//_emotiBitWiFi.sendData(_outDataPackets);
}

void EmotiBitCalibration::calibrateSensor(EmotiBitCalibration::SensorType sensor, float edlVal)
{
	if (sensor == EmotiBitCalibration::SensorType::GSR)
	{
		gsrcalibration.calibrateGsr();
	}
}

void EmotiBitCalibration::setSensorToCalibrate(EmotiBitCalibration::SensorType sensor)
{
	if (sensor == EmotiBitCalibration::SensorType::GSR)
	{
		calibrateSensor[(uint8_t)s] = true;
	}
	// TODO: other statements for other sensor calibrations
}
