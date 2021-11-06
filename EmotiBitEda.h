/**************************************************************************/
/*!
    @file     EmotiBitEda.h

    This is a library to handle electrodermal activity (EDA) on EmotiBit.

    EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    Written by Sean Montgomery for EmotiBit.

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/

#include "Arduino.h"

#ifndef _EMOTIBIT_EDA_H_
#define _EMOTIBIT_EDA_H_


#include "EmotiBitVersionController.h"
#include "EmotiBitNvmController.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include "Adafruit_ADS1X15.h"
#include <math.h> 
#include "DoubleBufferFloat.h"
#include "DigitalFilter.h"



class EmotiBitEda
{
private:
	EmotiBitVersionController::EmotiBitVersion _emotibitVersion;	
	uint8_t _calibRawValuesVersion = 0;
	DoubleBufferFloat* _edaBuffer = nullptr;
	DoubleBufferFloat* _edlBuffer = nullptr;
	DoubleBufferFloat* _edrBuffer = nullptr;
	BufferFloat* _edlOversampBuffer = nullptr;
	BufferFloat* _edrOversampBuffer = nullptr;
	bool isCalibrated = false;
	volatile bool _readFinished = false;
	
public:
		 
	// EDA constants shared by different EmotiBit versions
	struct Constants
	{
		float edaSeriesResistance = 0.f;	//Ohms
		float samplingRate = 15.f;	// Hz
		uint8_t adcBits;	// Bit resolution of ADC, e.g. 12, 16
		bool enableDigitalFilter = false;
	} _constants;

	// V2/V3 specific EDA constants
	struct Constants_V2_V3
	{
		float vcc = 3.3f;	// Volts
		float edrAmplification = 30.f;	// Hardware amplification at EDR stage
		float vRef1 = 15.f / 100.f;	// Volts
		float vRef2 = 2.65f;	// Volts
		float feedbackAmpR = 5000000; // Ohms
		float crossoverFilterFreq = 1.f / (2.f * PI * 200000.f * 0.0000047f);	// Hz
		int adcRes;	// 2^adcBits
		uint8_t edlPin = A4;	// ADC pin to read for EDL
		uint8_t edrPin = A3;	// ADC pin to read for EDR
		float isrOffsetCorr = 0.f;	// Correction for ADC value changes when ISR running
	} _constants_v2_v3;

	struct Constants_V4_plus
	{
		float edaTransformSlope = 728.8406523;
		float edaTransformIntercept = 14179797.05;
	} _constants_v4_plus;
	
	Adafruit_ADS1115 _ads;

	/*!
			@brief  Sets up EmotiBit to read EDA data
			@param version EmotiBit version
			@param samplingRate Nominal output sampling rate in Hz. Determines filtering and info.json params.
			@param edaBuffer DoubleBuffer for collecting eda data
			@param edlBuffer DoubleBuffer for collecting edl data
			@param edrBuffer DoubleBuffer for collecting edr data
			@param emotibitI2c EmotiBit I2C bus instance
			@param edlOversampBuffer Buffer for collecting oversampled edl data
			@param edrOversampBuffer Buffer for collecting oversamplededr data
			@return true if successful, otherwise false
	*/
	bool setup(EmotiBitVersionController::EmotiBitVersion version, float samplingRate, 
		DoubleBufferFloat* edaBuffer, DoubleBufferFloat* edlBuffer, DoubleBufferFloat* edrBuffer,
		TwoWire* emotibitI2c = nullptr, BufferFloat* edlOversampBuffer = nullptr, BufferFloat* edrOversampBuffer = nullptr);
	
	/*!
		@brief  ISR fn to read ADC values for EDA and stores the raw data in buffers passed during setup
		@return true if successful, otherwise false
	*/
	uint8_t readData();

	/*!
		@brief  Main loop fn to process raw ADC units into meaningful units (uSiemens, Volts, etc)
		@return true if successful, otherwise false
	*/
	bool processData();

	/*!
		@brief Writes EDA portion of _info.json file with relevant parameters
		@return true if successful, otherwise false
	*/
	bool writeInfoJson(File &jsonFile);

	/*!
		@brief Loads & calculates EDA calibration from the on-board storage
		@return true if successful, otherwise false
	*/
	bool stageCalibLoad(EmotiBitNvmController * nvmController, bool autoSync = false);

	/*!
		@brief Stores calibration values using on-board storage
		@return true if successful, otherwise false
	*/
	bool stageCalibStorage(EmotiBitNvmController * nvmController, String &edaCalibPacket, bool autoSync = false);

	/*!
		@brief Sets ISR offset correction for ADC
	*/
	void setAdcIsrOffsetCorr(float isrOffsetCorr);
};

#endif