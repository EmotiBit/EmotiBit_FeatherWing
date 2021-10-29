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
#ifndef _EMOTIBIT_EDA_H_
#define _EMOTIBIT_EDA_H_


#include "EmotiBitVersionController.h"
#include <Wire.h>
#include "Adafruit_ADS1X15.h"



class EmotiBitEda
{
private:
	EmotiBitVersionController::EmotiBitVersion _emotibitVersion;	
	uint8_t _calibRawValuesVersion = 0;
	DoubleBufferFloat* _edaBuffer = nullptr;
	DoubleBufferFloat* _edlBuffer = nullptr;
	DoubleBufferFloat* _edrBuffer = nullptr;
	BufferFloat* _edlOversampBuffer = nullptr;
	BufferFloat* _edlOversampBuffer = nullptr;
	bool isCalibrated = false;ve
	
public:
		 
	// EDA constants shared by different EmotiBit versions
	struct Constants
	{
		float edaSeriesResistance = 0;	//Ohms
		float samplingRate = 15.0;	// Hz
		uint8_t adcBits;	// Bit resolution of ADC, e.g. 12, 16
		bool enableDigitalFilter = false;
	} _constants;

	// V2/V3 specific EDA constants
	struct Constants_V2_V3
	{
		float vcc = 3.3;	// Volts
		float edrAmplification = 30;	// Hardware amplification at EDR stage
		float vRef1 = 15.0 / 100.0;	// Volts
		float vRef2 = 2.65;	// Volts
		float feedbackAmpR = 5000000; // Ohms
		float crossoverFilterFreq = 1.f / (2.f * PI * 200000.f * 0.0000047f);	// Hz
		int adcRes;	// 2^adcBits
		uint8_t edlPin = A4;	// ADC pin to read for EDL
		uint8_t edrPin = A3;	// ADC pin to read for EDR
	} _constants_v2_v3;

	struct Constants_V4_plus
	{
		float edaTransformSlope;
		float edaTransformIntercept;
	} _constants_v4_plus;
	
	Adafruit_ADS1115 _ads;

	/*!
			@brief  Sets up EmotiBit to read EDA data
			@param version of EmotiBit
			@param sampling rate (determines filtering and info.json params)
			@param DoubleBuffer for collecting eda data
			@param DoubleBuffer for collecting edl data
			@param DoubleBuffer for collecting edr data
			@param emotibit_i2c I2C bus
			@param Buffer for collecting oversampled edl data
			@param Buffer for collecting oversamplededr data

			@return true if successful, otherwise false
	*/
	bool setup(EmotiBitVersionController::EmotiBitVersion version, float samplingRate, 
		const DoubleBufferFloat* edaBuffer, const DoubleBufferFloat* edlBuffer, const DoubleBufferFloat* edrBuffer,
		const TwoWire* emotibitI2c = nullptr, const BufferFloat* edlOversampBuffer = nullptr, const BufferFloat* edrOversampBuffer = nullptr);
	
	/*!
		@brief  ISR fn to read ADC values for EDA and stores the raw data in buffers passed during setup
		@return true if successful, otherwise false
	*/
	bool readData();

	/*!
		@brief  Main loop fn to process raw ADC units into meaningful units (uSiemens, Volts, etc)
		@return true if successful, otherwise false
	*/
	bool processData();

	/*!
		@brief Writes EDA portion of _info.json file with relevant parameters
		@return true if successful, otherwise false
	*/
	bool writeInfoJson(File * jsonFile);

	/*!
		@brief Loads & calculates EDA calibration from the on-board storage
		@return true if successful, otherwise false
	*/
	bool stageCalibLoad(MemoryController * memoryController, bool waitForRead = false);

	/*!
		@brief Stores calibration values using on-board storage
		@return true if successful, otherwise false
	*/
	bool stageCalibStorage(MemoryController * memoryController, String &calibrationRawValues, bool waitForWrite);

	
};

#endif