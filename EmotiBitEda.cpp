/**************************************************************************/
/*!
    @file     EmotiBitEda.cpp
    @author   Sean Montgomery (EmotiBit)

    @mainpage Electrodermal activity (EDA) handler for EmotiBit

    @section intro_sec Introduction

    This is a library to handle electrodermal activity (EDA) on EmotiBit.

		EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    @section author Author

    Written by Sean Montgomery for EmotiBit.

    @section  HISTORY

    v1.0  - First release

    @section license License

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/

#include "EmotiBitEda.h"

bool EmotiBitEDA::setup(EmotiBitVersionController::EmotiBitVersion version, float samplingRate, 
	const DoubleBufferFloat* edaBuffer, const DoubleBufferFloat* edlBuffer, const DoubleBufferFloat* edrBuffer,
	const TwoWire* emotibitI2c, const BufferFloat* edlOversampBuffer, const BufferFloat* edrOversampBuffer);
{
	_emotibitVersion = version;
	_constants.samplingRate = samplingRate;

	// ToDo: Calculate digFiltAlpha

	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{	
		_edaBuffer = edaBuffer;
		_edlBuffer = edlBuffer;
		_edrBuffer = edrBuffer;
		_edlOversampBuffer = edlOversampBuffer;
		_edlOversampBuffer = edrOversampBuffer;

		_constants.adcBits = 16;

		// NOTE: if these values are changed in code, we should add parameters to _info.json
		ads.setDataRate(RATE_ADS1115_475SPS);	// set to 475Hz to allow for 300Hz oversampling
		ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV

		return ads.begin(0x48, emotibit_i2c, false); // callBegin -> false. the i2c wire has already been init in setup
	}
	else if (_emotibitVersion <= EmotiBitVersionController::EmotiBitVersion::V03B)
	{
		_edaBuffer = edaBuffer;
		_edlBuffer = edlBuffer;
		_edrBuffer = edrBuffer;
		_edlOversampBuffer = edlOversampBuffer;
		_edlOversampBuffer = edrOversampBuffer;

		_constants.adcBits = 12;
		_constants_v2_v3.adcRes = 2 ^ _constants.adcBits;
		
		return true;
	}

	return false;
}


bool EmotiBitEDA::storeCalibration(MemoryController * memoryController, String &calibrationRawValues, bool asyncRW)
{
	if (_emotiBitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{

	}
	else
	{
		// Storing calibration data on V02/V03 HW requires firmware v1.2.86 
		return false;
	}

	struct CalibrationPair
	{
		float res;
		float adcVal;
	};

	static const unsigned int nCalibVals_V2 = 5;
	struct ECalibrationRawValues_V2
	{
		unsigned int nVals = nCalibVals_V2;
		CalibrationPair vals[nCalibVals_V2];
	};
}

