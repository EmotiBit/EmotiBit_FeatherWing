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
#include "EmotiBitEdaCalibration.h"

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


bool EmotiBitEDA::stageCalibStorage(MemoryController * memoryController, String &edaCalibPacket, bool waitForWrite)
{
	if (_emotiBitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		uint8_t dataVersion;
		if (unpackCalibPacket(edaCalibPacket, dataVersion, rawVals))
		{
			if (dataVersion == EmotiBitEdaCalibration::V2)
			{
				EmotiBitEdaCalibration::RawValues_V2 rawVals;
				uint8_t status = memoryController->stageForWrite(EmotiBitMemoryController::DataType::EDA, dataVersion, sizeof(EmotiBitEdaCalibration::RawValues_V2), (uint8_t *)(&rawVals), waitForWrite);
				if (status != MemoryController::Status::SUCCESS)
				{
					return false;
				}
			}
			else
			{
				// Write cases for other dataVersions
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	else
	{
		// Storing calibration data on V02/V03 HW requires firmware v1.2.86 
		return false;
	}
	return true;
}


bool EmotiBitEDA::stageCalibLoad(MemoryController * memoryController, bool waitForRead = false)
{
	uint8_t dataVersion;
	uint8_t dataSize;
	uint8_t* data;

	memoryController->stageForRead(EmotiBitMemoryController::DataType::EDA, dataVersion, dataSize, data, waitForRead);

	if (dataVersion == EmotiBitEdaCalibration::V2)
	{
		EmotiBitEdaCalibration::RawValues_V2 *rawVals = (EmotiBitEdaCalibration::RawValues_V2 *)data;
		EmotiBitEdaCalibration::calculate(*rawVals, _constants_v4_plus.edaTransformSlope, _constants_v4_plus.edaTransformIntercept);
	}
	else if (dataVersion == EmotiBitEdaCalibration::V0)
	{
		EmotiBitEdaCalibration::RawValues_V0 *rawVals = (EmotiBitEdaCalibration::RawValues_V0 *)data;
		EmotiBitEdaCalibration::calculate(*rawVals, _constants_v2_v3.vRef1, _constants_v2_v3.vRef2, _constants_v2_v3.feedbackAmpR);
	}
	else
	{
		return false;
	}

	delete data;

	return true;
}

bool EmotiBitEDA::readData()
{
#ifdef DEBUG
		Serial.println("EmotiBitEDA::readData()");
#endif // DEBUG
		static float edaTemp;

		if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			// Reads EDA data from ADS1113
			if (emotibitEda.ads.conversionComplete())
			{
				edaTemp = emotibitEda.ads.getLastConversionResults();
				emotibitEda.ads.startADC_Differential_0_1();

				// ToDo: remove oversampling and utilize delta-sigma ADC for averaging
				_edlOversampBuffer->push_back(edaTemp);
			}
			if (edlBuffer.size() == _samplesAveraged.eda)
			{

			}

			// ToDo: handle clipping

		}

		int8_t status = 0;
		static float edaTemp;	// Electrodermal Activity in Volts
		static bool edlClipped = false;

		if (_version > EmotiBitVersionController::EmotiBitVersion::V03B)
		{
			// Reads EDA data from ADS1113
			if (emotibitEda.ads.conversionComplete())
			{
				edaTemp = emotibitEda.ads.getLastConversionResults();
				emotibitEda.ads.startADC_Differential_0_1();
				// ToDo: Add clipping checks

				// ToDo: consider how to utilize edl & edr buffers for different EmotiBit versions to minimize RAM footprint & code clarity
				edlBuffer.push_back(edaTemp);
			}
			if (edlBuffer.size() == _samplesAveraged.eda)
			{
				// Perform data averaging
				edaTemp = average(edlBuffer);

				// ToDo: Add conversion from ADC units to uSiemens conditional on isCalibrated

				// Add to data double buffer
				status = status | pushData(EmotiBit::DataType::EDA, edaTemp, &edlBuffer.timestamp);
				if (edlClipped) {
					pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDA, &edlBuffer.timestamp);
				}

				// Clear the averaging buffers
				edlBuffer.clear();
				edlClipped = false;
			}
		}
		else
		{
			// Reads EDA data from SAMD21 ADC


			static float edlTemp;	// Electrodermal Level in Volts
			static float edrTemp;	// Electrodermal Response in Volts
			static float sclTemp;	// Skin Conductance Level in uSeimens
			static float scrTemp;	// Skin Conductance Response in uSeimens
			static bool edrClipped = false;

			// ToDo: Optimize calculations for EDA

			// Check EDL and EDR voltages for saturation
			edlTemp = analogRead(_edlPin);
			edrTemp = analogRead(_edrPin);

			// Correct for offset correction only when not in ISR_CORRECTION_UPDATE
			if (testingMode != TestingMode::ISR_CORRECTION_UPDATE)
			{
				edlTemp = edlTemp - _isrOffsetCorr;
				edrTemp = edrTemp - _isrOffsetCorr;
			}

			// Add data to buffer for sample averaging (oversampling)
			edlBuffer.push_back(edlTemp);
			edrBuffer.push_back(edrTemp);

			// ToDo: move adc clipping limits to setup()
			static const int adcClippingLowerLim = 20;
			static const int adcClippingUpperLim = adcRes - 20;

			// Check for data clipping
			if (edlTemp < adcClippingLowerLim || edlTemp > adcClippingUpperLim)
			{
				edlClipped = true;
				status = status | (int8_t)Error::DATA_CLIPPING;
			}
			// Check for data clipping
			if (edrTemp < adcClippingLowerLim || edrTemp > adcClippingUpperLim)
			{
				edrClipped = true;
				status = status | (int8_t)Error::DATA_CLIPPING;
			}

			if (edlBuffer.size() == _samplesAveraged.eda) {
				// Perform data averaging
				edlTemp = average(edlBuffer);
				edrTemp = average(edrBuffer);

				if (testingMode == TestingMode::ISR_CORRECTION_UPDATE || testingMode == TestingMode::ISR_CORRECTION_TEST)
				{
					// Transmit raw ADC values
				}
				else
				{
					// transmit converted to volts
					// Perform data conversion
					edlTemp = edlTemp * _vcc / adcRes;	// Convert ADC to Volts
					edrTemp = edrTemp * _vcc / adcRes;	// Convert ADC to Volts
				}

				// send raw EDL values
				pushData(EmotiBit::DataType::EDL, edlTemp, &edlBuffer.timestamp);
				if (edlClipped) {
					pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDL, &edlBuffer.timestamp);
				}

				pushData(EmotiBit::DataType::EDR, edrTemp, &edrBuffer.timestamp);
				if (edrClipped) {
					pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDR, &edrBuffer.timestamp);
				}


				// EDL Digital Filter
				if (edaCrossoverFilterFreq > 0)// use only is a valid crossover freq is assigned
				{
					static DigitalFilter filterEda(DigitalFilter::FilterType::IIR_LOWPASS, (_samplingRates.eda / _samplesAveraged.eda), edaCrossoverFilterFreq);
					if (_enableDigitalFilter.eda)
					{
						edlTemp = filterEda.filter(edlTemp);
					}
				}
				// Link to diff amp biasing: https://ocw.mit.edu/courses/media-arts-and-sciences/mas-836-sensor-technologies-for-interactive-environments-spring-2011/readings/MITMAS_836S11_read02_bias.pdf
				edaTemp = (edrTemp - vRef2) / edrAmplification;	// Remove VGND bias and amplification from EDR measurement
				edaTemp = edaTemp + edlTemp;                     // Add EDR to EDL in Volts

				//edaTemp = (_vcc - edaTemp) / edaVDivR * 1000000.f;						// Convert EDA voltage to uSeimens

				if (edaTemp - vRef1 < 0.000086f)
				{
					edaTemp = 0.001f; // Clamp the EDA measurement at 1K Ohm (0.001 Siemens)
				}
				else
				{
					edaTemp = vRef1 / ((edaFeedbackAmpR * (edaTemp - vRef1)) - (_edaSeriesResistance * vRef1));
				}

				edaTemp = edaTemp * 1000000.f; // Convert to uSiemens


				// Add to data double buffer
				status = status | pushData(EmotiBit::DataType::EDA, edaTemp, &edrBuffer.timestamp);
				if (edlClipped || edrClipped) {
					pushData(EmotiBit::DataType::DATA_CLIPPING, (uint8_t)EmotiBit::DataType::EDA, &edrBuffer.timestamp);
				}

				// Clear the averaging buffers
				edlBuffer.clear();
				edrBuffer.clear();
				edlClipped = false;
				edrClipped = false;
			}
		}

		// ToDo: Consider moving calculation into getData

		// EDA -> Iskin
		//tempEda = edaLow + (edaHigh / edaHPAmplification);
		//tempEda = (1.f - tempEda / _adcRes) * _vcc / _edaVDivR; // Iskin calculation
		////tempEda = tempEda * _edaVDivR / (_adcRes - _edaVDivR); // Rskin calculation
		//if (eda.push_back(tempEda) == BufferFloat::ERROR_BUFFER_OVERFLOW) {
		//	status = (int8_t)Error::BUFFER_OVERFLOW;
		//}

		////// EDL, EDR -> Rskin
		//tempEda = edaLow;
		////tempEda = tempEda * _edaVDivR / (_adcRes - _edaVDivR); // Rskin calculation
		//tempEda = tempEda * _vcc / _adcRes;
		//if (edl.push_back(tempEda) == BufferFloat::ERROR_BUFFER_OVERFLOW) {
		//	status = (int8_t)Error::BUFFER_OVERFLOW;
		//}
		//tempEda = edaHigh;
		////tempEda = tempEda * _edaVDivR / (_adcRes - _edaVDivR); // Rskin calculation
		//tempEda = tempEda * _vcc / _adcRes;
		//if (edr.push_back(tempEda) == BufferFloat::ERROR_BUFFER_OVERFLOW) {
		//	status = (int8_t)Error::BUFFER_OVERFLOW;
		//}

		return status;
}


bool EmotiBitEDA::processData()
{

}

bool EmotiBitEDA::writeInfoJson(File * jsonFile)
{

}

