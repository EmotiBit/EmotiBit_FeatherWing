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

bool EmotiBitEda::setup(EmotiBitVersionController::EmotiBitVersion version, float samplingRate,
	DoubleBufferFloat* edaBuffer, DoubleBufferFloat* edlBuffer, DoubleBufferFloat* edrBuffer,
	TwoWire* emotibitI2c, BufferFloat* edlOversampBuffer, BufferFloat* edrOversampBuffer)
{
	_emotibitVersion = version;
	_constants.samplingRate = samplingRate;

	// ToDo: Calculate digFiltAlpha

	_edaBuffer = edaBuffer;
	_edlBuffer = edlBuffer;
	_edrBuffer = edrBuffer;
	_edlOversampBuffer = edlOversampBuffer;
	_edrOversampBuffer = edrOversampBuffer;

	String output;
	output.reserve(30);

	output = "edaSeriesResistance: " + String(_constants.edaSeriesResistance);
	Serial.println(output);
	output = "samplingRate: " + String(_constants.samplingRate);
	Serial.println(output);
	output = "enableDigitalFilter: " + String(_constants.enableDigitalFilter);
	Serial.println(output);

	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{	
		output = "Configuring ADS ADC... ";
		Serial.println(output);

		_constants.adcBits = 16;

		// NOTE: if these values are changed in code, we should add parameters to _info.json
		_ads.setDataRate(RATE_ADS1115_475SPS);	// set to 475Hz to allow for 300Hz oversampling
		_ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV

		output = "adcBits: " + String(_constants.adcBits);
		Serial.println(output);
		output = "_ads.setDataRate: " + String("RATE_ADS1115_475SPS");
		Serial.println(output);
		output = "_ads.setGain: " + String("GAIN_TWO");
		Serial.println(output);
		output = "edaTransformSlope: " + String(_constants_v4_plus.edaTransformSlope);
		Serial.println(output);
		output = "edaTransformIntercept: " + String(_constants_v4_plus.edaTransformIntercept);
		Serial.println(output);

		return _ads.begin(0x48, emotibitI2c, false); // callBegin -> false. the i2c wire has already been init in setup

	}
	else if (_emotibitVersion <= EmotiBitVersionController::EmotiBitVersion::V03B)
	{
		output = "Configuring SAMD ADC... ";
		Serial.println(output);

		_constants.adcBits = 12;
		analogReadResolution(_constants.adcBits);
		_constants_v2_v3.adcRes = pow(2, _constants.adcBits) - 1;

		output = "adcBits: " + String(_constants.adcBits);
		Serial.println(output);
		output = "vcc: " + String(_constants_v2_v3.vcc);
		Serial.println(output);
		output = "edrAmplification: " + String(_constants_v2_v3.edrAmplification);
		Serial.println(output);
		output = "vRef1: " + String(_constants_v2_v3.vRef1);
		Serial.println(output);
		output = "vRef2: " + String(_constants_v2_v3.vRef2);
		Serial.println(output);
		output = "feedbackAmpR: " + String(_constants_v2_v3.feedbackAmpR);
		Serial.println(output);
		output = "crossoverFilterFreq: " + String(_constants_v2_v3.crossoverFilterFreq);
		Serial.println(output);
		output = "adcRes: " + String(_constants_v2_v3.adcRes);
		Serial.println(output);
		output = "edlPin: " + String(_constants_v2_v3.edlPin);
		Serial.println(output);
		output = "edrPin: " + String(_constants_v2_v3.edrPin);
		Serial.println(output);
		output = "isrOffsetCorr: " + String(_constants_v2_v3.isrOffsetCorr);
		Serial.println(output);

		return true;
	}

	return false;
}


bool EmotiBitEda::stageCalibStorage(EmotiBitNvmController * nvmController, String &edaCalibPacket, bool autoSync)
{
	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		uint8_t dataVersion;
		EmotiBitEdaCalibration::RawValues_V2 rawVals;
		if (EmotiBitEdaCalibration::unpackCalibPacket(edaCalibPacket, dataVersion, rawVals))
		{
			Serial.println("Writing calibration data:");
			if (dataVersion == EmotiBitEdaCalibration::V2)
			{
				//Serial.println(edaCalibPacket);
				EmotiBitEdaCalibration::print(rawVals);

				Serial.println("Staging to write...");
				uint8_t status = nvmController->stageToWrite(EmotiBitNvmController::DataType::EDA, dataVersion, sizeof(EmotiBitEdaCalibration::RawValues_V2), (uint8_t *)(&rawVals), autoSync);

				if (status != (uint8_t)EmotiBitNvmController::Status::SUCCESS)
				{
					Serial.println("nvmController->stageToWrite() failed: " + String(status));
					return false;
				}
			}
			else
			{
				// Write cases for other dataVersions
				Serial.println("stageCalibStorage() failed: version not supported");
				return false;
			}
		}
		else
		{
			Serial.println("unpackCalibPacket() failed");
			return false;
		}
	}
	else
	{
		Serial.println("Storing calibration data on V02/V03 HW requires firmware v1.2.86");  
		return false;
	}

}


bool EmotiBitEda::stageCalibLoad(EmotiBitNvmController * nvmController, bool autoSync)
{
	uint8_t dataVersion;
	uint32_t dataSize;
	uint8_t* data = nullptr;
	uint8_t status = nvmController->stageToRead(EmotiBitNvmController::DataType::EDA, dataVersion, dataSize, data, autoSync);

	if (status != (uint8_t)EmotiBitNvmController::Status::SUCCESS || dataSize == 0)
	{
		Serial.print("[status=");
		Serial.print(status);
		Serial.print("] ");
		return false;
	}
	if (dataVersion == EmotiBitEdaCalibration::V2 && dataSize == sizeof(EmotiBitEdaCalibration::RawValues_V2))
	{
		EmotiBitEdaCalibration::RawValues_V2 *rawVals = (EmotiBitEdaCalibration::RawValues_V2 *)data;
		EmotiBitEdaCalibration::print(*rawVals);
		EmotiBitEdaCalibration::calculate(*rawVals, _constants_v4_plus.edaTransformSlope, _constants_v4_plus.edaTransformIntercept);
		Serial.print("edaTransformSlope = ");
		Serial.println(_constants_v4_plus.edaTransformSlope);
		Serial.print("edaTransformIntercept = ");
		Serial.println(_constants_v4_plus.edaTransformIntercept);
	}
	else if (dataVersion == EmotiBitEdaCalibration::V0 && dataSize == sizeof(EmotiBitEdaCalibration::RawValues_V0))
	{
		EmotiBitEdaCalibration::RawValues_V0 *rawVals = (EmotiBitEdaCalibration::RawValues_V0 *)data;
		EmotiBitEdaCalibration::print(*rawVals);
		EmotiBitEdaCalibration::calculate(*rawVals, _constants_v2_v3.vRef1, _constants_v2_v3.vRef2, _constants_v2_v3.feedbackAmpR);
		Serial.print("vRef1 = ");
		Serial.println(_constants_v2_v3.vRef1);
		Serial.print("vRef2 = ");
		Serial.println(_constants_v2_v3.vRef2);
		Serial.print("feedbackAmpR = ");
		Serial.println(_constants_v2_v3.feedbackAmpR);
	}
	else
	{
		if (data != nullptr)
		{
			delete[] data;
		}
		Serial.print("[");
		Serial.print("dataSize=");
		Serial.print(dataSize);
		Serial.print(", dataVersion=");
		Serial.print(dataVersion);
		Serial.print("] ");
		return false;
	}

	if (data != nullptr)
	{
		delete[] data;
	}

	return true;
}

uint8_t EmotiBitEda::readData()
{
	int8_t status = 0;
	float edlTemp;	// Electrodermal Activity 
	float edrTemp;	// Electrodermal Activity 


	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		// Code to debug missed conversions
		//static uint16_t completed = 0;
		//static uint16_t total = 0;

		// Reads EDA data from ADS1113
		if (_ads.conversionComplete())
		{
			edlTemp = _ads.getLastConversionResults();
			_ads.startADC_Differential_0_1();
			// ToDo: consider how to utilize edl & edr buffers for different EmotiBit versions to minimize RAM footprint & code clarity
			status = status | _edlOversampBuffer->push_back(edlTemp);

			// Check for clipping
			// ToDo: assess correct levels more carefully
			const int16_t clipMinV4 = -32700;
			const int16_t clipMaxV4 = 32700;
			if (edlTemp < clipMinV4 && edlTemp > clipMaxV4)
			{
				status = status | _edlOversampBuffer->incrClippedCount();
			}

			// Code to debug missed conversions
			//completed++;
			//total++;
			//if (completed == 100)
			//{
			//	Serial.println(": " + String(completed) + "/" + String(total));
			//	completed = 0;
			//	total = 0;
			//}
		}
		else
		{
			_edlBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::IN);	// Count an overflow event if conversion was late
			
			// Code to debug missed conversions
			//total++;
			//Serial.print(String(total) + ",");
		}

		// Check if ready for downsampling
		if (_edlOversampBuffer->isFull()) 
		{
			// ToDo: Consider how to have version-specific changes in oversampling -- Using isFull/capacity of _edlOversampBuffer won't work
			// Note: data is saved in _edlBuffer to make factory test calibration easy
			// ToDo: Consider refactoring to use _edaBuffer
			status = status | _edlBuffer->downsample(_edlOversampBuffer);
			_edlOversampBuffer->clear();
		}
	}
	else
	{
		// Reads EDA data from SAMD21 ADC

		// Check EDL and EDR voltages for saturation
		edlTemp = analogRead(_constants_v2_v3.edlPin);
		edrTemp = analogRead(_constants_v2_v3.edrPin);

		status = status | _edlOversampBuffer->push_back(edlTemp);
		status = status | _edrOversampBuffer->push_back(edrTemp);

		// Check for clipping
		const int16_t clipMinV3 = 10;
		const int16_t clipMaxV3 = _constants_v2_v3.adcRes - 20;
		if (edlTemp < clipMinV3 && edlTemp > clipMaxV3)
		{
			status = status | _edlOversampBuffer->incrClippedCount();
		}
		if (edrTemp < clipMinV3 && edrTemp > clipMaxV3)
		{
			status = status | _edrOversampBuffer->incrClippedCount();
		}

		// Check if ready for downsampling
		if (_edlOversampBuffer->isFull())
		{
			status = status | _edlBuffer->downsample(_edlOversampBuffer);
			_edlOversampBuffer->clear();
		}
		if (_edrOversampBuffer->isFull())
		{
			status = status | _edrBuffer->downsample(_edrOversampBuffer);
			_edrOversampBuffer->clear();
		}
	}
	_readFinishedTime = micros();
	return status;
}


bool EmotiBitEda::processData()
{
	if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		size_t n;
		float * edlData;
		float edaTemp;
		uint32_t timestamp;
		n = _edlBuffer->getData(&edlData, &timestamp, true);
		for (size_t i = 0; i < n; i++)
		{
			// ToDo: Calculate slope/intercept to avoid expensive division in loop
			edaTemp = edlData[i] * _constants_v4_plus.edaTransformSlope + _constants_v4_plus.edaTransformIntercept;
			edaTemp = max(1000.f , edaTemp);
			edaTemp = 1000000.f / edaTemp;
			// ToDo: Consider filtering
			_edaBuffer->push_back(edaTemp, &timestamp);
		}
		// Transfer clipped counts
		_edaBuffer->incrClippedCount(DoubleBufferFloat::BufferSelector::IN, 
			_edlBuffer->getClippedCount(DoubleBufferFloat::BufferSelector::OUT)
		);

		// Transfer overflow counts
		_edaBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::IN,
			_edlBuffer->getOverflowCount(DoubleBufferFloat::BufferSelector::OUT)
		);

		// Swap EDA buffer
		_edaBuffer->swap();
	}
	else
	{
		size_t n, edlN, edrN;
		float *edlData, *edrData;
		uint32_t edlTs, edrTs;
		float edlTemp, edrTemp, edaTemp;

		// Wait for readData() to complete to avoid EDL/EDR size mismatch
		// NOTE: this can create a main loop delay up to the oversampling rate
		// This wouldn't be necessary with a ring buffer

		static const unsigned long int samplingInterval = 1000000 / (_constants.samplingRate * _edrOversampBuffer->capacity());

		//Serial.print("window: " + String(samplingInterval - (micros() - _readFinishedTime)));
		//Serial.println("");
		unsigned long int waitEnd;
		unsigned long int waitStart = micros();
		while (samplingInterval - (micros() - _readFinishedTime) < 500)
		{
			// Wait until we have at least 500 usec to do swap
			//Serial.println("WAIT");
			waitEnd = micros();
			if (waitEnd - waitStart > 100000)
			{
				Serial.println("Timeout waiting for EmotiBitEda::_readFinishedTime");
				break;
			}
		}
		
		// Swap EDL and EDR buffers with minimal delay to avoid size mismatch
		//unsigned long int swapStart = micros();
		_edlBuffer->swap();
		_edrBuffer->swap();
		//unsigned long int swapEnd = micros();
		//Serial.println("swap: " + String(swapEnd - swapStart));

		// Get pointers to the data buffers
		edlN = _edlBuffer->getData(&edlData, &edlTs, false);
		edrN = _edrBuffer->getData(&edrData, &edrTs, false);

		if (edlN != edrN)
		{
			Serial.print("WARNING: EDL (");
			Serial.print(edlN);
			Serial.print(") and EDR (");
			Serial.print(edrN);
			Serial.print(") buffers different sizes.");
			//Serial.print(" [");
			//Serial.print(waitEnd - waitStart);
			//Serial.print(", ");
			//Serial.print(swapEnd - swapStart);
			//Serial.print(" usecs]");
			Serial.println();
			// ToDo: Consider how to manage buffer size differences
			// One fix option is to switch to ring buffers instead of double buffers
			
			// Add overflow event(s) to account for the mismatched sizes
			size_t mismatch = abs(((int)edlN) - ((int)edrN));
			_edlBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::OUT, mismatch);
			_edrBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::OUT, mismatch);
		}

		// Loop through the data buffers and perform calculations
		n = min(edlN, edrN);
		for (uint8_t i = 0; i < n; i++)
		{
			edlTemp = edlData[i];
			edrTemp = edrData[i];

			// Correction for ADC value changes when ISR running
			edlTemp -= _constants_v2_v3.isrOffsetCorr;
			edrTemp -= _constants_v2_v3.isrOffsetCorr;

			// Perform data conversion
			// Convert ADC to Volts
			edlTemp = edlTemp * _constants_v2_v3.vcc / ((float) _constants_v2_v3.adcRes);
			edrTemp = edrTemp * _constants_v2_v3.vcc / ((float) _constants_v2_v3.adcRes);

			// In-place update buffers to Volts
			edlData[i] = edlTemp;
			edrData[i] = edrTemp;

			// EDL Digital Filter to remove noise
			if (_constants_v2_v3.crossoverFilterFreq > 0)// use only is a valid crossover freq is assigned
			{
				static DigitalFilter filterEda(DigitalFilter::FilterType::IIR_LOWPASS, _constants.samplingRate, _constants_v2_v3.crossoverFilterFreq);
				if (_constants.enableDigitalFilter)
				{
					edlTemp = filterEda.filter(edlTemp);
				}
			}

			// Link to diff amp biasing: https://ocw.mit.edu/courses/media-arts-and-sciences/mas-836-sensor-technologies-for-interactive-environments-spring-2011/readings/MITMAS_836S11_read02_bias.pdf
			edaTemp = (edrTemp - _constants_v2_v3.vRef2) / _constants_v2_v3.edrAmplification;	// Remove VGND bias and amplification from EDR measurement
			edaTemp = edaTemp + edlTemp;                     // Add EDR to EDL in Volts

			if (edaTemp - _constants_v2_v3.vRef1 < 0.000086f)
			{
				edaTemp = 0.001f; // Clamp the EDA measurement at 1K Ohm (0.001 Siemens)
			}
			else
			{
				edaTemp = _constants_v2_v3.vRef1 / 
					((_constants_v2_v3.feedbackAmpR * (edaTemp - _constants_v2_v3.vRef1))
						- (_constants.edaSeriesResistance * _constants_v2_v3.vRef1)
					);
			}

			edaTemp = edaTemp * 1000000.f; // Convert to uSiemens

			// Push calculated EDA value
			_edaBuffer->push_back(edaTemp, &edrTs);
		}
		// Transfer clipped counts
		_edaBuffer->incrClippedCount(DoubleBufferFloat::BufferSelector::IN,
			max(
				_edlBuffer->getClippedCount(DoubleBufferFloat::BufferSelector::OUT),
				_edrBuffer->getClippedCount(DoubleBufferFloat::BufferSelector::OUT)
			)
		);

		// Transfer overflow counts
		_edaBuffer->incrOverflowCount(DoubleBufferFloat::BufferSelector::IN,
			max(
				_edlBuffer->getOverflowCount(DoubleBufferFloat::BufferSelector::OUT),
				_edrBuffer->getOverflowCount(DoubleBufferFloat::BufferSelector::OUT)
			)
		);

		// Swap EDA buffer
		_edaBuffer->swap();
	}
	return true;
}

bool EmotiBitEda::writeInfoJson(File &jsonFile)
{
	const uint16_t bufferSize = 1024;
	{
		// Parse the root object
		StaticJsonBuffer<bufferSize> jsonBuffer;
		JsonObject &root = jsonBuffer.createObject();
		const uint8_t nInfo = 1;
		JsonObject* infos[nInfo];
		JsonArray* typeTags[nInfo];
		JsonObject* setups[nInfo];
		uint8_t i = 0;
		infos[i] = &(root.createNestedObject("info"));
		infos[i]->set("name", "ElectrodermalActivity");
		infos[i]->set("type", "ElectrodermalActivity");
		typeTags[i] = &(infos[i]->createNestedArray("typeTags"));
		typeTags[i]->add("EA");
		infos[i]->set("channel_count", 1);
		infos[i]->set("nominal_srate", _constants.samplingRate);
		infos[i]->set("channel_format", "float");
		infos[i]->set("units", "microsiemens");
		setups[i] = &(infos[i]->createNestedObject("setup"));
		setups[i]->set("eda_series_resistance", _constants.edaSeriesResistance);
		setups[i]->set("adc_bits", _constants.adcBits);
		setups[i]->set("enable_digital_filter", _constants.enableDigitalFilter);
		setups[i]->set("samples_averaged", _edlOversampBuffer->capacity());
		setups[i]->set("oversampling_rate", _edlOversampBuffer->capacity() * _constants.samplingRate);
		if (_emotibitVersion >= EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			setups[i]->set("eda_transform_slope", _constants_v4_plus.edaTransformSlope);
			setups[i]->set("eda_transform_intercept", _constants_v4_plus.edaTransformIntercept);
		} 
		else
		{
			setups[i]->set("voltage_reference_1", _constants_v2_v3.vRef1);
			setups[i]->set("voltage_reference_2", _constants_v2_v3.vRef2);
			setups[i]->set("EDA_feedback_amp_resistance", _constants_v2_v3.feedbackAmpR);
			setups[i]->set("EDR_amplification", _constants_v2_v3.edrAmplification);
			setups[i]->set("EDA_crossover_filter_frequency", _constants_v2_v3.crossoverFilterFreq);
			setups[i]->set("VCC", _constants_v2_v3.vcc);
			setups[i]->set("ISR_ADC_offset_correction", _constants_v2_v3.isrOffsetCorr);
		}

		if (root.printTo(jsonFile) == 0) {
#ifdef DEBUG
			Serial.println(F("Failed to write to file"));
#endif
		}
	}

}

void EmotiBitEda::setAdcIsrOffsetCorr(float isrOffsetCorr)
{
	_constants_v2_v3.isrOffsetCorr = isrOffsetCorr;
}
