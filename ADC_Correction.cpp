#include "ADC_Correction.h"

AdcCorrection::AdcCorrection(AdcCorrection::AdcCorrectionRigVersion version)
{
	if (version == AdcCorrection::AdcCorrectionRigVersion::VER_0)
	{
		adcCorrectionRig.N[0] = 1;  adcCorrectionRig.N[1] = 1;  adcCorrectionRig.N[2] = 10;
		adcCorrectionRig.D[0] = 11; adcCorrectionRig.D[1] = 2;  adcCorrectionRig.D[2] = 11;
		adcInputPins[0] = A0; adcInputPins[1] = A1; adcInputPins[2] = A2;
		setRigVersion(version);
	}
	else
	{
		//ToDo: Handle other version execption
	}
}

AdcCorrection::Status AdcCorrection::updateAtwincDataArray()
{
	if (dataFormatVersion == 0)
	{
		for (int i = 0; i < numAdcPoints; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if (j == 0)
				{
					atwincDataArray[4 * i + j] = adcCorrectionRig.N[i];
				}
				else if (j == 1)
				{
					atwincDataArray[4 * i + j] = adcCorrectionRig.D[i];
				}
				else if (j == 2)
				{
					atwincDataArray[4 * i + j] = adcCorrectionRig.AdcHigh[i];
				}
				else if (j == 3)
				{
					atwincDataArray[4 * i + j] = adcCorrectionRig.AdcLow[i];
				}
			}
		}
		updateAtwincMetadataArray();
		// toDo: ix this return
		return AdcCorrection::Status::SUCCESS;
	}
}

AdcCorrection::Status AdcCorrection::updateAtwincMetadataArray()
{
	rigMetadata[0] = numAdcPoints;
	rigMetadata[1] = dataFormatVersion;
	rigMetadata[2] = (int8_t)getRigVersion();
	//ToDo: Fix this return
	return AdcCorrection::Status::SUCCESS;
}

AdcCorrection::Status AdcCorrection::initWifiModule()
{
	uint8_t ret;
	if (!_isWifiDownloadMode)
	{
		
		WiFi.setPins(8, 7, 4, 2); // Need this for working with WiFi module on Adafruit feather
		nm_bsp_init();
		ret = m2m_wifi_download_mode();
		if (M2M_SUCCESS != ret)
		{
			Serial.print("Unable to enter download mode\r\n");
		}
		else
		{
			_atwincFlashSize = spi_flash_get_size();
		}
		Serial.println("Entered download mode successfully");
		Serial.print("The flash size is:"); Serial.println(u32FlashTotalSize);
		if (_atwincFlashSize == 4)// for 4 M flash
		{
			u32FlashTotalSize = FLASH_4M_TOTAL_SZ;
		}
	}
}

AdcCorrection::Status AdcCorrection::writeAtwincFlash()
{
	uint8_t  ret;
	// erasing primary sector
	Serial.println("\nErasing flash for write\n");
	ret = spi_flash_erase(ATWINC_MEM_LOC_PRIMARY_DATA, sizeof(atwincDataArray));
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;
	}

	// erasing the secondary sector
	ret = spi_flash_erase(ATWINC_MEM_LOC_DUPLICATE_DATA, sizeof(atwincDataArray));
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	// erase the metadata sector
	ret = spi_flash_erase(ATWINC_MEM_LOC_METADATA_LOC, 3); // ToDo: store this number "3" somewhere
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}

	// Writing the primary data
	ret = spi_flash_write(atwincDataArray, ATWINC_MEM_LOC_PRIMARY_DATA, sizeof(atwincDataArray));
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to write primary SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	// writing the secondary data
	Serial.println("\nWriting new data to the flash(primary)\n");
	ret = spi_flash_write(atwincDataArray, ATWINC_MEM_LOC_DUPLICATE_DATA, sizeof(atwincDataArray));
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to write secondary SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	// writing tthe metadata
	Serial.println("\nWriting new data to the flash(secondary)\n");
	ret = spi_flash_write(rigMetadata, ATWINC_MEM_LOC_METADATA_LOC, 3);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to write metadata SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	// ToDo: add a return success
}

AdcCorrection::Status AdcCorrection::readAtwincFlash(size_t readMemLoc, uint8_t* data, uint8_t isReadData = 1)
{
	uint8_t ret;
	size_t memoryReadSize;
	if (isReadData)
	{
		memoryReadSize = sizeof(atwincDataArray);
	}
	else 
	{
		memoryReadSize = sizeof(rigMetadata);
	}
	ret = spi_flash_read(data, readMemLoc, memoryReadSize);
	if (M2M_SUCCESS != ret)
	{
		printf("Unable to read SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	else
	{
		Serial.print("\nthe data stored in the last 2 bytes"); Serial.print("mem loc:"); Serial.println(readMemLoc);
		for (int i = 0; i < memoryReadSize; i++)
		{
			Serial.print(i); Serial.print(":"); Serial.print(data[i]); Serial.print("\t");
		}
	}

}

bool AdcCorrection::isSamdFlashWritten()
{

}

bool AdcCorrection::writeSamdFlash(uint16_t gainCorr, uint16_t offsetCorr)
{

}

void AdcCorrection::readSamdFlash(uint16_t &gainCorr, uint16_t &offsetCorr)
{

}
// ToDo: define ADC_CORRECTION_VERBOSE in the header file

bool AdcCorrection::calcCorrectionValues()
{
	if ((uint8_t)getRigVersion() == (uint8_t) AdcCorrection::AdcCorrectionRigVersion::VER_0)
	{
		float slope;
		uint16_t offsetCorr = 0, gainCorr = 0;
		uint16_t adcHighMeasured = int8Toint16(adcCorrectionRig.AdcHigh[2], adcCorrectionRig.AdcLow[2]);
		uint16_t adcLowMeasured = int8Toint16(adcCorrectionRig.AdcHigh[0], adcCorrectionRig.AdcLow[0]);
		uint16_t adcLowIdeal = (uint16_t)(((float)adcCorrectionRig.N[0]/ (float)adcCorrectionRig.D[0]) * 4096);
		uint16_t adcHighIdeal = (uint16_t)(((float)adcCorrectionRig.N[2] / (float)adcCorrectionRig.D[2]) * 4096);
#ifdef ADC_CORRECTION_VERBOSE
		Serial.print("ADC high Ideal:"); Serial.println(adcHighIdeal);
		Serial.print("ADC low Ideal:"); Serial.println(adcLowIdeal);
		Serial.print("ADC high measured:"); Serial.println(adcHighMeasured);
		Serial.print("ADC low measured:"); Serial.println(adcLowMeasured);
#endif
		slope = ((float)(adcHighMeasured- adcLowMeasured)/ (float)(adcHighIdeal-adcLowIdeal) );
#ifdef ADC_CORRECTION_VERBOSE
		Serial.print("\nslope in floating point is: "); Serial.println(slope, 6);
		Serial.print("gainCorr in float: "); Serial.println(2048 / slope, 6);
#endif
		offsetCorr = (int16_t)(adcLowMeasured - (slope * adcLowIdeal));
		if (offsetCorr < 0)
		{
			offsetCorr = 2048 + offsetCorr + 1;
		}
		gainCorr = round((2048 / slope));
		setGainCorrection(gainCorr);
		setOffsetCorrection(offsetCorr);
#ifdef ADC_CORRECTION_VERBOSE
		Serial.print("GainCorr:"); Serial.print(gainCorr); 
		Serial.print("\toffsetCorr:"); Serial.println(offsetCorr);
#endif
	}
}

uint16_t AdcCorrection::getGainCorrection()
{
	return _gainCorr;
}

uint16_t AdcCorrection::getOffsetCorrection()
{
	return _offsetCorr;
}

void AdcCorrection::setGainCorrection(uint16_t gainCorr)
{
	_gainCorr = gainCorr;
}

void AdcCorrection::setOffsetCorrection(uint16_t offsetCorr)
{
	_offsetCorr = offsetCorr;
}

void AdcCorrection::readAdcPins()
{
	uint16_t adcValue = 0;
	for (int i = 0; i < numAdcPoints; i++)
	{
		adcValue = getAverageAnalogInput(adcInputPins[i]);
		adcCorrectionRig.AdcHigh[i] = (adcValue & 0xFF00)>> 8;
		adcCorrectionRig.AdcLow[i] = adcValue & 0x00FF;
#ifdef ADC_CORRECTION_VERBOSE
		Serial.print("the ADC value read is:"); Serial.print(adcValue, HEX);
		Serial.print("\tADC high:"); Serial.print(adcCorrectionRig.AdcHigh[i], HEX);
		Serial.print("\tADC low:"); Serial.println(adcCorrectionRig.AdcLow[i], HEX);
#endif
	}

}

int AdcCorrection::getAverageAnalogInput(uint8_t inputPin)
{
	//ToDo: Add a LED indication to help tester with execution
	const int numReadings = 100;

	int readings[numReadings] = { 0 };      // the readings from the analog input
	int readIndex = 0;              // the index of the current reading
	int total = 0;                  // the running total
	int average = 0;                // the average

	uint32_t timeStart = millis();
	//digitalWrite(LED_BUILTIN, HIGH);
	while (true)
	{
		// subtract the last reading:
		total = total - readings[readIndex];
		// read from the sensor:
		//Serial.println(analogRead(inputPin));
		readings[readIndex] = analogRead(inputPin);
		// add the reading to the total:
		total = total + readings[readIndex];
		// advance to the next position in the array:
		readIndex = readIndex + 1;

		// if we're at the end of the array...
		if (readIndex >= numReadings) {
			// ...wrap around to the beginning:
			readIndex = 0;
		}

		// calculate the average:
		average = total / numReadings;
		// send it to the computer as ASCII digits
		//Serial.print("Time:"); Serial.print(millis()); Serial.print("\t");
		//Serial.println(average);
		if (millis() - timeStart >= 2000)
		{
			break;
		}
	}
	//digitalWrite(LED_BUILTIN, LOW);
	return average;

}
// ToDo: change the name of the function to begin?
void AdcCorrection::correctAdc()
{
	bool state;
	// write all the functoin calls here
	readAdcPins();
	Serial.println("finding correction values");
	state = calcCorrectionValues();
	Serial.println("calling update data array");
	updateAtwincDataArray();
	writeAtwincFlash();
}

AdcCorrection::AdcCorrectionRigVersion AdcCorrection::getRigVersion()
{
	return _version;
}

void AdcCorrection::setRigVersion(AdcCorrection::AdcCorrectionRigVersion version)
{
	_version = version;
}

uint16_t AdcCorrection::int8Toint16(uint8_t highByte, uint8_t lowByte)
{
	uint16_t result = 0;
	result = (uint16_t)(result | highByte);
	result = result << 8;
	result = (uint16_t)result | lowByte;
	return result;
}