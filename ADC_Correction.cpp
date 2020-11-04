#include "ADC_Correction.h"

AdcCorrection::AdcCorrection(AdcCorrection::AdcCorrectionRigVersion version, AdcCorrection::DataFormatVersion dataFormatVer)
{
	if (version == AdcCorrection::AdcCorrectionRigVersion::VER_0)
	{
		setRigVersion(version);
		dataFormatVersion = dataFormatVer;
		numAdcPoints = 3;
		if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0)
		{
			BYTES_PER_ADC_DATA_POINT = 4;
			ATWINC_DATA_ARRAY_SIZE = BYTES_PER_ADC_DATA_POINT * numAdcPoints;
		}
		adcCorrectionRig.N[0] = 1;  adcCorrectionRig.N[1] = 1;  adcCorrectionRig.N[2] = 10;
		adcCorrectionRig.D[0] = 11; adcCorrectionRig.D[1] = 2;  adcCorrectionRig.D[2] = 11;
		adcInputPins[0] = A0; 
		adcInputPins[1] = A1; 
		adcInputPins[2] = A2;
		rigMetadata[0] = numAdcPoints;
		rigMetadata[1] = (uint8_t)dataFormatVersion;
		rigMetadata[2] = (int8_t)getRigVersion();
		_isupdatedAtwincMetadataArray = true;
	}
	else if( version == AdcCorrection::AdcCorrectionRigVersion::UNKNOWN)
	{
		readAtwincFlash(ATWINC_MEM_LOC_METADATA_LOC, RIG_METADATA_SIZE, rigMetadata, 0);
		// Put a check here to see if the data in the metadata array is not corrupted
		if (rigMetadata[0] == 3)// the numAdcPoints is 3
		{
			atwincAdcMetaDataCorruptionTest = AdcCorrection::Status::SUCCESS;
			numAdcPoints = (uint8_t)rigMetadata[0];
			setRigVersion((AdcCorrection::AdcCorrectionRigVersion)((uint8_t)rigMetadata[1]));
			dataFormatVersion = (AdcCorrection::DataFormatVersion)((uint8_t)rigMetadata[2]); // typecasting uint8 to AdcCorrectionRigVersion
			_isupdatedAtwincMetadataArray = true;
			if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0)
			{
				BYTES_PER_ADC_DATA_POINT = 4;
				ATWINC_DATA_ARRAY_SIZE = BYTES_PER_ADC_DATA_POINT * numAdcPoints;
			}

			if (atwincFlashIntegrityCheck() == AdcCorrection::Status::SUCCESS)
			{
				readAtwincFlash(ATWINC_MEM_LOC_PRIMARY_DATA, ATWINC_DATA_ARRAY_SIZE, atwincDataArray);
				if (rigMetadata[1] == (uint8_t)AdcCorrection::AdcCorrectionRigVersion::VER_0)
				{
					// ToDo: Change this to read from the AT-WINC flash data
					adcCorrectionRig.N[0] = 1;  adcCorrectionRig.N[1] = 1;  adcCorrectionRig.N[2] = 10;
					adcCorrectionRig.D[0] = 11; adcCorrectionRig.D[1] = 2;  adcCorrectionRig.D[2] = 11;
					adcInputPins[0] = A0;
					adcInputPins[1] = A1;
					adcInputPins[2] = A2;
				}
				atwincAdcDataCorruptionTest = AdcCorrection::Status::SUCCESS;
			}
			else
			{
				atwincAdcDataCorruptionTest = AdcCorrection::Status::FAILURE;
				// failed data integrity. Something is wrong. Contact info@emotiBit
			}
		}
		else
		{
			atwincAdcMetaDataCorruptionTest = AdcCorrection::Status::FAILURE;
		}
	}
	else
	{
		// handle other versions
		Serial.println("You are using an invalid version");

	}

}

AdcCorrection::Status AdcCorrection::updateAtwincDataArray()
{
	if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0)
	{
		for (int i = 0; i < numAdcPoints; i++)
		{
			for (int j = 0; j < BYTES_PER_ADC_DATA_POINT; j++)
			{
				if (j == 0)
				{
					atwincDataArray[BYTES_PER_ADC_DATA_POINT * i + j] = adcCorrectionRig.N[i];
				}
				else if (j == 1)
				{
					atwincDataArray[BYTES_PER_ADC_DATA_POINT * i + j] = adcCorrectionRig.D[i];
				}
				else if (j == 2)
				{
					atwincDataArray[BYTES_PER_ADC_DATA_POINT * i + j] = adcCorrectionRig.AdcHigh[i];
				}
				else if (j == 3)
				{
					atwincDataArray[BYTES_PER_ADC_DATA_POINT * i + j] = adcCorrectionRig.AdcLow[i];
				}
			}
		}
		// updateAtwincMetadataArray();
		// toDo: Fix this return
		_isupdatedAtwincArray = true;
		return AdcCorrection::Status::SUCCESS;
	}
}

// moved this to the constructor
//AdcCorrection::Status AdcCorrection::updateAtwincMetadataArray()
//{
//	rigMetadata[0] = numAdcPoints;
//	rigMetadata[1] = dataFormatVersion;
//	rigMetadata[2] = (int8_t)getRigVersion();
//	_isupdatedAtwincArray = true;
//	//ToDo: Fix this return
//	return AdcCorrection::Status::SUCCESS;
//}

AdcCorrection::Status AdcCorrection::initWifiModule()
{
	uint8_t ret;
	if (!_isAtwincDownloadMode)
	{
		WiFi.setPins(8, 7, 4, 2); // Need this for working with WiFi module on Adafruit feather
		nm_bsp_init();
		ret = m2m_wifi_download_mode(); // required to access spi flash
		if (M2M_SUCCESS != ret)
		{
			Serial.print("Unable to enter download mode\r\n");
			return AdcCorrection::Status::FAILURE;
		}
		else
		{
			_isAtwincDownloadMode = true;
			_atwincFlashSize = spi_flash_get_size();
			Serial.println("\nEntered download mode successfully");
			Serial.print("The flash size is:"); Serial.println(_atwincFlashSize);
			return AdcCorrection::Status::SUCCESS;
		}

	}
}

AdcCorrection::Status AdcCorrection::writeAtwincFlash()
{
	if (!_isAtwincDownloadMode)
	{
		initWifiModule();
	}
	if (!_isupdatedAtwincArray && !_isupdatedAtwincMetadataArray)
	{
		Serial.println("Data array not ready to write");
		return AdcCorrection::Status::FAILURE;
	}
	uint8_t  ret;
	// erasing primary sector
	Serial.println("Erasing flash for write\n");
	ret = spi_flash_erase(ATWINC_MEM_LOC_PRIMARY_DATA, BYTES_PER_ADC_DATA_POINT * numAdcPoints);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;
	}
	// Writing the primary data

	Serial.println("Writing new data to the flash(primary)");
	ret = spi_flash_write(atwincDataArray, ATWINC_MEM_LOC_PRIMARY_DATA, BYTES_PER_ADC_DATA_POINT * numAdcPoints);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to write primary SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}


	// erasing the secondary sector
	ret = spi_flash_erase(ATWINC_MEM_LOC_DUPLICATE_DATA, BYTES_PER_ADC_DATA_POINT * numAdcPoints);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	// writing the secondary data
	Serial.println("Writing new data to the flash(secondary)");
	ret = spi_flash_write(atwincDataArray, ATWINC_MEM_LOC_DUPLICATE_DATA, BYTES_PER_ADC_DATA_POINT * numAdcPoints);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to write secondary SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}


	// erase the metadata sector
	ret = spi_flash_erase(ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE, RIG_METADATA_SIZE); 
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	// writing the metadata
	Serial.println("Writing meta data");
	ret = spi_flash_write(rigMetadata, ATWINC_MEM_LOC_METADATA_LOC, RIG_METADATA_SIZE);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to write metadata SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	// ToDo: add a return success
}

AdcCorrection::Status AdcCorrection::readAtwincFlash(size_t readMemLoc, uint16_t readSize, uint8_t* data, uint8_t readAdcData)
{
	if (!_isAtwincDownloadMode)
	{
		initWifiModule();
	}
	uint8_t ret;
	/*size_t memoryReadSize;
	if (readAdcData)
	{
		memoryReadSize = BYTES_PER_ADC_DATA_POINT * numAdcPoints;
	}
	else 
	{
		memoryReadSize = RIG_METADATA_SIZE;
	}*/
	ret = spi_flash_read(data, readMemLoc, readSize);
	if (M2M_SUCCESS != ret)
	{
		Serial.println("Unable to read SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	else
	{
#ifdef ADC_CORRECTION_VERBOSE
		Serial.print("\nthe data stored @"); Serial.print("mem loc:"); Serial.println(readMemLoc);
		for (int i = 0; i < readSize; i++)
		{
			Serial.print(i); Serial.print(":"); Serial.print(data[i]); Serial.print("\t");
		}
#endif
	}
}

bool AdcCorrection::isSamdFlashWritten()
{
	// SamdStorageAdcValues samdStorageAdcValues;
	// samdStorageAdcValues = samdFlashStorage.read();
	// return samdStorageAdcValues.valid;
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
	//if ((uint8_t)getRigVersion() == (uint8_t) AdcCorrection::AdcCorrectionRigVersion::VER_0)
	//{
		if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0)
		{
			uint16_t offsetCorr = 0, gainCorr = 0;
			float slope;
			uint8_t dataArrayNoffset = 0, dataArrayDoffset = 1, dataArrayAdcMsbOffset = 2, dataArrayAdcLsbOffset = 3;
			uint8_t AdcHighMem[2]; // Adc High value - [MSB] [LSB] 
			uint8_t AdcLowMem[2];  // Adc Low Value - [MSB] [LSB]
			uint8_t adcLowPos = 0, adcHighPos = 2; // Position of the ADC low and high points in the 
			AdcLowMem[0] = atwincDataArray[adcLowPos * BYTES_PER_ADC_DATA_POINT + dataArrayAdcMsbOffset]; // ADC Low point MSB
			AdcLowMem[1] = atwincDataArray[adcLowPos * BYTES_PER_ADC_DATA_POINT + dataArrayAdcLsbOffset];  // ADC Low point LSB
			AdcHighMem[0] = atwincDataArray[adcHighPos * BYTES_PER_ADC_DATA_POINT + dataArrayAdcMsbOffset];  // ADC High point MSB
			AdcHighMem[1] = atwincDataArray[adcHighPos * BYTES_PER_ADC_DATA_POINT + dataArrayAdcLsbOffset];  // ADC High point LSB
			uint16_t adcHighMeasured = int8Toint16(AdcHighMem[0], AdcHighMem[1]); // ( MSB, LSB)
			uint16_t adcLowMeasured = int8Toint16(AdcLowMem[0], AdcLowMem[1]); // (MSB, LSB)
			uint16_t adcLowIdeal = (uint16_t)(((float)adcCorrectionRig.N[0] / (float)adcCorrectionRig.D[0]) * 4096);
			uint16_t adcHighIdeal = (uint16_t)(((float)adcCorrectionRig.N[2] / (float)adcCorrectionRig.D[2]) * 4096);
#ifdef ADC_CORRECTION_VERBOSE
			Serial.print("\nADC high(Ideal):"); Serial.println(adcHighIdeal);
			Serial.print("ADC high(Measured):"); Serial.println(adcHighMeasured);
			Serial.print("ADC low(Ideal):"); Serial.println(adcLowIdeal);
			Serial.print("ADC low(Measured):"); Serial.println(adcLowMeasured);
#endif
			slope = ((float)(adcHighMeasured - adcLowMeasured) / (float)(adcHighIdeal - adcLowIdeal));
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
	//}
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
#ifdef ADC_CORRECTION_VERBOSE
		Serial.print("Reading pin:"); Serial.println(adcInputPins[i]);
#endif
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
void AdcCorrection::begin()
{
	bool state;
	AdcCorrection::Status status;
	// write all the functoin calls here
	readAdcPins();
	Serial.println("Consolidating all values into one atwinc data array");
	status = updateAtwincDataArray();
	Serial.println("finding correction values");
	state = calcCorrectionValues();
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Writing the raw correction data to the flash");
#endif
	status = writeAtwincFlash();
	if (status == AdcCorrection::Status::SUCCESS)
	{
		Serial.println("Data written on the ATWINC flash successfully.");
	}
	else
	{
		Serial.println("Failed to write to the ATWINC flash");
	}
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

AdcCorrection::Status AdcCorrection::atwincFlashIntegrityCheck()
{
	uint8_t primaryFromAtwinc[4*MAX_ADC_POINTS], duplicateFromAtwinc[4*MAX_ADC_POINTS];
	readAtwincFlash(ATWINC_MEM_LOC_PRIMARY_DATA, ATWINC_DATA_ARRAY_SIZE, primaryFromAtwinc);
	readAtwincFlash(ATWINC_MEM_LOC_DUPLICATE_DATA, ATWINC_DATA_ARRAY_SIZE, duplicateFromAtwinc);
	for (int i = 0; i < (ATWINC_DATA_ARRAY_SIZE); i++)
	{
		if (primaryFromAtwinc[i] != duplicateFromAtwinc[i])
		{
			return AdcCorrection::Status::FAILURE;
		}
	}
	return AdcCorrection::Status::SUCCESS;
}