#include "AdcCorrection.h"


AdcCorrection::AdcCorrection()
{
	Serial.println("Enter the ADC Correction Rig version being used.");
	Serial.println("Enter 0 for VER_0: R-Ladder Correction");
	Serial.println("Enter 1 for VER_1: AbsV correction");

	int rigVersionInput = -1, dataFormatVerInput = -1;
	while (rigVersionInput == -1)
	{
		rigVersionInput = serialToInt();
	}
	if ((int)AdcCorrection::DataFormatVersion::COUNT == 2) // if there exist only 1 format version(0 and Unknown)
	{
		dataFormatVerInput = (int)AdcCorrection::DataFormatVersion::DATA_FORMAT_0;
	}
	else
	{
		while (dataFormatVerInput != -1)
		{
			dataFormatVerInput = serialToInt();
		}
	}
	Serial.println("\n\n\n");
	Serial.print("RigVersion entered: "); Serial.println(rigVersionInput);
	Serial.print("dataFormatVersion entered: "); Serial.println(dataFormatVerInput);
	Serial.println("\n\n\n");

	if (rigVersionInput == (int)AdcCorrection::AdcCorrectionRigVersion::VER_0 || rigVersionInput == (int)AdcCorrection::AdcCorrectionRigVersion::VER_1)
	{
		setRigVersion((AdcCorrection::AdcCorrectionRigVersion)rigVersionInput);
		numAdcPoints = 3;
		if (dataFormatVerInput == (int)AdcCorrection::DataFormatVersion::DATA_FORMAT_0)
		{
			BYTES_PER_ADC_DATA_POINT = 4;
			ATWINC_DATA_ARRAY_SIZE = BYTES_PER_ADC_DATA_POINT * numAdcPoints;
		}
		else
		{
			// enter the format version details for other versions here
		}
		adcCorrectionRig.N[0] = 1;  adcCorrectionRig.N[1] = 1;  adcCorrectionRig.N[2] = 10;
		adcCorrectionRig.D[0] = 11; adcCorrectionRig.D[1] = 2;  adcCorrectionRig.D[2] = 11;
		adcInputPins[0] = A0;
		adcInputPins[1] = A1;
		adcInputPins[2] = A2;
		rigMetadata[0] = numAdcPoints;
		rigMetadata[1] = (uint8_t)dataFormatVerInput;
		rigMetadata[2] = (int8_t)getRigVersion();
		_isupdatedAtwincMetadataArray = true;
	}
	else
	{
		// handle other versions
		Serial.println("You are using an invalid version");
		Serial.println("stopping Execution");
		while (1);
	}
}

AdcCorrection::AdcCorrection(AdcCorrection::AdcCorrectionRigVersion version)
{
	if( version == AdcCorrection::AdcCorrectionRigVersion::UNKNOWN)
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

int AdcCorrection::serialToInt()
{
	// flush the serial buffer
	while (Serial.available())
	{
		Serial.read();
	}
	while (!Serial.available());
	String versionInString = "";
	int serialInput;
	while (Serial.available())
	{
		serialInput = Serial.read();
		if (isDigit(serialInput))
		{
			versionInString += (char)serialInput;
		}
		else
		{
			Serial.println("entered character not numeral.");
			Serial.println("Try again");
			versionInString = "";
			while (Serial.available())
				Serial.read(); // flushing serial
			return -1;
		}
	}
	return versionInString.toInt();
}

bool AdcCorrection::begin()
{
	Serial.println("################################");
	Serial.println("#####  ADC Correction Mode  ####");
	Serial.println("################################\n");
	Serial.println("IF YOU ARE A TESTER, enter A to continue.");
	Serial.println("Enter any other key to continue to normal bootup");
	while (!Serial.available());
	if (Serial.read() != 'A')
	{
		return false;
	}
	else
	{
		analogReadResolution(12); // Setting ADC resolution to 12 bits
#ifdef ADC_CORRECTION_VERBOSE
		Serial.println("Using the rig to generate correction values");
#endif
		Serial.println("ADC Correction Steps:");
		Serial.println("  * Make sure the battery is connected to the feather");
		Serial.println("  * Enter any character to initiate correction measurements");
		Serial.println("  * Then immediately remove the USB cable");
		Serial.println("  * Once the correction has been performed, the Red LED on the feather will start blinking with 2 pulses");
		Serial.println("  * At this point, reconnect the serial cable. A message will be displayed prompting you to enter any character to continue");
		Serial.println("Enter any key to begin...");
		while (!Serial.available()); Serial.read();
		Serial.println("** UNPLUG USB CABLE within 5 seconds to obtain accurate measurements **");
		for (int i = 5; i > 0; i--)
		{
			Serial.print(i); Serial.print(" "); delay(1000);
		}
		Serial.println("\nIF YOU SEE THIS MESSAGE, YOUR MEASUREMENTS MAY BE INACCURATE...");
		Serial.println("You should re-run ADC calibration...");
	}
	return true;
}

void AdcCorrection::echoResults(uint16_t gainCorr, uint16_t offsetCorr)
{
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("atwincDataArray");
	for (int i = 0; i < ATWINC_DATA_ARRAY_SIZE; i++)
	{
		Serial.print("  "); Serial.print(atwincDataArray[i]);
	}
#endif
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Enabling the ADC to use the correction values");
#endif
	uint16_t adcBeforeCorrection[3], adcAfterCorrection[3];
	adcBeforeCorrection[0] = analogRead(A0);
	adcBeforeCorrection[1] = analogRead(A1);
	adcBeforeCorrection[2] = analogRead(A2);
	analogReadCorrection(offsetCorr, gainCorr);
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Comparing results before and after correction");
#endif
	adcAfterCorrection[0] = analogRead(A0);
	adcAfterCorrection[1] = analogRead(A1);
	adcAfterCorrection[2] = analogRead(A2);

	digitalWrite(LED_BUILTIN, LOW);
	uint32_t timeCurrent = millis();
	while (!Serial.available())
	{
		if (millis() - timeCurrent > 2000)
		{
			Serial.println("Enter a character to continue to record results");
			timeCurrent = millis();
		}
		digitalWrite(LED_BUILTIN, HIGH);
		delay(200);
		digitalWrite(LED_BUILTIN, LOW);
		delay(200);
		digitalWrite(LED_BUILTIN, HIGH);
		delay(200);
		digitalWrite(LED_BUILTIN, LOW);
		delay(1000);
	}
	digitalWrite(LED_BUILTIN, HIGH);
	Serial.read();// pop from the buffer
	Serial.println("COPY AND PASTE the folowing into the feather records");
	Serial.println("==============================================");
	Serial.print(gainCorr); Serial.print(",");
	Serial.print(offsetCorr); Serial.print(",");
	Serial.print(adcBeforeCorrection[0]); Serial.print(",");
	Serial.print(adcBeforeCorrection[1]); Serial.print(",");
	Serial.print(adcBeforeCorrection[2]); Serial.print(",");
	Serial.print(adcAfterCorrection[0]); Serial.print(",");
	Serial.print(adcAfterCorrection[1]); Serial.print(",");
	Serial.print(adcAfterCorrection[2]);
	uint8_t tempData[12], tempdata2[3];
	readAtwincFlash(ATWINC_MEM_LOC_PRIMARY_DATA, 12, tempData);
	for (int i = 0; i < ATWINC_DATA_ARRAY_SIZE; i++)
	{
		Serial.print(","); Serial.print(tempData[i]);
	}
	Serial.print(",");
	WiFi.setPins(8, 7, 4, 2);
	WiFi.init();
	uint8_t atwincMacAddr[6], atwincMacValid;
	m2m_wifi_get_otp_mac_address(atwincMacAddr, &atwincMacValid);
	if (atwincMacValid)
	{
		for (int i = 0; i < 6; i++)
		{
			if (atwincMacAddr[i] <= 15)
			{
				Serial.print("0");
			}
			Serial.print(atwincMacAddr[i], HEX);

		}
	}
	else
	{
		Serial.println("INVALID_MAC");
	}
	Serial.print(",");
	printChipId();
	WiFi.end();
	Serial.println("\n==============================================");
	Serial.println("After you have copied the data, enter any character to continue");
	while (!Serial.available()); Serial.read();
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("\ntesting AT-WINC flash read");
	readAtwincFlash(ATWINC_MEM_LOC_PRIMARY_DATA, 12, tempData);
	readAtwincFlash(ATWINC_MEM_LOC_DUPLICATE_DATA, 12, tempData);
	readAtwincFlash(ATWINC_MEM_LOC_METADATA_LOC, 3, tempData, 0);
#endif

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
#ifdef ADC_CORRECTION_VERBOSE
			Serial.println("\nEntered download mode successfully");
			Serial.print("The flash size is:"); Serial.println(_atwincFlashSize);
			return AdcCorrection::Status::SUCCESS;
#endif
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
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Erasing flash for write\n");
#endif
	ret = spi_flash_erase(ATWINC_MEM_LOC_PRIMARY_DATA, BYTES_PER_ADC_DATA_POINT * numAdcPoints);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;
	}
	// Writing the primary data
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Writing new data to the flash(primary)");
#endif
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
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Writing new data to the flash(secondary)");
#endif
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
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Writing meta data");
#endif
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
			int16_t offsetCorr = 0, gainCorr = 0;
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
			uint16_t adcLowIdeal = (uint16_t)round((((float)adcCorrectionRig.N[0] / (float)adcCorrectionRig.D[0]) * 4096));
			uint16_t adcHighIdeal = (uint16_t)round((((float)adcCorrectionRig.N[2] / (float)adcCorrectionRig.D[2]) * 4096));
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
			// refer http://ww1.microchip.com/downloads/en/DeviceDoc/90003185A.pdf
			offsetCorr = (int16_t)round((adcLowMeasured - (slope * adcLowIdeal)));
			if (offsetCorr < 0)
			{
				offsetCorr = 4095 + offsetCorr + 1;
			}
			gainCorr = round((2048 / slope));
			setGainCorrection((uint16_t)gainCorr);
			setOffsetCorrection((uint16_t)offsetCorr);
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

// for below function, refer https://www.arduino.cc/en/Tutorial/BuiltInExamples/Smoothing
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
void AdcCorrection::execute(uint16_t &gainCorr, uint16_t &offsetCorr, bool &valid)
{
	bool state;
	AdcCorrection::Status status;
	// write all the functoin calls here
	readAdcPins();
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Consolidating all values into one atwinc data array");
#endif
	status = updateAtwincDataArray();
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("finding correction values");
#endif
	state = calcCorrectionValues();
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Writing the raw correction data to the flash");
#endif
	gainCorr = getGainCorrection();
	offsetCorr = getOffsetCorrection();
	valid = true;
	status = writeAtwincFlash();
	if (status == AdcCorrection::Status::SUCCESS)
	{
#ifdef ADC_CORRECTION_VERBOSE
		Serial.println("Data written on the ATWINC flash successfully.");
#endif
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

// refer SAMD21 datasheeet section 10.3.3 : http://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_DataSheet_DS40001882F.pdf
// refer https://gist.github.com/mgk/c9ec87436d2d679e5d08
void AdcCorrection::printChipId()
{
	volatile uint32_t val1, val2, val3, val4;
	volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
	val1 = *ptr1;
	volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
	val2 = *ptr;
	ptr++;
	val3 = *ptr;
	ptr++;
	val4 = *ptr;
	Serial.print("0x");
	char buf[33];
	sprintf(buf, "%8x%8x%8x%8x", val1, val2, val3, val4);
	Serial.println(buf);
}