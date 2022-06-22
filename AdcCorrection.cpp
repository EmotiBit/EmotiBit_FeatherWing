#ifdef ADAFRUIT_FEATHER_M0
#include "AdcCorrection.h"


AdcCorrection::AdcCorrection()
{
	Serial.println("################################");
	Serial.println("#####  ADC Correction Mode  ####");
	Serial.println("################################\n");
}

AdcCorrection::AdcCorrection(AdcCorrection::AdcCorrectionRigVersion version, uint16_t &gainCorr, uint16_t &offsetCorr, bool &valid, float &isrOffsetCorr)
{
	//Serial.println("No correction found on SAMD. Calculating correction by reading ATWINC");
	if( version == AdcCorrection::AdcCorrectionRigVersion::UNKNOWN)
	{
		if (isAtWincMetadataUpdated())
		{
			atwincAdcMetaDataCorruptionTest = AdcCorrection::Status::SUCCESS;
			numAdcPoints = (uint8_t)rigMetadata[METADATA_LOC_NUM_ADC];
			setRigVersion((AdcCorrection::AdcCorrectionRigVersion)((uint8_t)rigMetadata[METADATA_LOC_RIG_VERSION]));
			dataFormatVersion = (AdcCorrection::DataFormatVersion)((uint8_t)rigMetadata[METADATA_LOC_DATA_FORMAT]); // typecasting uint8 to AdcCorrectionRigVersion
			_isupdatedAtwincMetadataArray = true;
			if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0 || dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_1)
			{
				BYTES_PER_ADC_DATA_POINT = 4;
				if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0)
				{
					ATWINC_DATA_ARRAY_SIZE = BYTES_PER_ADC_DATA_POINT * numAdcPoints;
				}
				else if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_1)
				{
					ATWINC_DATA_ARRAY_SIZE = (BYTES_PER_ADC_DATA_POINT * numAdcPoints) + 4;
				}
			}

			if (atwincFlashIntegrityCheck() == AdcCorrection::Status::SUCCESS)
			{
				readAtwincFlash(ATWINC_MEM_LOC_PRIMARY_DATA, ATWINC_DATA_ARRAY_SIZE, atwincDataArray);
				// depending on the version, read the At-Winc to extract the required values
				parseAtwincDataArray();
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
			// flash has not been updated.
			atwincAdcMetaDataCorruptionTest = AdcCorrection::Status::FAILURE;
		}
		if (atwincAdcDataCorruptionTest == AdcCorrection::Status::FAILURE || atwincAdcMetaDataCorruptionTest == AdcCorrection::Status::FAILURE)
		{
			Serial.println("data on atwinc corrupted or not present");
			Serial.println("Using the ADC without any correction");
			return;
		}
		else// passed data corruption test. Data detected on the AT-Winc flash!
		{
			Serial.println("\nReading correction data from the AT-WINC flash");
			Serial.println("Calculating correction values");
			calcCorrectionValues();
			// Store the values on the flash
			Serial.println("Storing correction values on the SAMD flash");
			gainCorr = getGainCorrection();
			offsetCorr = getOffsetCorrection();
			valid = true;
			isrOffsetCorr = _isrOffsetCorr.inFloat;
		}
	}
	else
	{
		// handle other versions
		Serial.println("You are using an invalid version");
	}

}

void AdcCorrection::parseAtwincDataArray()
{
	if (rigMetadata[METADATA_LOC_RIG_VERSION] == (uint8_t)AdcCorrection::AdcCorrectionRigVersion::VER_0 || rigMetadata[METADATA_LOC_RIG_VERSION] == (uint8_t)AdcCorrection::AdcCorrectionRigVersion::VER_1)
	{
		// Retrieve calculation values depending on the data format version
		if (rigMetadata[METADATA_LOC_DATA_FORMAT] == (uint8_t)AdcCorrection::AdcCorrection::DataFormatVersion::DATA_FORMAT_0 || rigMetadata[METADATA_LOC_DATA_FORMAT] == (uint8_t)AdcCorrection::AdcCorrection::DataFormatVersion::DATA_FORMAT_1)
		{
			for (int i = 0; i < numAdcPoints; i++)
			{
				for (int j = 0; j < BYTES_PER_ADC_DATA_POINT; j++)
				{
					if (j == 0)
					{
						adcCorrectionRig.N[i] = atwincDataArray[BYTES_PER_ADC_DATA_POINT * i + j];
					}
					else if (j == 1)
					{
						adcCorrectionRig.D[i] = atwincDataArray[BYTES_PER_ADC_DATA_POINT * i + j];
					}
				}
			}
			if (rigMetadata[METADATA_LOC_DATA_FORMAT] == (uint8_t)AdcCorrection::AdcCorrection::DataFormatVersion::DATA_FORMAT_1)
			{
				// reading 4 bytes to retrieve the float
				for (int byte = 4; byte > 0; byte--)
				{
					_isrOffsetCorr.inBytes[4 - byte] = (int8_t)atwincDataArray[ATWINC_DATA_ARRAY_SIZE - byte];
				}
			}
			else
			{
				_isrOffsetCorr.inFloat = 0;
			}
		}
	}
}

bool AdcCorrection::begin(uint16_t &gainCorr, uint16_t &offsetCorr, bool &valid)
{
	bool state;
	AdcCorrection::Status status;
	analogReadResolution(12); // Setting ADC resolution to 12 bits
	Serial.println("Enter the ADC Correction Rig version being used.");
	Serial.println("Enter 0 for VER_0: R-Ladder Correction");
	Serial.println("Enter 1 for VER_1: AbsV correction");
	Serial.println("Any other number to exit");
	int rigVersionInput = -1, dataFormatVerInput = -1;
	while (rigVersionInput == -1)
	{
		rigVersionInput = serialToInt();
	}
	if (rigVersionInput != (int)AdcCorrection::AdcCorrectionRigVersion::VER_0 && rigVersionInput != (int)AdcCorrection::AdcCorrectionRigVersion::VER_1)
	{
		return false;
	}
	Serial.println("------------------"); Serial.print("RigVersion entered: "); Serial.println(rigVersionInput);
	// if there exist only 1 format version, use that
	if ((int)AdcCorrection::DataFormatVersion::COUNT <= 2) 
	{
		dataFormatVersion = AdcCorrection::DataFormatVersion::DATA_FORMAT_0;
		Serial.print("dataFormatVersion chosen: "); Serial.println((int)dataFormatVersion); Serial.println("------------------");
	}
	else
	{
		Serial.println("Enter the Data Format Version being used");
		while (dataFormatVerInput == -1)
		{
			dataFormatVerInput = serialToInt();
		}
		Serial.print("dataFormatVersion chosen: "); Serial.println(dataFormatVerInput); Serial.println("------------------");
		dataFormatVersion = (AdcCorrection::DataFormatVersion)dataFormatVerInput;
	}

	// Rig version sets rigVersion, numAdcPoints, Calculation Constants
	if (rigVersionInput == (int)AdcCorrection::AdcCorrectionRigVersion::VER_0 || rigVersionInput == (int)AdcCorrection::AdcCorrectionRigVersion::VER_1)
	{
		setRigVersion((AdcCorrection::AdcCorrectionRigVersion)rigVersionInput);
		numAdcPoints = 3;
		adcCorrectionRig.N[0] = 1;  adcCorrectionRig.N[1] = 1;  adcCorrectionRig.N[2] = 10;
		adcCorrectionRig.D[0] = 11; adcCorrectionRig.D[1] = 2;  adcCorrectionRig.D[2] = 11;
		adcInputPins[0] = A0;
		adcInputPins[1] = A1;
		adcInputPins[2] = A2;

		// Set bytes required per measurement depending on the dataformat chosen
		if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0)
		{
			BYTES_PER_ADC_DATA_POINT = 4;
			ATWINC_DATA_ARRAY_SIZE = BYTES_PER_ADC_DATA_POINT * numAdcPoints;
		}
		else
		{
			// enter the format version details for other versions here
		}

		// update metadata array values
		rigMetadata[METADATA_LOC_NUM_ADC] = numAdcPoints;
		rigMetadata[METADATA_LOC_RIG_VERSION] = (int8_t)getRigVersion();
		rigMetadata[METADATA_LOC_DATA_FORMAT] = (uint8_t)dataFormatVersion;
		
		_isupdatedAtwincMetadataArray = true;
	}
	else
	{
		// handle other versions
	}

	Serial.println("- Enter T to for TESTING MODE::  Values are calculated but not written to the flash. The Samd is updated.\n\t\t\t\tReprogram feather again to remove any changes made to the correction values.");
	Serial.println("- Enter P to for PROGRAMMING MODE(use for Shipping):: Values are calculated are written to the AT-Winc flash");
	Serial.println("- Enter any other key to continue to normal bootup");
	while (!Serial.available());
	char modeChoice = Serial.read();
	if (modeChoice == 'T' || modeChoice == 'P')
	{
		while (Serial.available())// flushing any extra characters
		{
			Serial.read();
		}
		uint32_t now = millis();
		// wait for 1 second
		while (!Serial.available() && millis() - now < 1000);
		if (Serial.available())
		{
			char input = Serial.read();
			// Special mode
			// if the user entered E, update with Existing correction values
			if (input == 'E')
			{
				if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0 || dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_1)
				{
					while (true)
					{
						Serial.println("-------------------------");
						Serial.println("Enter the existing correction data in the format shown below");
						Serial.println("N, D, ADC_MSB, ADC_LSB");
						Serial.println("Enter X to exit");
						while (!Serial.available());
						if (Serial.peek() == 'X')
						{
							Serial.println("Exiting adc update.");
							return false;
						}
						for (int i = 0; i < 12; i++)
						{
							String splitString = Serial.readStringUntil(',');
							atwincDataArray[i] = (uint8_t)splitString.toInt();
						}
						Serial.println("The data entered is ");
						for (int i = 0; i < 12; i++)
						{
							Serial.print(atwincDataArray[i]); Serial.print("  ");
						}
						Serial.println("Press Y to aprove, any other key to enter again");
						while (!Serial.available());
						if (Serial.read() != 'Y')
						{
							Serial.println("Enter the data again");
							for (int i = 0; i < 12; i++)
							{
								atwincDataArray[i] = 0;
							}
							while (Serial.available())
								Serial.read();
						}
						else
						{
							break;
						}
					}
				}
			}
			else
			{
				Serial.println("Invalid Option entered");
				while (Serial.available())
				{
					Serial.read();
				}
				return false;
			}
		}
		// Start ADC correction from scratch
		else
		{
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

			// Measure the voltages on all the ADC pins
			readAdcPins();
#ifdef ADC_CORRECTION_VERBOSE
			Serial.println("Consolidating all values into one atwinc data array");
#endif
			// update the AtWinc data array with measured values
			status = updateAtwincDataArray();
		}
#ifdef ADC_CORRECTION_VERBOSE
		Serial.println("finding correction values");
#endif
		calcCorrectionValues();
#ifdef ADC_CORRECTION_VERBOSE
		Serial.println("Writing the raw correction data to the flash");
#endif
		gainCorr = getGainCorrection();
		offsetCorr = getOffsetCorrection();
		valid = true;
		if (modeChoice == 'P') // Programmer Mode
		{
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
				return false;
			}
		}
		else
		{
			Serial.println("-------");
			Serial.println("TESTING MODE. ATwinc flash not updated.");
			Serial.println("-------");
		}
	}
	else
	{
		Serial.println("Invalid Option Chosen.");
		return false;
	}
	return true;
}


bool AdcCorrection::isAtWincMetadataUpdated()
{
	readAtwincFlash(ATWINC_MEM_LOC_METADATA, RIG_METADATA_SIZE, rigMetadata);
	// If the flash has not been updated, the value read = 255
	if (rigMetadata[METADATA_LOC_NUM_ADC] != 255 && rigMetadata[METADATA_LOC_RIG_VERSION] != 255 && rigMetadata[METADATA_LOC_DATA_FORMAT] != 255)
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool AdcCorrection::updateIsrOffsetCorr()
{
	uint16_t expectedValue;
#ifdef ADC_CORRECTION_VERBOSE 
	Serial.println("updateIsrOffsetCorr()");
#endif
	char input = 'N';
	String inputMeasurement;
	while (input != 'Y')
	{
		Serial.println("Enter the measured EDL value in ISR and the expected value if the format shown below:");
		Serial.println("EDL Measured Avg, Expected Value");
		while (!Serial.available());
		inputMeasurement = Serial.readStringUntil(',');
		_measuredAdcInIsr = inputMeasurement.toFloat();
		inputMeasurement = Serial.readStringUntil(',');
		expectedValue = inputMeasurement.toFloat();
		//_measuredAdcInIsr = serialToInt();
		if (_measuredAdcInIsr && expectedValue)
		{
			Serial.print("The measured value entered is: "); Serial.println(_measuredAdcInIsr,3);
			Serial.print("The expected value entered is: "); Serial.println(expectedValue);
			Serial.println("Do you wish to proceed? Press Y for yes and N to enter data again");
			while (!Serial.available());
			input = Serial.read();
			while (input != 'Y' && input != 'N')
			{
				Serial.println("Enter a valid option");
				while (!Serial.available());
				input = Serial.read();
			}
		}
		else
		{
			Serial.println("Invalid Entry! Enter a valid number");
			input = 'N';
		}
	}
	if (isAtWincMetadataUpdated())
	{
		atwincAdcMetaDataCorruptionTest = AdcCorrection::Status::SUCCESS;
		numAdcPoints = (uint8_t)rigMetadata[METADATA_LOC_NUM_ADC];
		setRigVersion((AdcCorrection::AdcCorrectionRigVersion)((uint8_t)rigMetadata[METADATA_LOC_RIG_VERSION]));
		dataFormatVersion = (AdcCorrection::DataFormatVersion)((uint8_t)rigMetadata[METADATA_LOC_DATA_FORMAT]); // typecasting uint8 to AdcCorrectionRigVersion
		if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0 || dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_1)
		{
			BYTES_PER_ADC_DATA_POINT = 4;
			ATWINC_DATA_ARRAY_SIZE = BYTES_PER_ADC_DATA_POINT * numAdcPoints;
			if (atwincFlashIntegrityCheck() == AdcCorrection::Status::SUCCESS)
			{
				atwincAdcDataCorruptionTest = AdcCorrection::Status::SUCCESS;
				// read before writing again
				readAtwincFlash(ATWINC_MEM_LOC_PRIMARY_DATA, ATWINC_DATA_ARRAY_SIZE, atwincDataArray);
				_isrOffsetCorr.inFloat = _measuredAdcInIsr - expectedValue;
				Serial.print("\nThe isrOffset correction calculated is: "); Serial.println(_isrOffsetCorr.inFloat,6);
				int prevSize = ATWINC_DATA_ARRAY_SIZE;
				ATWINC_DATA_ARRAY_SIZE += 4;// update dataArray size
				for (int byte = 0; byte < 4; byte++)
				{
					atwincDataArray[prevSize + byte] = _isrOffsetCorr.inBytes[byte];
				}
				_isupdatedAtwincArray = true;
				dataFormatVersion = AdcCorrection::DataFormatVersion::DATA_FORMAT_1;// updating the dataformat version to the new version
				rigMetadata[METADATA_LOC_DATA_FORMAT] = (uint8_t)dataFormatVersion;
				_isupdatedAtwincMetadataArray = true;
#ifdef ADC_CORRECTION_VERBOSE
				Serial.println("\nWriting to the Flash with the updated ISR correction");
#endif
				writeAtwincFlash();
				Serial.println("Completed updating the flash");
				return true;
			}
			else
			{
				atwincAdcDataCorruptionTest = AdcCorrection::Status::FAILURE;
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
		// Adc Correction has not been performed
#ifdef ADC_CORRECTION_VERBOSE
		Serial.println("Flash has not been updated with adc correction");
#endif
		return false;
	}
}

/*
Converts serial input char array to int
*/
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

AdcCorrection::Status AdcCorrection::updateAtwincDataArray()
{
	/*
	The data is stored in the format [Numerator, Denominator, MSB, LSB]
	*/
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
	return AdcCorrection::Status::FAILURE;
}


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
#endif
			return AdcCorrection::Status::SUCCESS;
		}

	}
	return AdcCorrection::Status::FAILURE;;
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
	ret = spi_flash_erase(ATWINC_MEM_LOC_PRIMARY_DATA, ATWINC_DATA_ARRAY_SIZE);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;
	}
	// Writing the primary data
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Writing new data to the flash(primary)");
#endif
	ret = spi_flash_write(atwincDataArray, ATWINC_MEM_LOC_PRIMARY_DATA, ATWINC_DATA_ARRAY_SIZE);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to write primary SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}


	// erasing the secondary sector
	ret = spi_flash_erase(ATWINC_MEM_LOC_DUPLICATE_DATA, ATWINC_DATA_ARRAY_SIZE);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to erase SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	// writing the secondary data
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Writing new data to the flash(secondary)");
#endif
	ret = spi_flash_write(atwincDataArray, ATWINC_MEM_LOC_DUPLICATE_DATA, ATWINC_DATA_ARRAY_SIZE);
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
	ret = spi_flash_write(rigMetadata, ATWINC_MEM_LOC_METADATA, RIG_METADATA_SIZE);
	if (M2M_SUCCESS != ret)
	{
		Serial.print("Unable to write metadata SPI sector\r\n");
		return AdcCorrection::Status::FAILURE;;
	}
	return AdcCorrection::Status::SUCCESS;
}

AdcCorrection::Status AdcCorrection::readAtwincFlash(size_t readMemLoc, uint16_t readSize, uint8_t* data)
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
		return AdcCorrection::Status::SUCCESS;
	}
}


void AdcCorrection::calcCorrectionValues()
{
	// determines which location in the data array to query for measured low and high values
	if ((uint8_t)getRigVersion() == (uint8_t) AdcCorrection::AdcCorrectionRigVersion::VER_0 || (uint8_t)getRigVersion() == (uint8_t)AdcCorrection::AdcCorrectionRigVersion::VER_1)
	{
		if (dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_0 || dataFormatVersion == AdcCorrection::DataFormatVersion::DATA_FORMAT_1)
		{
			int16_t offsetCorr = 0, gainCorr = 0;
			float rawOffsetCorr = 0.0, rawGainCorr = 0.0;
			float slope;
			uint8_t dataArrayNoffset = 0, dataArrayDoffset = 1, dataArrayAdcMsbOffset = 2, dataArrayAdcLsbOffset = 3;  // [N, D, MSB, LSB] 
			// ADC-High value measured and stored in AtWinc Data Array 
			uint8_t AdcHighMem[2]; // Adc High value - [MSB] [LSB] 
			// ADC-Low value measured and stored in At-Winc data array
			uint8_t AdcLowMem[2];  // Adc Low Value - [MSB] [LSB]
			// Position of the ADC low and high points in the AtWinc Data Array. For RigVer=0 or RigVer=1, Data stored = [ADC-Low, ADC-Mid, ADC-High]
			uint8_t adcLowPos = 0, adcHighPos = 2; // version dependent. ToDo: place it inside a if(version) conditional.
			AdcLowMem[0] = atwincDataArray[adcLowPos * BYTES_PER_ADC_DATA_POINT + dataArrayAdcMsbOffset]; // ADC Low point MSB
			AdcLowMem[1] = atwincDataArray[adcLowPos * BYTES_PER_ADC_DATA_POINT + dataArrayAdcLsbOffset];  // ADC Low point LSB
			AdcHighMem[0] = atwincDataArray[adcHighPos * BYTES_PER_ADC_DATA_POINT + dataArrayAdcMsbOffset];  // ADC High point MSB
			AdcHighMem[1] = atwincDataArray[adcHighPos * BYTES_PER_ADC_DATA_POINT + dataArrayAdcLsbOffset];  // ADC High point LSB
			uint16_t adcHighMeasured = int8Toint16(AdcHighMem[0], AdcHighMem[1]); // ( MSB, LSB)
			uint16_t adcLowMeasured = int8Toint16(AdcLowMem[0], AdcLowMem[1]); // (MSB, LSB)
			float adcLowIdeal = (((float)adcCorrectionRig.N[0] / (float)adcCorrectionRig.D[0]) * 4095.f);  // ToDo: make the ADC resolution a constant.
			float adcHighIdeal = (((float)adcCorrectionRig.N[2] / (float)adcCorrectionRig.D[2]) * 4095.f);  // ToDo: make the ADC resolution a constant.
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
			rawOffsetCorr = (float)adcLowMeasured - (slope * adcLowIdeal);
#ifdef ADC_CORRECTION_VERBOSE
			Serial.print("offset correction before Rounding: ");
			Serial.println(rawOffsetCorr);
#endif
			offsetCorr = round(rawOffsetCorr);
#ifdef ADC_CORRECTION_VERBOSE
			Serial.print("offsetCorrection after rounding: ");
			Serial.println(offsetCorr);
#endif
			if (offsetCorr < 0)
			{
				offsetCorr = 4095 + offsetCorr + 1; // ToDo: make the ADC resolution a constant
			}
			rawGainCorr = (2048.f / slope);  // ToDo: make the ADC resolution a constant
#ifdef ADC_CORRECTION_VERBOSE
			Serial.print("gain correction before Rounding: ");
			Serial.println(rawGainCorr);
#endif
			gainCorr = round(rawGainCorr);
#ifdef ADC_CORRECTION_VERBOSE
			Serial.print("gain Correction after rounding: ");
			Serial.println(gainCorr);
#endif
			setGainCorrection((uint16_t)gainCorr);
			setOffsetCorrection((uint16_t)offsetCorr);
#ifdef ADC_CORRECTION_VERBOSE
			Serial.print("GainCorr:"); Serial.print(gainCorr);
			Serial.print("\toffsetCorr:"); Serial.println(offsetCorr);
#endif
		}
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
#ifdef ADC_CORRECTION_VERBOSE
		Serial.print("Reading pin:"); Serial.println(adcInputPins[i]);
#endif
		adcValue = getAverageAnalogInput(adcInputPins[i]);
		adcCorrectionRig.AdcHigh[i] = (adcValue & 0xFF00)>> 8;
		adcCorrectionRig.AdcLow[i] = adcValue & 0x00FF;
#ifdef ADC_CORRECTION_VERBOSE
		Serial.print("\nthe ADC value read is(DEC):"); Serial.println(adcValue);
		Serial.print("the ADC value read is(HEX):"); Serial.print(adcValue, HEX);
		Serial.print("\tADC high:"); Serial.print(adcCorrectionRig.AdcHigh[i], HEX);
		Serial.print("\tADC low:"); Serial.println(adcCorrectionRig.AdcLow[i], HEX);
#endif
	}

}

int AdcCorrection::getAverageAnalogInput(uint8_t inputPin)
{
	uint32_t sum = 0, counter = 0;
	int average = 0;
	uint32_t timeStart = millis();
	while (millis() - timeStart < 2000)
	{
		sum += analogRead(inputPin);
		counter++;
		delay(1);
	}
	average = sum / counter;
#ifdef ADC_CORRECTION_VERBOSE
	Serial.print("Samples averaged: "); Serial.println(counter);
	Serial.print("Average value: "); Serial.println(average);
#endif
	return ((float)sum / (float)counter);
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
	Serial.println("\nComparing results before and after correction");
#endif
	uint16_t adcBeforeCorrection[3], adcAfterCorrection[3];
	adcBeforeCorrection[0] = getAverageAnalogInput(A0);
	adcBeforeCorrection[1] = getAverageAnalogInput(A1);
	adcBeforeCorrection[2] = getAverageAnalogInput(A2);
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("\nEnabling the ADC to use the correction values");
#endif
	analogReadCorrection(offsetCorr, gainCorr);
	adcAfterCorrection[0] = getAverageAnalogInput(A0);
	adcAfterCorrection[1] = getAverageAnalogInput(A1);
	adcAfterCorrection[2] = getAverageAnalogInput(A2);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	uint32_t timeCurrent = millis();
	while (!Serial.available())
	{
		if (millis() - timeCurrent > 2000)
		{
			Serial.println("\nEnter a character to continue to record results");
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
	pinPeripheral(LED_BUILTIN, PIO_SERCOM);
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
	uint8_t tempData[ATWINC_DATA_ARRAY_SIZE];
	readAtwincFlash(ATWINC_MEM_LOC_PRIMARY_DATA, ATWINC_DATA_ARRAY_SIZE, tempData);
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

	Serial.println("\n==============================================");
	Serial.println("After you have copied the data, enter any character to continue");
	while (!Serial.available()); Serial.read();
/*
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("\ntesting AT-WINC flash read");
	readAtwincFlash(ATWINC_MEM_LOC_PRIMARY_DATA, 12, tempData);
	readAtwincFlash(ATWINC_MEM_LOC_DUPLICATE_DATA, 12, tempData);
	readAtwincFlash(ATWINC_MEM_LOC_METADATA, 3, tempData2);
#endif
*/
	WiFi.end();

}
#endif