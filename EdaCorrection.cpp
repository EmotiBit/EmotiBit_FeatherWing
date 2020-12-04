#include "EdaCorrection.h"


void EdaCorrection::begin(uint8_t emotiBitVersion)
{
	Serial.println("################################");
	Serial.println("#####  EDA Correction Mode  ####");
	Serial.println("################################\n");
	Serial.println("If you are an EmotiBit Tester, enter A to continue.");
	Serial.println("Otherwise enter any key to exit EDA Correction Mode");
	while (!Serial.available());
	char choice = Serial.read();
	if (choice == 'A')
	{
		EdaCorrection::Status status;
		status = enterUpdateMode(emotiBitVersion, EdaCorrection::OtpDataFormat::DATA_FORMAT_0);
	}
	
}

EdaCorrection::Status EdaCorrection::enterUpdateMode(uint8_t emotiBitVersion, EdaCorrection::OtpDataFormat dataFormat)
{
	
	Serial.println("Enabling Mode::UPDATE");
	_mode = EdaCorrection::Mode::UPDATE;
	_emotiBitVersion = emotiBitVersion;
	otpDataFormat = dataFormat;
	Serial.print("\nEnter X to perform Actual OTP R/W\n");
	Serial.println("Enter any other key to default into DUMMY MODE");
	while (!Serial.available());
	char choice = Serial.read();
	if(choice == 'X')
	{
		Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		Serial.println("!!!! Actually writing to the OTP  !!!!");
		Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	}
	else
	{
		dummyWrite = true;
		Serial.println("###################");
		Serial.println("## IN DUMMY MODE ##");
		Serial.println("###################");
	}
	Serial.println("Initializing state machine. State: WAITING_FOR_SERIAL_DATA");
	progress = EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA;
	Serial.println("The tester should now the use the EmotiBit with the High precision EDA test rig");
	Serial.println("You should use the EmotiBit Oscilloscope in TESTING MODE, connect to the EmotiBit attached to the EDA test rig. Record EDL and EDR values using the Oscilloscope.");
	Serial.println("Once you have the appropriate data, please plug in the serial cable and enter the data.");
	Serial.println("Copy and paste the 5 EDL values and 5 EDR values in the format shown below");
	Serial.println("EDL_0R, EDL_10K, EDL_100K, EDL_1M, EDL_10M, vRef2_0R, vRef2_10K, vRef2_100K, vRef2_1M, vRef2_10M");
	Serial.println("Enter any character to proceed with execution");
	while (!Serial.available()); Serial.read();
	Serial.print("Proceeding with normal execution in\n");
	uint8_t msgTimer = 3;
	while (msgTimer > 0)
	{
		Serial.print(msgTimer); Serial.print("\t");
		delay(1000);
		msgTimer--;
	}
	return EdaCorrection::Status::SUCCESS;

}

void EdaCorrection::normalModeOperations(float &vref1, float &vref2, float &Rfeedback)
{
	// reading the OTP is done in the ISR
	// the correction values should be generated in the update function outside the ISR,
	// to reduce the load on the ISR
	// call calcCorrection
	if (triedRegOverwrite)
	{
		Serial.println("Register OverWrite Protection Triggered");
		Serial.println("Please check the OTP of the Si7013 independently.");
		if (successfulwrite)
		{
			Serial.println("New memory location(s) has been correctly updated.");
			Serial.println("Completely power down the EmotiBit and start EmotiBit again");
		}
		otpMemoryMap.echoWriteCount();
		progress = EdaCorrection::Progress::FINISH;// prevents re-enterin normal mode operations
	}
	else
	{
		if (!powerCycled)
		{
			Serial.println("Done writing to the OTP. Completely power down the EmotiBit by unpluging the Serial cable and removing the battery.");
			Serial.println("After the initial power down, can start using the EmotiBit normally.");
			otpMemoryMap.echoWriteCount();
			progress = EdaCorrection::Progress::FINISH;// stops entering this function again from emotibit.update
		}
		else
		{
			if (readOtpValues)
			{
				if (isSensorConnected)
				{
					if (isOtpValid)// metadata presence is checked in the OTP read operation
					{
						Serial.print("The EmotiBit Version stored for EDA correction is: "); Serial.println((int)_emotiBitVersion);
						Serial.print("The data format version stored for EDA correction is: "); Serial.println((int)otpDataFormat);
						calcEdaCorrection();
						if (correctionDataReady)// do not echo on screen if writing to test memory space
						{
							Serial.print("Estimated Rskin values BEFORE correction:|");
							float RskinEst = 0;
							for (int i = 0; i < NUM_EDL_READINGS; i++)
							{
								RskinEst = (((correctionData.edlReadings[i] / vref1) - 1)*Rfeedback);
								Serial.print(RskinEst); Serial.print(" | ");
							}
							vref1 = correctionData.vRef1;// updated vref1
							vref2 = correctionData.vRef2;// updated vref2
							Rfeedback = correctionData.Rfb;// updated edaFeedbackAmpR
							Serial.print("\nEstimated Rskin values AFTER correction: |");
							for (int i = 0; i < NUM_EDL_READINGS; i++)
							{
								RskinEst = (((correctionData.edlReadings[i] / vref1) - 1)*Rfeedback);
								Serial.print(RskinEst); Serial.print(" | ");
							}
							if (dummyWrite)
							{
								Serial.println("\nupdated emotibit class with these values");
								Serial.println("\nYou can now use this EmotiBit without restarting to measure the EDA test rig values");
							}
							//edaCorrection.correctionDataReady = false; // once the values are updated, we can set it to false to not enter this case again
							otpMemoryMap.echoWriteCount();
							progress = EdaCorrection::Progress::FINISH;
						}
					}
					else
					{
						Serial.println("OTP has not been updated. Not performing correction calculation.");
						Serial.println("Perform EDA correction first.");
						Serial.println("Using EDA without correction");
						otpMemoryMap.echoWriteCount();
						progress = EdaCorrection::Progress::FINISH;

					}
				}	
				else
				{
#ifdef USE_ALT_SI7013
					Serial.println("EDA Correction ERROR: External SI-7013 sensor not detected on I2C line. Please check connection");
#else
					Serial.println("EDA Correction ERROR: Main EmotiBit SI-7013 sensor not detected on I2C line. Please check connection");
#endif
					otpMemoryMap.echoWriteCount();
					progress = EdaCorrection::Progress::FINISH;
				}
			}
		}
	}
}

EdaCorrection::Mode EdaCorrection::getMode()
{
	return _mode;
}

EdaCorrection::Status EdaCorrection::getFloatFromString()
{
	float input[2 * NUM_EDL_READINGS];
	for (int i = 0; i < 2 * NUM_EDL_READINGS; i++)
	{
		if (Serial.available())
		{
			String splitString = Serial.readStringUntil(',');
			input[i] = splitString.toFloat();
		}
		else // not enough data entered
		{
			Serial.println("Fewer than expected data points");
			return EdaCorrection::Status::FAILURE;
		}
	}
	if (Serial.available()) // more data that should be parsed present.
	{
		Serial.println("More than expected data points");
		while (Serial.available())
			Serial.read();
		return EdaCorrection::Status::FAILURE;
	}

	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		correctionData.edlReadings[i] = input[i];
	}

	for (int i = NUM_EDL_READINGS; i < 2 * NUM_EDL_READINGS; i++)
	{
		correctionData.edrReadings[i - NUM_EDL_READINGS] = input[i];
	}
	
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		correctionData.vRef2 += correctionData.edrReadings[i];
	}
	correctionData.vRef2 = correctionData.vRef2 / NUM_EDL_READINGS;
	
	return EdaCorrection::Status::SUCCESS;
}

EdaCorrection::Status EdaCorrection::monitorSerial()
{

	if (progress == EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA)
	{
		if (Serial.available())
		{
			Serial.println("Serial data detected.");
			Serial.println("####################");
			Serial.println("### EDA TESTING  ###");
			Serial.println("####################");
			if (dummyWrite)
			{
			Serial.println("#### DUMMY MODE ####");
			}
			else 
			{
				Serial.println("!!! OTP MODE !!!");
#ifdef USE_ALT_SI7013
				Serial.println("!!! Writing to Alternate(External) I2C sensor !!!");
#else
				Serial.println("!!! Writing to main EmotiBit I2C sensor !!!");
#endif
#ifdef ACCESS_MAIN_ADDRESS
				Serial.println("!!! Writing to Main Address space !!!");
#else
				Serial.println("!!! Writing to Test Address Space(0xA0 - 0xA7) !!!\n");
#endif
			}
			if (getFloatFromString() != EdaCorrection::Status::SUCCESS)
			{
				Serial.println("Please check and enter again");
				Serial.println("Expected Format");
				Serial.println("EDL_0R, EDL_10K, EDL_100K, EDL_1M, EDL_10M, vRef2_0R, vRef2_10K, vRef2_100K, vRef2_1M, vRef2_10M");
			}
			else
			{
				echoEdaReadingsOnScreen();
				progress = EdaCorrection::Progress::WAITING_USER_APPROVAL;
			}
		}
	}
	else if (progress == EdaCorrection::Progress::WAITING_USER_APPROVAL)
	{
		if (_responseRecorded == false)
		{
			getUserApproval();
		}
		else
		{
			if (getApprovalStatus() == true)
			{
				Serial.println("#### GOT APPROVAL ####");
				if (!dummyWrite)
				{
					Serial.println(".....");
					Serial.println("Writing to the OTP");
					Serial.println("...");
				}
				progress = EdaCorrection::Progress::WRITING_TO_OTP;
			}
			else
			{

				correctionData.vRef1 = 0; correctionData.vRef2 = 0; correctionData.Rfb = 0;
				for (int i = 0; i < NUM_EDL_READINGS; i++)
				{
					correctionData.edlReadings[i] = 0;
					correctionData.edrReadings[i] = 0;
				}
				
				// ask to enter the second time 
				Serial.println("back to Progress::WAITING_FOR_SERIAL_DATA");
				Serial.println("Enter the eda values into the serial monitor");
				progress = EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA;
				_responseRecorded = false;
			}
		}
	}

}


void EdaCorrection::echoEdaReadingsOnScreen()
{
	Serial.println("The EDA values entered by the user are:");

	Serial.println("EDL readings:");
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		Serial.print(correctionData.edlReadings[i], FLOAT_PRECISION); Serial.print("\t");
	}
	Serial.println("\nVref2 readings:");
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		Serial.print(correctionData.edrReadings[i], FLOAT_PRECISION); Serial.print("\t");
	}
	Serial.print("\nAvg Vref2 :"); Serial.println(correctionData.vRef2, FLOAT_PRECISION);
	//_EdaReadingsPrinted = true;

	Serial.println("\nIf you are in Dummy mode, NO OTP R/W ACTIONS WILL BE PERFORMED");
	Serial.println("This is how the OTP is will be updated(If NOT IN DUMMY MODE)");
	Serial.println("You can change R/W locations in OTP by toggling ACCESS_MAIN_ADDRESS\n");

	for (int iterEdl = 0; iterEdl < NUM_EDL_READINGS; iterEdl++)
	{
		otpData.inFloat = correctionData.edlReadings[iterEdl];
		Serial.print("EdlVal:"); Serial.println(correctionData.edlReadings[iterEdl], FLOAT_PRECISION);
#ifdef ACCESS_MAIN_ADDRESS
		uint8_t baseAddr = otpMemoryMap.edlAddresses[iterEdl];
#else
		uint8_t baseAddr = otpMemoryMap.edlTestAddress;
#endif
		for (int byte = 0; byte < BYTES_PER_FLOAT; byte++)
		{
			Serial.print("0x"); Serial.print(baseAddr + byte, HEX); Serial.print(" : ");
			Serial.println(otpData.inByte[byte], DEC);
		}
#ifndef ACCESS_MAIN_ADDRESS
		break;
#endif
	}

#ifdef ACCESS_MAIN_ADDRESS
	//vref2
	otpData.inFloat = correctionData.vRef2;
	Serial.println();
	Serial.print("vRef2: ");
	Serial.println(correctionData.vRef2, FLOAT_PRECISION);
	for (int byte = 0; byte < BYTES_PER_FLOAT; byte++)
	{
		Serial.print("0x"); Serial.print(otpMemoryMap.edrAddresses + byte, HEX); Serial.print(" : ");
		Serial.println(otpData.inByte[byte], DEC);
	}
	// metadata
	Serial.println("\nmetadata");
	Serial.print("0x"); Serial.print(otpMemoryMap.dataVersionAddr, HEX); Serial.print(" : ");
	Serial.println((uint8_t)otpDataFormat, DEC);
	Serial.print("0x"); Serial.print(otpMemoryMap.emotiBitVersionAddr, HEX); Serial.print(" : ");
	Serial.println(_emotiBitVersion, DEC);
#endif

	Serial.println("\nProceed with these values?");
	Serial.println("Enter Y for yes and N to enter data again");

}


bool EdaCorrection::getUserApproval()
{
	if (Serial.available())
	{
		char response = Serial.read();
		if (response == 'Y')
		{
			setApprovalStatus(true);
			_responseRecorded = true;
		}
		else if (response == 'N')
		{
			setApprovalStatus(false);
			_responseRecorded = true;
		}
		else
		{
			Serial.println("incorrect choice. Please enter either Y for Yes or N for No");
		}

	}
}

void EdaCorrection::setApprovalStatus(bool response)
{
	_approvedToWriteOtp = response;
}


bool EdaCorrection::getApprovalStatus()
{
	return _approvedToWriteOtp;
}

bool EdaCorrection::checkSensorConnection(Si7013* si7013)
{
	return (si7013->sendCommand(0x00)); // returns false if failed to send command
}

bool EdaCorrection::isOtpRegWritten(Si7013* si7013, uint8_t addr, bool isOtpOperation)
{
	if (si7013->readRegister8(addr, isOtpOperation) == 255)
	{
		return false;
	}
	else
	{
		return true;
	}
}

EdaCorrection::Status EdaCorrection::writeToOtp(Si7013* si7013, uint8_t addr, char val, uint8_t mask)
{
	if (isOtpRegWritten(si7013, addr))
	{
		//Serial.print(addr, HEX); Serial.println(":N");
		return EdaCorrection::Status::FAILURE;
	}
	// For testing
	//Serial.print(addr, HEX); Serial.print(":Y:"); Serial.println(val);
	// for real write
	si7013->writeToOtp(addr, val, mask);
	successfulwrite = true;
	return EdaCorrection::Status::SUCCESS;
}

EdaCorrection::Status EdaCorrection::writeToOtp(Si7013* si7013)
{
	if (dummyWrite)
	{
		_mode = EdaCorrection::Mode::NORMAL;
	}
	else
	{

		if (checkSensorConnection(si7013) == true)// sensor is present
		{
			if (progress == EdaCorrection::Progress::WRITING_TO_OTP && triedRegOverwrite == false)
			{

#ifdef ACCESS_MAIN_ADDRESS
				// writing the metadata
				if (otpMemoryMap.dataVersionWritten == false)
				{
					otpMemoryMap.writeCount.dataVersion++;
					if ((uint8_t)writeToOtp(si7013, otpMemoryMap.dataVersionAddr, (uint8_t)otpDataFormat) == (uint8_t)EdaCorrection::Status::SUCCESS)
					{
						otpMemoryMap.dataVersionWritten = true;
					}
					else
					{
						triedRegOverwrite = true;
					}
				}
				else
				{
					otpMemoryMap.writeCount.dataVersion++;
				}

				if (otpMemoryMap.emotiBitVersionWritten == false)
				{
					otpMemoryMap.writeCount.emotiBitVersion++;
					if ((uint8_t)writeToOtp(si7013, otpMemoryMap.emotiBitVersionAddr, _emotiBitVersion) == (uint8_t)EdaCorrection::Status::SUCCESS)
					{
						otpMemoryMap.emotiBitVersionWritten = true;
					}
					else
					{
						triedRegOverwrite = true;
					}
				}
				else
				{
					otpMemoryMap.writeCount.emotiBitVersion++;
				}

				// writing the vref 2 to OTP
				otpData.inFloat = correctionData.vRef2;
				for (uint8_t j = 0; j < 4; j++)
				{
					if (otpMemoryMap.edrDataWritten[j] == false)
					{
						otpMemoryMap.writeCount.edrData[j]++;
						if ((uint8_t)writeToOtp(si7013, otpMemoryMap.edrAddresses + j, otpData.inByte[j]) == (uint8_t)EdaCorrection::Status::SUCCESS)
						{
							otpMemoryMap.edrDataWritten[j] = true;
						}
						else
						{
							triedRegOverwrite = true;
						}
					}
					else
					{
						otpMemoryMap.writeCount.edrData[j]++;
					}
				}
				// writing the main EDL values
				for (uint8_t iterEdl = 0; iterEdl < NUM_EDL_READINGS; iterEdl++)
				{
					otpData.inFloat = correctionData.edlReadings[iterEdl];
					for (uint8_t byte = 0; byte < BYTES_PER_FLOAT; byte++)
					{
						if (otpMemoryMap.edlDataWritten[iterEdl][byte] == false)
						{
							otpMemoryMap.writeCount.edlData[iterEdl][byte]++;
							if ((uint8_t)writeToOtp(si7013, otpMemoryMap.edlAddresses[iterEdl] + byte, otpData.inByte[byte]) == (uint8_t)EdaCorrection::Status::SUCCESS)
							{
								otpMemoryMap.edlDataWritten[iterEdl][byte] = true;
							}
							else
							{
								triedRegOverwrite = true;
							}
						}
						else
						{
							otpMemoryMap.writeCount.edlData[iterEdl][byte]++;
						}
					}
				}


#else

				otpData.inFloat = correctionData.edlReadings[0];
				for (uint8_t byte = 0; byte < BYTES_PER_FLOAT; byte++)
				{
					if (otpMemoryMap.testDataWritten[byte] == false)
					{
						if ((uint8_t)writeToOtp(si7013, otpMemoryMap.edlTestAddress + byte, otpData.inByte[byte]) == (uint8_t)EdaCorrection::Status::SUCCESS)
						{
							otpMemoryMap.testDataWritten[byte] = true;
						}
						else
						{
							triedRegOverwrite = true;
						}
					}
					else
					{
						// write the case were u are trying to write to same location again
					}

				}

#endif
			_mode = EdaCorrection::Mode::NORMAL;
			powerCycled = false;
			return EdaCorrection::Status::SUCCESS;
			}
		}
		else
		{
			// sensor not present
			isSensorConnected = false;
			_mode = EdaCorrection::Mode::NORMAL;
			return EdaCorrection::Status::FAILURE;
		}

	}
}

EdaCorrection::Status EdaCorrection::readFromOtp(Si7013* si7013, bool isOtpOperation)
{
	if (!dummyWrite)
	{

		if (checkSensorConnection(si7013) == false)
		{
			isSensorConnected = false;
			readOtpValues = true;
			isOtpValid = false;
			return EdaCorrection::Status::FAILURE;
		}
#ifdef ACCESS_MAIN_ADDRESS
		if(si7013->readRegister8(otpMemoryMap.dataVersionAddr, isOtpOperation) == 255)
		//if ((uint8_t)readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EMOTIBIT_VERSION) == 255)
		{
			isOtpValid = false;
			readOtpValues = true;
			return EdaCorrection::Status::FAILURE;
		}
		otpDataFormat = (EdaCorrection::OtpDataFormat)si7013->readRegister8(otpMemoryMap.dataVersionAddr, isOtpOperation);
		_emotiBitVersion = (uint8_t)si7013->readRegister8(otpMemoryMap.emotiBitVersionAddr, isOtpOperation);

		for (uint8_t j = 0; j < BYTES_PER_FLOAT; j++)
		{
			otpData.inByte[j] = si7013->readRegister8(otpMemoryMap.edrAddresses + j, isOtpOperation);
		}
		correctionData.vRef2 = otpData.inFloat;
#endif
		// reading the main address block or the test address block
		for (uint8_t i = 0; i < NUM_EDL_READINGS; i++)
		{
			for (uint8_t j = 0; j < BYTES_PER_FLOAT; j++)
			{
#ifdef ACCESS_MAIN_ADDRESS
				otpData.inByte[j] = si7013->readRegister8(otpMemoryMap.edlAddresses[i] + j, isOtpOperation);
#else
				otpData.inByte[j] = si7013->readRegister8(otpMemoryMap.edlTestAddress + j, isOtpOperation);
#endif
			}
			correctionData.edlReadings[i] = otpData.inFloat;
		}
	}
	else
	{
		// do nothing here. displaycorrections() will echo the already saved variables.
	}
	readOtpValues = true;
}


void EdaCorrection::displayCorrections()
{
	Serial.println("EDL readings stored:");
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		Serial.print("edlReadings["); Serial.print(i); Serial.print("]: "); Serial.println(correctionData.edlReadings[i], FLOAT_PRECISION);
	}
	Serial.println("### Calculating values ####\n");
	Serial.print("Vref1: "); Serial.println(correctionData.vRef1, FLOAT_PRECISION);
	Serial.print("Vref2: "); Serial.println(correctionData.vRef2, FLOAT_PRECISION);
	Serial.print("Rfb: "); Serial.println(correctionData.Rfb, FLOAT_PRECISION);
}


EdaCorrection::Status EdaCorrection::calcEdaCorrection()
{
	correctionData.vRef1 = correctionData.edlReadings[0];
	correctionData.Rfb = 0;
	// Avg of 10K,100K and 1M
	/*
	for (int i = 1; i < 4; i++)
	{
		correctionData.Rfb += (correctionData.trueRskin[i] / ((correctionData.edlReadings[i] / correctionData.vRef1) - 1)); // use the EDl @10K, @100K, @1M
	}
	correctionData.Rfb = correctionData.Rfb / 3;// taking avg of 3 readings
	*/
	// using 1M to find Rfeedback
	correctionData.Rfb += (correctionData.trueRskin[3] / ((correctionData.edlReadings[3] / correctionData.vRef1) - 1));
	if (dummyWrite)
	{
		displayCorrections();
		correctionDataReady = true;
	}
	else
	{
#ifdef ACCESS_MAIN_ADDRESS

		displayCorrections();
		correctionDataReady = true;
#else
		Serial.print("No calculations performed. Just printing values read from the test Address space ");
#ifdef USE_ALT_SI7013
		Serial.println("of the alternate(external) sensor");
#else
		Serial.println("of the main EmotiBit sensor");
#endif

		Serial.print("correctionData.edlReadings[0]: "); Serial.println(correctionData.edlReadings[0], 6);
		progress = EdaCorrection::Progress::FINISH;// prevets the emotiBit class variables to be updated in normalModeoperations()
		correctionDataReady = false;
#endif
	}

	//calculationPerformed = true;
}

void EdaCorrection::OtpMemoryMap_V0::echoWriteCount()
{
	if (writeCount.dataVersion)// only echo if an attempt has been made to write to the OTP
	{
		Serial.println("\n::::Attempted write count::::");
		Serial.print("DataFormat:"); Serial.println(writeCount.dataVersion);
		Serial.print("EmotiBitVersion:"); Serial.println(writeCount.emotiBitVersion);
		Serial.println("edrVoltage(4 bytes):");
		for (int i = 0; i < 4; i++)
		{
			Serial.print("Byte "); Serial.print(i); Serial.print(":"); Serial.println(writeCount.edrData[i]);
		}
		Serial.println("edlVoltage(20Bytes):");
		for (int j = 0; j < NUM_EDL_READINGS; j++)
		{
			Serial.print("EdldataPoint "); Serial.print(j); Serial.println(":");
			for (int i = 0; i < 4; i++)
			{
				Serial.print("Byte "); Serial.print(i); Serial.print(":"); Serial.println(writeCount.edlData[j][i]);
			}
		}
	}
}