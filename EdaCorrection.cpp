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
	Serial.print("\Enter X to perform Actual OTP R/W\n");
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
	//if (!isSensorConnected)
	//{

	//	//readOtpValues = true;
	//	//progress = EdaCorrection::Progress::FINISH;
	//}

	if (readOtpValues && !correctionDataReady)
	{
		if (isOtpValid)// metadata presence is checked in the OTP read operation
		{
			Serial.print("The EmotiBit Version stored for EDA correction is: "); Serial.println((int)_emotiBitVersion);
			Serial.print("The data format version stored for EDA correction is: "); Serial.println((int)otpDataFormat);
			calcEdaCorrection();
		}
		else
		{
			if (isSensorConnected)
			{
				Serial.println("OTP has not been updated. Not performing correction calculation.");
				Serial.println("Perform EDA correction first.");
				Serial.println("Using EDA without correction");
				progress = EdaCorrection::Progress::FINISH;
			}
			else
			{
#ifdef USE_ALT_SI7013
				Serial.println("EDA Correction ERROR: External SI-7013 sensor not detected on I2C line. Please check connection");
#else
				Serial.println("EDA Correction ERROR: Main EmotiBit SI-7013 sensor not detected on I2C line. Please check connection");
#endif
				progress = EdaCorrection::Progress::FINISH;
			}
		}
	}

	// register overwrite occurs in UPDATE mode, but once it occurs, the mode is shifted to NORMAL and is detected here 
	if (triedRegOverwrite)
	{
		Serial.println("You are trying to overwrite a register, which is not allowed.");
		Serial.println("Aborting OTP R/W operations");
		Serial.println("Please check the OTP state before trying to write again");
		progress = EdaCorrection::Progress::FINISH;// prevents re-enterin normal mode operations
	}

	if (correctionDataReady)
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
	progress = EdaCorrection::Progress::FINISH;
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
		correctionData.vref2Readings[i - NUM_EDL_READINGS] = input[i];
	}
	
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		correctionData.vRef2 += correctionData.vref2Readings[i];
	}
	correctionData.vRef2 = correctionData.vRef2 / NUM_EDL_READINGS;
	
	// updating the inClass otpBuffer.
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		otpData.inFloat = correctionData.edlReadings[i];
		for (int j = 0; j < 4; j++)
		{
			correctionData.otpBuffer[i * 4 + j] = otpData.inByte[j];
		}
	}
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
				// clearing all class members with old values
				for (int i = 0; i < MAX_OTP_SIZE; i++)
				{
					correctionData.otpBuffer[i] = 0;
				}
				correctionData.vRef1 = 0; correctionData.vRef2 = 0; correctionData.Rfb = 0;
				for (int i = 0; i < NUM_EDL_READINGS; i++)
				{
					correctionData.edlReadings[i] = 0;
					correctionData.vref2Readings[i] = 0;
				}
				
				// ask to enter the second time 
				Serial.println("back to Progress::WAITING_FOR_SERIAL_DATA");
				Serial.println("Enter the eda values into the serial monitor");
				progress = EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA;
				_responseRecorded = false;
			}
		}
	}
	//if (!isSensorConnected)
	//{
	//	Serial.println("Sensor not found. Did not execute R/W operations.");
	//	Serial.println("Please check I2C connections");
	//	_mode = EdaCorrection::Mode::NORMAL;
	//}

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
		Serial.print(correctionData.vref2Readings[i], FLOAT_PRECISION); Serial.print("\t");
	}
	Serial.print("\nAvg Vref2 :"); Serial.println(correctionData.vRef2, FLOAT_PRECISION);
	//_EdaReadingsPrinted = true;

	Serial.println("\nIf you are in Dummy mode, NO OTP R/W ACTIONS WILL BE PERFORMED");
	Serial.println("This is how the OTP is will be updated(If NOT IN DUMMY MODE)");
	Serial.println("You can change R/W locations in OTP by toggling ACCESS_MAIN_ADDRESS\n");

#ifdef ACCESS_MAIN_ADDRESS
	uint8_t startAddr = SI_7013_OTP_ADDRESS_EDL_TABLE;
#else
	uint8_t startAddr = SI_7013_OTP_ADDRESS_TEST;
#endif
	for (int i = startAddr,j=0; i < startAddr + OTP_SIZE_IN_USE; i++, j++)
	{
		Serial.print("0x"); Serial.print(i, HEX); Serial.print(" : "); 
		Serial.print(correctionData.otpBuffer[i - startAddr], DEC);
		if (j % 4 == 0)
		{
			Serial.print(" -- "); Serial.println(correctionData.edlReadings[j/4], FLOAT_PRECISION);
		}
		else
		{
			Serial.println();
		}
	}
#ifdef ACCESS_MAIN_ADDRESS
	otpData.inFloat = correctionData.vRef2;
	Serial.println();
	Serial.print("vRef2: ");
	Serial.println(correctionData.vRef2, FLOAT_PRECISION);
	for (int i = (int)SI_7013_OTP_ADDRESS_VREF2; i < (int)SI_7013_OTP_ADDRESS_VREF2 + BYTES_PER_FLOAT; i++)
	{
		Serial.print("0x"); Serial.print(i, HEX); Serial.print(" : ");
		Serial.println(otpData.inByte[i - (int)SI_7013_OTP_ADDRESS_VREF2], DEC);
	}
	// metadata
	Serial.println("\nmetadata");
	Serial.print("0x"); Serial.print(SI_7013_OTP_ADDRESS_DATA_FORMAT, HEX); Serial.print(" : ");
	Serial.println((uint8_t)otpDataFormat, DEC);
	Serial.print("0x"); Serial.print(SI_7013_OTP_ADDRESS_EMOTIBIT_VERSION, HEX); Serial.print(" : ");
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

bool EdaCorrection::checkSensorConnection(TwoWire* emotiBit_i2c)
{
	emotiBit_i2c->beginTransmission(_si7013Addr);
	emotiBit_i2c->write((int)0x00);
	if (emotiBit_i2c->endTransmission())
	{
		isSensorConnected = false;
		return false;
	}
	else
	{
		return true;
	}
}

EdaCorrection::Status EdaCorrection::writeToOtp(TwoWire* emotiBit_i2c, uint8_t addr, char val)
{
	if (isOtpRegWritten(emotiBit_i2c, addr))
	{
		return EdaCorrection::Status::FAILURE;
	}
	emotiBit_i2c->beginTransmission(_si7013Addr);
	emotiBit_i2c->write(SI_7013_CMD_OTP_WRITE);
	emotiBit_i2c->write(addr);
	emotiBit_i2c->write(val);
	emotiBit_i2c->endTransmission();
	return EdaCorrection::Status::SUCCESS;
}

EdaCorrection::Status EdaCorrection::writeToOtp(TwoWire* emotiBit_i2c)
{ 

	/*for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		otpData.inFloat = correctionData.edlReadings[i];
		for (int j = 0; j < 4; j++)
		{
			correctionData.otpBuffer[i * 4 + j] = otpData.inByte[j];
		}
	}*/
	if (dummyWrite)
	{
		_mode = EdaCorrection::Mode::NORMAL;
	}
	else
	{

		if (checkSensorConnection(emotiBit_i2c))
		{
			if (progress == EdaCorrection::Progress::WRITING_TO_OTP && triedRegOverwrite == false)
			{

#ifdef ACCESS_MAIN_ADDRESS
				// writing the metadata
				if (writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_DATA_FORMAT, (uint8_t)otpDataFormat) != EdaCorrection::Status::SUCCESS ||
					writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EMOTIBIT_VERSION, _emotiBitVersion) != EdaCorrection::Status::SUCCESS)
				{
					triedRegOverwrite = true;
					_mode = EdaCorrection::Mode::NORMAL;
					return EdaCorrection::Status::FAILURE;
				}

				// writing the vref 2 to OTP
				otpData.inFloat = correctionData.vRef2;
				for (uint8_t j = 0; j < 4; j++)
				{
					if (writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_VREF2 + j, otpData.inByte[j]) != EdaCorrection::Status::SUCCESS)
					{
						triedRegOverwrite = true;
						_mode = EdaCorrection::Mode::NORMAL;
						return EdaCorrection::Status::FAILURE;
					}
				}
				// writing the main EDL values
				for (uint8_t offset = 0; offset < OTP_SIZE_IN_USE; offset++)
				{
					if (writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EDL_TABLE + offset, correctionData.otpBuffer[offset]) != EdaCorrection::Status::SUCCESS)
					{
						triedRegOverwrite = true;
						_mode = EdaCorrection::Mode::NORMAL;
						return EdaCorrection::Status::FAILURE;
					}
				}


#else
				for (uint8_t i = 0; i < OTP_SIZE_IN_USE; i++)// 8 or 20 depending on the main address space access or aux addr. space access
				{
					if (writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_TEST + i, correctionData.otpBuffer[i]) != EdaCorrection::Status::SUCCESS)
					{
						triedRegOverwrite = true;
						_mode = EdaCorrection::Mode::NORMAL;
						return EdaCorrection::Status::FAILURE;
					}
				}
#endif
			_mode = EdaCorrection::Mode::NORMAL;
			}
		// after writing to the OTP, change the mode to normal
		}
		else
		{
			_mode = EdaCorrection::Mode::NORMAL;
			return EdaCorrection::Status::FAILURE;
		}

	}
}


uint8_t EdaCorrection::readFromOtp(TwoWire* emotiBit_i2c, uint8_t addr)
{
	emotiBit_i2c->beginTransmission(_si7013Addr);
	emotiBit_i2c->write(SI_7013_CMD_OTP_READ);
	emotiBit_i2c->write(addr);
	emotiBit_i2c->endTransmission();
	emotiBit_i2c->requestFrom(_si7013Addr, 1);
	if (emotiBit_i2c->available())
	{
		return(emotiBit_i2c->read());
	}
}

EdaCorrection::Status EdaCorrection::readFromOtp(TwoWire* emotiBit_i2c)
{
	if (!dummyWrite)
	{

		if (checkSensorConnection(emotiBit_i2c) == false)
		{
			isSensorConnected = false;
			readOtpValues = true;
			isOtpValid = false;
			return EdaCorrection::Status::FAILURE;
		}
#ifdef ACCESS_MAIN_ADDRESS
		if ((uint8_t)readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EMOTIBIT_VERSION) == 255)
		//if ((uint8_t)readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EMOTIBIT_VERSION) != 3)
		{
			isOtpValid = false;
			readOtpValues = true;
			return EdaCorrection::Status::FAILURE;
		}
		otpDataFormat = (EdaCorrection::OtpDataFormat)readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_DATA_FORMAT);
		_emotiBitVersion = (uint8_t)readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EMOTIBIT_VERSION);

		for (uint8_t j = 0; j < BYTES_PER_FLOAT; j++)
		{
			otpData.inByte[j] = readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_VREF2 + j);
		}
		correctionData.vRef2 = otpData.inFloat;
#endif
		// reading the main address block or the test address block
		for (uint8_t i = 0; i < OTP_SIZE_IN_USE / BYTES_PER_FLOAT; i++)
		{
			for (uint8_t j = 0; j < BYTES_PER_FLOAT; j++)
			{
#ifdef ACCESS_MAIN_ADDRESS
				otpData.inByte[j] = readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EDL_TABLE + BYTES_PER_FLOAT * i + j);
#else
				otpData.inByte[j] = readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_TEST + BYTES_PER_FLOAT * i + j);
#endif
			}
			correctionData.edlReadings[i] = otpData.inFloat;
		}
	}
	else
	{
		for (uint8_t i = 0; i < NUM_EDL_READINGS; i++)
		{
			correctionData.edlReadings[i] = 0;
			for (uint8_t j = 0; j < BYTES_PER_FLOAT; j++)
			{
				otpData.inByte[j] = correctionData.otpBuffer[4 * i + j];
			}
			correctionData.edlReadings[i] = otpData.inFloat;
		}
	}
	readOtpValues = true;
}

bool EdaCorrection::isOtpRegWritten(TwoWire* emotiBit_i2c, uint8_t addr)
{
	if ((uint8_t)readFromOtp(emotiBit_i2c, addr) == 255)
	{
		return false;
	}
	else
	{
		return true;
	}
}


void EdaCorrection::displayCorrections()
{
	Serial.println("EDL readings stored:");
	for (int i = 0; i < NUM_EDL_READINGS; i++)
	{
		Serial.print("edlReadings["); Serial.print(i); Serial.print("]: "); Serial.println(correctionData.edlReadings[i], 6);
	}
	Serial.println("### Calculating values ####\n");
	Serial.print("Vref1: "); Serial.println(correctionData.vRef1, 6);
	Serial.print("Vref2: "); Serial.println(correctionData.vRef2, 6);
	Serial.print("Rfb: "); Serial.println(correctionData.Rfb, 6);
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
		for (int i = 0; i < OTP_SIZE_IN_USE/ BYTES_PER_FLOAT; i++)
		{
			Serial.print("correctionData.edlReadings["); Serial.print(i); Serial.print("]: "); Serial.println(correctionData.edlReadings[i], 6);
		}
		progress = EdaCorrection::Progress::FINISH;// prevets the emotiBit class variables to be updated in normalModeoperations()
		correctionDataReady = false;
#endif
	}

	//calculationPerformed = true;
}

