#include "EDA_Correction.h"


EdaCorrection::Status EdaCorrection::enterUpdateMode()
{
	
	Serial.println("Enabling Mode::UPDATE");
	_mode = EdaCorrection::Mode::UPDATE;
	Serial.print("\n**Enter Dummy mode?**\n"); Serial.println("\tPress Y to work in dummy mode, N to actually write to OTP");
	while (!Serial.available());
	if (Serial.read() == 'Y')
	{
		dummyWrite = true;
		Serial.println("###################");
		Serial.println("## IN DUMMY MODE ##");
		Serial.println("###################");
	}
	else
	{
		Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		Serial.println("!!!! Actually writing to the OTP  !!!!");
		Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	}
	Serial.println("Initializing state machine. State: WAITING_FOR_SERIAL_DATA");
	progress = EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA;
	Serial.println("Once you have the appropriate data, please plug in the serial cable and enter the data.");
	Serial.print("The input should be in the format:");
	Serial.println("EDL_0R, EDL_10K, EDL_100K, EDL_1M, EDL_10M, vRef2_0R, vRef2_10K, vRef2_100K, vRef2_1M, vRef2_10M");
	Serial.print("Proceeding with normal execution in\n");
	uint8_t msgTimer = 5;
	while (msgTimer > 0)
	{
		Serial.print(msgTimer); Serial.print("\t");
		delay(1000);
		msgTimer--;
	}
	return EdaCorrection::Status::SUCCESS;

}

EdaCorrection::Mode EdaCorrection::getMode()
{
	return _mode;
}

void EdaCorrection::getFloatFromString()
{
	float input[2 * NUM_EDA_READINGS];
	for (int i = 0; i < 2 * NUM_EDA_READINGS; i++)
	{
		String splitString = Serial.readStringUntil(',');
		input[i] = splitString.toFloat();
	}

	for (int i = 0; i < NUM_EDA_READINGS; i++)
	{
		edaReadings[i] = input[i];
	}

	for (int i = NUM_EDA_READINGS; i < 2 * NUM_EDA_READINGS; i++)
	{
		vref2Readings[i - NUM_EDA_READINGS] = input[i];
	}

}

EdaCorrection::Status EdaCorrection::monitorSerial()
{

	if (progress == EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA)
	{
		if (Serial.available())
		{
			//ToDo: implement a check if the input is indeed 5 float values
			Serial.println("Serial data detected. Reading from Serial");
			getFloatFromString();
			echoEdaReadingsOnScreen();
			progress = EdaCorrection::Progress::WAITING_USER_APPROVAL;
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
				progress = EdaCorrection::Progress::WRITING_TO_OTP;
			}
			else
			{
				//ToDo: user denied writing to OTP
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
	for (int i = 0; i < NUM_EDA_READINGS; i++)
	{
		Serial.print(edaReadings[i], 6); Serial.print("\t");
	}
	Serial.println("\nVref2 readings:");
	for (int i = 0; i < NUM_EDA_READINGS; i++)
	{
		Serial.print(vref2Readings[i], 6); Serial.print("\t");
	}

	//_EdaReadingsPrinted = true;
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

EdaCorrection::Status EdaCorrection::writeToOtp(TwoWire* emotiBit_i2c, uint8_t addr, char val)
{
#ifdef USE_ALT_SI7013
	emotiBit_i2c->beginTransmission(SI_7013_I2C_ADDR_ALT);
#else
	emotiBit_i2c->beginTransmission(SI_7013_I2C_ADDR_MAIN);
#endif
	emotiBit_i2c->write(SI_7013_CMD_OTP_WRITE);
	emotiBit_i2c->write(addr);
	emotiBit_i2c->write(val);
	emotiBit_i2c->endTransmission();
}

EdaCorrection::Status EdaCorrection::writeToOtp(TwoWire* emotiBit_i2c)
{ 
	if (dummyWrite)
	{
		// Serial.println("DUMMY DUMMY DUMMY");
		union Data {
			float edaReading; 
			char byte[4];// buffer to store the float in BYTE form
		}data;
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			data.edaReading = edaReadings[i];
			for (int j = 0; j < 4; j++)
			{
				dummyOtp[i * 4 + j] = data.byte[j];
			}
		}
		_mode = EdaCorrection::Mode::NORMAL;
	}
	else
	{
		if (progress == EdaCorrection::Progress::WRITING_TO_OTP && triedRegOverwrite == false)
		{
			// Serial.println("#######WRITING TO OTP######");

			union Data {
				float floatValue;
				char byte[4];// buffer to store the float in BYTE form
			}data;

#ifdef WRITE_MAIN_ADDRESS
			for (uint8_t i = 0; i < NUM_EDA_READINGS; i++)
			{
				data.floatValue = edaReadings[i];
				for (uint8_t j = 0; j < 4; j++)
				{
					if (!isOtpRegWritten(emotiBit_i2c, SI_7013_OTP_ADDRESS_EDL_TABLE + 4 * i + j))
					{
						writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EDL_TABLE + 4 * i + j, data.byte[j]);
					}
					else
					{
						triedRegOverwrite = true;
						_mode = EdaCorrection::Mode::NORMAL;
						return EdaCorrection::Status::FAILURE;
					}
				}
			}

			// writing the vref 2 value
			float vRef2_temp = 0;
			for (int i = 0; i < NUM_EDA_READINGS; i++)
			{
				vRef2_temp += vref2Readings[i];
			}
			vRef2_temp = vRef2_temp / NUM_EDA_READINGS;
			data.floatValue = vRef2_temp;
			for (uint8_t j = 0; j < 4; j++)
			{
				if (!isOtpRegWritten(emotiBit_i2c, SI_7013_OTP_ADDRESS_VREF2 + j))
				{
					writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_VREF2 + j, data.byte[j]);
				}
				else
				{
					triedRegOverwrite = true;
					_mode = EdaCorrection::Mode::NORMAL;
					return EdaCorrection::Status::FAILURE;
				}
			}

			// writing the metadata
			if (!isOtpRegWritten(emotiBit_i2c, SI_7013_OTP_ADDRESS_METADATA) && !isOtpRegWritten(emotiBit_i2c, SI_7013_OTP_ADDRESS_METADATA + 1))
			{
				writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_METADATA, DATA_FORMAT_VERSION);
				writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_METADATA + 1, EMOTIBIT_VERSION);
			}
			else
			{
				triedRegOverwrite = true;
				_mode = EdaCorrection::Mode::NORMAL;
				return EdaCorrection::Status::FAILURE;
			}

#else
			
			for (uint8_t i = 0; i < 2; i++)//count first 2 readings from the EDA array
			{
				data.floatValue = edaReadings[i];
				for (uint8_t j = 0; j < 4; j++)// count 4 bytes
				{
					if (!isOtpRegWritten(emotiBit_i2c, SI_7013_OTP_ADDRESS_TEST_1 + 4 * i + j))
					{
						writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_TEST_1 + 4 * i + j, data.byte[j]);
					}
					else
					{
						triedRegOverwrite = true;
						_mode = EdaCorrection::Mode::NORMAL;
						return EdaCorrection::Status::FAILURE;
					}
				}
			}
#endif
		}

		// after writing to the OTP, the mode become normal
		_mode = EdaCorrection::Mode::NORMAL;
	}
}


uint8_t EdaCorrection::readFromOtp(TwoWire* emotiBit_i2c, uint8_t addr)
{
#ifdef USE_ALT_SI7013
	emotiBit_i2c->beginTransmission(SI_7013_I2C_ADDR_ALT);
#else
	emotiBit_i2c->beginTransmission(SI_7013_I2C_ADDR_MAIN);
#endif
	emotiBit_i2c->write(SI_7013_CMD_OTP_READ);
	emotiBit_i2c->write(addr);
	emotiBit_i2c->endTransmission();
#ifdef USE_ALT_SI7013
	emotiBit_i2c->requestFrom(SI_7013_I2C_ADDR_ALT, 1);
#else
	emotiBit_i2c->requestFrom(SI_7013_I2C_ADDR_MAIN, 1);
#endif
	if (emotiBit_i2c->available())
	{
		return(emotiBit_i2c->read());
	}
}

EdaCorrection::Status EdaCorrection::readFromOtp(TwoWire* emotiBit_i2c)
{
	if (dummyWrite)
	{
		union Data {
			float edaReading; 
			char byte[4];// buffer to store the float in BYTE form
		}data;
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				data.byte[j] = dummyOtp[i * 4 + j];
			}
			edaReadings[i] = data.edaReading;
		}
	}
	else
	{
		union Data {
			float floatValue; 
			char byte[4];// buffer to store the float in BYTE form
		}data;
#ifdef USE_ALT_SI7013
		for (uint8_t i = 0; i < 2; i++)
		{
			for (uint8_t j = 0; j < 4; j++)
			{
				data.byte[j] = readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_TEST_1 + 4 * i + j);
			}
			edaReadings[i] = data.floatValue;
		}
#else
		if ((uint8_t)readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_METADATA) == 255)
		{
			isOtpValid = false;
			readOtpValues = true;
			return EdaCorrection::Status::FAILURE;
		}
		
		for (uint8_t i = 0; i < NUM_EDA_READINGS; i++)
		{
			for (uint8_t j = 0; j < 4; j++)
			{
				data.byte[j] = readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_EDL_TABLE + 4 * i + j);
			}
			edaReadings[i] = data.floatValue;
		}
		for (uint8_t j = 0; j < 4; j++)
		{
			data.byte[j] = readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_VREF2 + j);
		}
		vRef2 = data.floatValue;
#endif

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

EdaCorrection::Status EdaCorrection::calcEdaCorrection(TwoWire* emotiBit_i2c)
{
	// perform a check to see if the last byte is written to
	if (dummyWrite)
	{
		Serial.println("The values stored on the mock OTP are:");
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			Serial.print("edaReadings["); Serial.print(i); Serial.print("]: "); Serial.println(edaReadings[i], 6);
		}
		Serial.println("### Calculating values ####\n");
		vRef1 = edaReadings[0];
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			vRef2 += vref2Readings[i];
		}
		vRef2 = vRef2 / NUM_EDA_READINGS;
		Rfb = 0;
		for (int i = 1; i < 4; i++)
		{
			Rfb += (trueRskin[i] / ((edaReadings[i] / vRef1) - 1)); // use the EDl @10K, @100K, @1M
		}
		Rfb = Rfb / 3;// taking avg of 3 readings
		Serial.print("Vref1: "); Serial.println(vRef1, 6);
		Serial.print("Vref2: "); Serial.println(vRef2, 6);
		Serial.print("Rfb: "); Serial.println(Rfb, 6);
		correctionDataReady = true;
	}
	else
	{
#ifdef USE_ALT_SI7013
		Serial.println("No claculations performed. Just reading values read from the OTP of the Alternate SI chip.");
		for (int i = 0; i < 2; i++)
		{
			Serial.print("edaReadings["); Serial.print(i); Serial.print("]: "); Serial.println(edaReadings[i], 6);
		}
#else
		Serial.println("The values stored on the mock OTP are:");
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			Serial.print("edaReadings["); Serial.print(i); Serial.print("]: "); Serial.println(edaReadings[i], 6);
		}
		Serial.println("### Calculating values ####\n");
		vRef1 = edaReadings[0];
		//vRef2 is automatically updated in the readFromOtp function
		Rfb = (100000 / ((edaReadings[2] / vRef1) - 1)); // use the EDl @100K
		Serial.print("Vref1: "); Serial.println(vRef1, 6);
		Serial.print("Vref2: "); Serial.println(vRef2, 6);
		Serial.print("Rfb: "); Serial.println(Rfb, 6);
		correctionDataReady = true;
#endif
	}

	calculationPerformed = true;
}

