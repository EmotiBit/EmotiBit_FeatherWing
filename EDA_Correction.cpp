#include "EDA_Correction.h"


EdaCorrection::Status EdaCorrection::enterUpdateMode()
{
	
	Serial.println("Setting edaCorrection class mode to Mode::UPDATE");
	_mode = EdaCorrection::Mode::UPDATE;
	Serial.print("Do you want to activate dummy write?"); Serial.println("Press Y for Yes and N for no");
	while (!Serial.available());
	if (Serial.read() == 'Y')
	{
		dummyWrite = true;
		Serial.println("In dummy mode");
	}
	else
	{
		Serial.println("!!!!!!!!Actually writing to the OTP!!!!");
	}
	Serial.println("Initializing state machine. State: WAITING_FOR_SERIAL_DATA");
	progress = EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA;
	Serial.print("Once you have the appropriate data, please plug in the serial cable and enter the data.");
	Serial.print("proceed with normal execution of emotibit in\n");
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

EdaCorrection::Status EdaCorrection::readFloatFromSerial()
{

	if (progress == EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA)
	{
		if (Serial.available())
		{
			//ToDo: implement a check if the input is indeed 5 float values
			Serial.println("Serial data detected. Reading from Serial");

			if (dummyWrite)
			{
				Serial.println("Writing into dummy float array");
				for (int i = 0; i < NUM_EDA_READINGS; i++)
				{
					dummyEdaReadings[i] = Serial.parseFloat();
				}
			}
			else
			{
				Serial.println("Updating class with Eda readings");
				for (int i = 0; i < NUM_EDA_READINGS; i++)
				{
					edaReadings[i] = Serial.parseFloat();
				}
			}
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
				Serial.println("Proceeding to write into OTP!!");
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
	if (dummyWrite)
	{
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			Serial.print(dummyEdaReadings[i], 6); Serial.print("\t");
		}
	}
	else
	{
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			Serial.print(edaReadings[i], 6); Serial.print("\t");
		}
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
#ifdef EDA_TESTING
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
			char buff[4];// buffer to store the float in BYTE form
		}dummyData;
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			dummyData.edaReading = dummyEdaReadings[i];
			for (int j = 0; j < 4; j++)
			{
				dummyOtp[i * 4 + j] = dummyData.buff[j];
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
				float edaReading; 
				char buff[4];// buffer to store the float in BYTE form
			}data;
#ifdef EDA_TESTING
			for (uint8_t i = 0; i < 2; i++)//count first 2 readings from the EDA array
			{
				data.edaReading = edaReadings[i];
				for (uint8_t j = 0; j < 4; j++)// count 4 bytes
				{
					if (!isOtpRegWritten(emotiBit_i2c, SI_7013_OTP_ADDRESS_TEST_1 + 4 * i + j))
					{
						writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_TEST_1 + 4 * i + j, data.buff[j]);
					}
					else
					{
						triedRegOverwrite = true;
						_mode = EdaCorrection::Mode::NORMAL;
						return EdaCorrection::Status::FAILURE;
					}
				}
			}
#else
			for (uint8_t i = 0; i < NUM_EDA_READINGS; i++)
			{
				data.edaReading = edaReadings[i];
				for (uint8_t j = 0; j < 4; j++)
				{
					if (!isOtpRegWritten(emotiBit_i2c, SI_7013_OTP_ADDRESS_FLOAT_0 + 4 * i + j))
					{
						writeToOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_FLOAT_0 + 4 * i + j, data.buff[j]);
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
			// Serial.println("\n########OTP UPDATED. Exiting########");
		}

		// after writing to the OTP, the mode become normal
		_mode = EdaCorrection::Mode::NORMAL;
	}
}


uint8_t EdaCorrection::readFromOtp(TwoWire* emotiBit_i2c, uint8_t addr)
{
#ifdef EDA_TESTING
	emotiBit_i2c->beginTransmission(SI_7013_I2C_ADDR_ALT);
#else
	emotiBit_i2c->beginTransmission(SI_7013_I2C_ADDR_MAIN);
#endif
	emotiBit_i2c->write(SI_7013_CMD_OTP_READ);
	emotiBit_i2c->write(addr);
	emotiBit_i2c->endTransmission();
	emotiBit_i2c->requestFrom(SI_7013_I2C_ADDR_ALT, 1);
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
			float edaReading; // 0, 10K, 100K, 1M, 10M
			char buff[4];// buffer to store the float in BYTE form
		}dummyData;
		//Serial.println("Reading from dummy OTP");
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				dummyData.buff[j] = dummyOtp[i * 4 + j];
			}
			edaReadings[i] = dummyData.edaReading;
			//Serial.print(dummyData.edaReading,6); Serial.print("\t");
		}
		//Serial.println();
	}
	else
	{
		//Serial.println("Reading from OTP");
		union Data {
			float edaReading; // 0, 10K, 100K, 1M, 10M
			char buff[4];// buffer to store the float in BYTE form
		}data;
		// read from OTP into the buff
		// Serial.print("0x"); Serial.print(addr, HEX);
#ifdef EDA_TESTING
		for (uint8_t i = 0; i < 2; i++)
		{
			for (uint8_t j = 0; j < 4; j++)
			{
				data.buff[j] = readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_TEST_1 + 4 * i + j);
			}
			edaReadings[i] = data.edaReading;
		}
#else
		for (uint8_t i = 0; i < NUM_EDA_READINGS; i++)
		{
			for (uint8_t j = 0; j < 4; j++)
			{
				data.buff[j] = datareadFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_FLOAT_0 + 4 * i + j);
			}
			edaReadings[i] = data.edaReading;
		}
#endif

		//Serial.print(" : "); Serial.println(otpByte);
	}
	//Serial.println("DONE reading the OTP.");
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
	if ((uint8_t)readFromOtp(emotiBit_i2c, SI_7013_OTP_ADDRESS_METADATA) == 255)
	{
		Serial.println("OTP has not been updated. Not performing correction calculation.");
		Serial.println("Perform EDA correction first.");
		Serial.println("Using EDA with correction");
	}
	else
	{

	// perform correction
#ifdef EDA_TESTING
	Serial.println("Done with the calculation. The values are");
	Serial.print("edaReadings[0]: "); Serial.println(edaReadings[0], 6);
	Serial.print("edaReadings[1]: "); Serial.println(edaReadings[1], 6);
#else
	for (int i = 0; i < NUM_EDA_READINGS; i++)
	{
		Serial.print("edaReadings["); Serial.print(i); Serial.print("]: "); Serial.println(edaReadings[i], 6);
	}
#endif
	}
	calculationPerformed = true;
}

