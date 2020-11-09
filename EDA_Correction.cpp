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
	Serial.print("proceed with normal execution of emotibit\n\n");
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

/*
EdaCorrection::Status EdaCorrection::setFloatValues()
{

}
*/

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


EdaCorrection::Status EdaCorrection::writeToOtp()
{ 
	if (dummyWrite)
	{
		Serial.println("DUMMY DUMMY DUMMY");
		union Data {
			float edaReading; // 0, 10K, 100K, 1M, 10M
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
		Serial.println("Changing mode to NORMAL");
		_mode = EdaCorrection::Mode::NORMAL;
	}
	else
	{
		if (progress == EdaCorrection::Progress::WRITING_TO_OTP)
		{
			Serial.println("WRITING TO OTP!!!!!!");

			union Data {
				float edaReading; // 0, 10K, 100K, 1M, 10M
				char buff[4];// buffer to store the float in BYTE form
			}data;

			for (int i = 0; i < NUM_EDA_READINGS; i++)
			{
				data.edaReading = edaReadings[i];
				for (int j = 0; j < 4; j++)
				{
					Serial.print(data.buff[j]);
				}
			}

			Serial.println("\nWritten to OTP. Exiting");
		}

		// after writing to the OTP, the mode become normal
		_mode = EdaCorrection::Mode::NORMAL;
	}
}

EdaCorrection::Status EdaCorrection::readFromOtp()
{
	if (dummyWrite)
	{
		union Data {
			float edaReading; // 0, 10K, 100K, 1M, 10M
			char buff[4];// buffer to store the float in BYTE form
		}dummyData;
		Serial.println("Reading from dummy OTP");
		for (int i = 0; i < NUM_EDA_READINGS; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				dummyData.buff[j] = dummyOtp[i * 4 + j];
			}
			Serial.print(dummyData.edaReading,6); Serial.print("\t");
		}
		Serial.println();
	}
	else
	{
		Serial.println("Reading from OTP");
		union Data {
			float edaReading; // 0, 10K, 100K, 1M, 10M
			char buff[4];// buffer to store the float in BYTE form
		};
		// read from OTP into the buff
	}
	Serial.println("DONE reading the OTP. flipping read switch");
	readOtpValues = true;
}

EdaCorrection::Status EdaCorrection::calcEdaCorrection()
{
	// perform correction

	Serial.println("Done with the calculation. THe values are");
	Serial.println("val 1, val 2, val 3");
	calculationPerformed = true;
}

