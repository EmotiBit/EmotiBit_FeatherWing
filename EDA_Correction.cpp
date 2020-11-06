#include "EDA_Correction.h"

EdaCorrection::Status EdaCorrection::enterUpdateMode()
{
	Serial.println("Do you wish to switch to update mode? Press Y for yes and N for No");
	while (!Serial.available());
	if (Serial.read() == 'Y')
	{
		Serial.println("setting edaCorrection class mode to Mode::UPDATE");
		_mode = EdaCorrection::Mode::UPDATE;
		Serial.println("Initializing state machine. State: WAITING_FOR_SERIAL_DATA");
		Serial.print("Once you have the appropriate data, please plug in the serial cable and enter the data.");
		Serial.print("proceed with normal execution of emotibit");
		return EdaCorrection::Status::SUCCESS;
	}
	else
	{
		Serial.println("exiting... Class in NORMAL mode");
		return EdaCorrection::Status::FAILURE;
	}
}

EdaCorrection::Mode EdaCorrection::getMode()
{
	return _mode;
}

EdaCorrection::Status EdaCorrection::readFloatFromSerial()
{
	// do a time check here or in the emotibit update function
	if (_mode == EdaCorrection::Mode::UPDATE)
	{
		if (progress == EdaCorrection::Progress::WAITING_FOR_SERIAL_DATA)
		{
			if (Serial.available())
			{
				for (int i = 0; i < NUM_EDA_READINGS; i++)
				{
					edaReadings[i] = Serial.parseFloat();
				}
			}
			// advance the state
			echoEdaReadingsOnScreen();
			progress = EdaCorrection::Progress::WAITING_USER_APPROVAL;
		}
		else if (progress == EdaCorrection::Progress::WAITING_USER_APPROVAL)
		{
			if (_responseRecorded = false)
			{
				getUserApproval();
			}
			else
			{
				if (getApprovalStatus() == true)
				{
					progress = EdaCorrection::Progress::WRITING_TO_OTP;
				}
				else
				{
					//ToDo: user denied writing to OTP
				}
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
	for (int i = 0; i < NUM_EDA_READINGS; i++)
	{
		Serial.print(edaReadings[i]); Serial.print("\t");
	}
	_EdaReadingsPrinted = true;
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
		}
		else if (response == 'N')
		{
			setApprovalStatus(false);
		}
		//ToDO: add exception is user inputs something else

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
	if (_mode == EdaCorrection::Mode::UPDATE)
	{
		if (progress == EdaCorrection::Progress::WRITING_TO_OTP)
		{
			Serial.println("WRITING TO OTP!!!!!!");
			
			union Data {
				float edaReading; // 0, 10K, 100K, 1M, 10M
				char buff[4];// buffer to store the float in BYTE form
			};

			Serial.println("Wriiten to OTP. Exiting");
		}

		// after writing to the OTP, the mode become normal
		_mode = EdaCorrection::Mode::NORMAL;
	}
}

EdaCorrection::Status EdaCorrection::readFromOtp()
{
	Serial.println("Reading from OTP");
	union Data {
		float edaReading; // 0, 10K, 100K, 1M, 10M
		char buff[4];// buffer to store the float in BYTE form
	};
	// read from OTP into the buff
	
}

EdaCorrection::Status EdaCorrection::calcEdaCorrection()
{
	// perform correction
}

