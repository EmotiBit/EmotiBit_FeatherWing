#include "EDA_Correction.h"

EdaCorrection::Status EdaCorrection::enterUpdateMode()
{
	Serial.println("Entered the Update mode");
	Serial.println("Do you wish to switch to update mode? Press Y for yes and N for No");
	while (!Serail.available());
	if (Serail.read() == 'Y')
	{
		Serial.println("setting class mode to Mode::UPDATE");
		_mode = EdaCorrection::Mode::UPDATE;
		Serial.println("Initializing state machine. State: WAITING_FOR_SERIAL_DATA");
		Serial.print("Once you have the appropriate data, please plug in the serail cable and enter the data.");
		Serial.print("proceed with normal execution of emotibit");
		return EdaCorrection::Status::SUCCESS;
	}
	else
	{
		Serial.println("No by user. Class in NORMAL mode");
		return EdaCorrection::Status::FAILURE;
	}
}

EdaCorrection::Mode EdaCorrection::getClassMode()
{
	return _mode;
}

EdaCorrection::Status EdaCorrection::readFloatFromSerial()
{

}


EdaCorrection::Status EdaCorrection::setFloatValues()
{

}


void EdaCorrection::echoFloatOnScreen()
{


}


bool EdaCorrection::getUserApproval()
{

}

void EdaCorrection::setApprovalStatus()
{

}


bool EdaCorrection::getApprovalStatus()
{

}


EdaCorrection::Status EdaCorrection::writeToOtp()
{

}

EdaCorrection::Status EdaCorrection::readFromOtp()
{

}

EdaCorrection::Status EdaCorrection::calcCorrectionFromOtp()
{

}

