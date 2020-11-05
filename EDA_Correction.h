/*
This class has been designed to work asynchronously as writing to the SI-7013 chip can be performed only using I2c and
emotibit only allows I2C communication during ISR.
The class will work in 2 modes
1. UPDATE mode, where the class will update the OTP
2. NORMAL mode, which is the general case mose, where the constants already exist in the OTP
	and will be read during startup
UPDATE MODE
*******************
The class is a state machine with the following states:
1. WAITING_FOR_SERIAL_DATA
2. WAITING_FOR_USER_APPROVAL
3. WRITING_TO_OTP
4. FINISH
Code flow:
1. The user activates this special correction class on startup by pressing a "special" key before
	emotibit starts setup.
	1. This changes the class state to the initial state of WAITING_FOR_SERIAL_DATA
2. Once the special mode has been activated, the EmotiBit periodically listens on the serial monitor
	for input.
	1. The input is the array of characters copied from the Oscilloscope testing helper terminal
	in the Acute Testing mode.
	2. This input is a comma separated list of floating point values for different Simulated
	skin resistance
3. Once the serial monitor gets input, it parses the input to get the float values and updates the class variables.
	1. The state of the class changes to  WAITING_FOR_USER_APPROVAL
4. The class then prompts the user to Approve writing these values to the OTP of the SI-7013 chip
	1. This "request for approval" is echoed on the screen periodically as a part of the emotibit.update() routine.
5. Once the user approves writing to the OTP:
	1. the class changes the state to WRITING_TO_OTP
	2. Now, the class member function tries to write the values to the OTP as a part of the ISR
6. Once the Values have been written to teh OTP, the class moves to the last state FINISHED.
7. The values stored in the OTP can then be read at any time to calculate the correciton values.
8. class will have a data member which sets on every emotibit setup, which tracks if the emotibit has data
	written on the OTP
***************
NORMAL

1. The EDA Correction calss will be initialized in setup nad begin running in NORMAL mode. 
2. After the I2C is initialized, read the OTP data snd calculate the correction values.
3. update the emotibit class variables accordingly.
5. proceed with normal execution.

*/
class EdaCorrection
{
private:
	bool _updateMode = false; // set when entered testing mode during production 
	bool _approvedToWriteOtp = false; // indicated user's approval to write to the OTP
	bool _isDataOnOtp; // bool to keep track if data is written to the OTP

public:
	float edaReadings[5];
	union Data {
	float edaReading; // 0, 10K, 100K, 1M, 10M
	char buff[4];// buffer to store the float in BYTE form
	};
	const uint8_t SI_7013_OTP_ADDRESS_FLOAT_0; // 0x82  
	const uint8_t SI_7013_OTP_ADDRESS_FLOAT_1; // 0x86
	const uint8_t SI_7013_OTP_ADDRESS_FLOAT_2; // 0x8A
	const uint8_t SI_7013_OTP_ADDRESS_FLOAT_3; // 0x8E
	const uint8_t SI_7013_OTP_ADDRESS_FLOAT_4; // 0x92
	uint16_t vref1Corrected;  // Corrected vrefValue
	uint32_t RskinFeedback;  // Corrected Rfeedback value

	enum class Status
	{
		SUCCESS,
		FAILURE
	};

	enum class Mode
	{
		NORMAL,
		UPDATE
	};
private:
	Mode _mode = EdaCorrection::Mode::NORMAL;

public:
	// enum to asynchronously track the progress.
	// the progress variable will be tracked in emotibit.update and the ISR to perform various functions sequentially in a non-blocking manner.
	enum class Progress
	{
		WAITING_FOR_SERIAL_DATA,
		WAITING_USER_APPROVAL,
		WRITING_TO_OTP,
		FINISH
	}progress;


public:
	/*
	usage: called in emotibit.setup(). Once called, it enables the emotibit to keep sensing the Serial on ever "loop" 
	changes progress from NOT_BEGUN to WAITING_FOR_SERIAL_DATA
	*/
	EdaCorrection::Status enterUpdateMode();


	/*
	usage: returns if the program is in update mode.
	
	note:in update mode: EmotiBit periodically checks for Serial buffers and if data is found, it updates SI-7013 OTP
	*/
	EdaCorrection::Mode getClassMode();

	
	/*
	usage: called in emotibit.update
	on every call, checks if the Serial input buffers have any data
	Then calls calcCorrection to get the correction values. 
	calls setFloatValues to update class variables
	changes progress to UPDATED_CLASS_VARIABLES
	*/
	EdaCorrection::Status readFloatFromSerial(); // change the name to make it more "special purpose"
	
	
	/*
	usage: called from readSerialinput
	sets the float array of the class data member after parsing the serial
	*/
	EdaCorrection::Status setFloatValues();

	/*
	usage: once the float data has been received, use this function to echo it on the serial monitor to ask for user confirmation to write to the OTP
	changes progress to WAITING_USER_APPROVAL
	note: will be non-blocking
	*/
	void echoFloatOnScreen();


	/*
	usage: once the class has been updated with the correct values from the serial input, ask user permission to write to the otp
	checks the Serial on every emotibit.update() for Serial available. if user approved, then change progress to WRITING_TO_OTP
	*/
	bool getUserApproval();
	
	
	/*
	usage: set the approvalStatus
	*/
	void setApprovalStatus(); // see if this can be absorbed by the getUserApproval or make it private
	
	
	/*
	usage: used to determineapproval status
	*/
	bool getApprovalStatus();

	/*
	usage:
	called in the ISR. writes data to the OTP
	sets progress to FINISH
	*/
	EdaCorrection::Status writeToOtp();

	/*
	usage: read from OTP
	*/
	EdaCorrection::Status readFromOtp();


	/*
	usage: solve for EmotiBit variables based on the data stored in OTP
	*/
	EdaCorrection::Status calcCorrectionFromOtp();
	

};