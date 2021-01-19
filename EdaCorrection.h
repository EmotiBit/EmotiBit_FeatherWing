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
6. Once the Values have been written to the OTP, the class moves to the last state FINISHED.
7. The values stored in the OTP can then be read at any time to calculate the correction values.
8. class will have a data member which sets on every emotibit setup, which tracks if the emotibit has data
	written on the OTP
***************
NORMAL

1. The EDA Correction calss will be initialized in setup and begin running in NORMAL mode. 
2. After the I2C is initialized, read the OTP data snd calculate the correction values.
3. update the emotibit class variables accordingly.
5. proceed with normal execution.

*/

// PLEASE SEE
// comment/uncomment the USE_ALT_SI7013 #define to run it in test mode/real mode
// Note: Test mode also WRITES TO THE OTP
// To not use the OTP, choose dummyMode from Serial while execution

#include "Arduino.h"
#include "Wire.h"
#include "EmotiBit_Si7013.h"

//#define USE_ALT_SI7013
#define ACCESS_MAIN_ADDRESS

class EdaCorrection
{
private:
	bool _approvedToWriteOtp = false;
	bool _responseRecorded = false;
	uint8_t FLOAT_PRECISION = 7;
	uint8_t _emotiBitVersion;
	bool powerCycled = true;
	bool successfulwrite = false;
public:
	class OtpMemoryMap_V0 {
	public:
		static const uint8_t NUM_EDL_CORRECTION_VALUES = 5;
		//static const uint8_t NUM_EDR_CORRECTION_VALUES = 1;
		size_t edlAddresses[NUM_EDL_CORRECTION_VALUES] = { 0x82, 0x86, 0x8A, 0x8E, 0x92 };
		size_t edrAddresses = 0xB0 ;
		size_t edlTestAddress = 0xA0;
		size_t emotiBitVersionAddr = 0xB7;
		size_t dataVersionAddr = 0xB6;
		bool emotiBitVersionWritten = false;
		bool dataVersionWritten = false;
		bool edrDataWritten[4] = {false};
		bool edlDataWritten[NUM_EDL_CORRECTION_VALUES][4] = { {false} };
		bool testDataWritten[4] = {false};
		struct WriteCount {
			uint8_t emotiBitVersion = 0;
			uint8_t dataVersion = 0;
			uint8_t edrData[4] = {0} ;
			uint8_t edlData[NUM_EDL_CORRECTION_VALUES][4] = { {0} };
		}writeCount;

	public:
		void echoWriteCount();
	}otpMemoryMap;
	bool isSensorConnected = true;
	//bool sensorConnectionTested = false;
	static const uint8_t NUM_EDL_READINGS = OtpMemoryMap_V0::NUM_EDL_CORRECTION_VALUES;
	static const size_t MAX_OTP_SIZE = 54;
	const uint8_t BYTES_PER_FLOAT = sizeof(float);

	struct CorrectionData {
		float edlReadings[NUM_EDL_READINGS] = { 0 };
		float edrReadings[NUM_EDL_READINGS] = { 0 };
		//char  otpBuffer[MAX_OTP_SIZE] = { 0 };
		float trueRskin[NUM_EDL_READINGS] = { 0,10000.0,100000.0,1000000.0,10000000.0 };
		float vRef1 = 0, vRef2= 0, Rfb= 0;
	}correctionData;

public:// flags
	bool isOtpValid = true;
	//bool displayedValidityStatus = false;
	bool readOtpValues = false; // flag to monitor if the class has read the OTP 
	bool calculationPerformed = false; // flag to monitor is calculation was performed from values read from OTP
	bool dummyWrite = false; // flag to check if in dummy mode or real OTP mode
	bool triedRegOverwrite = false; // flag to monitor if we are writing to previously written OTP location
	bool correctionDataReady = false;

public: 
#ifndef ACCESS_MAIN_ADDRESS
	const uint8_t SI_7013_OTP_ADDRESS_TEST = (uint8_t)0xA0;
#endif

	enum class Status
	{
		SUCCESS = 0,
		FAILURE = 1
	};

	enum class Mode
	{
		NORMAL,
		UPDATE
	};

	/*enum class EmotiBitVersion
	{
		EMOTIBIT_V02H = 0,
		NUM_VERSIONS
	}emotiBitVersion;*/

	enum class OtpDataFormat
	{
		DATA_FORMAT_0
	}otpDataFormat;

	union OtpData {
		float inFloat;
		char inByte[4];// buffer to store the float in BYTE form
	}otpData;

private:
	Mode _mode = EdaCorrection::Mode::NORMAL;

public:
	// enum to asynchronously track the progress.
	// the progress variable will be tracked in emotibit.update and the ISR to perform various functions sequentially in a non-blocking manner.
	enum class Progress
	{
		BEGIN,
		WAITING_FOR_SERIAL_DATA,
		WAITING_USER_APPROVAL,
		WRITING_TO_OTP,
		FINISH
	};

	Progress progress = EdaCorrection::Progress::BEGIN;

public:

	void begin(uint8_t emotiBitVersion);

	/*
	usage: called in emotibit.setup(). Once called, it enables the emotibit to keep sensing the Serial on ever "loop" 
	changes progress from NOT_BEGUN to WAITING_FOR_SERIAL_DATA
	*/
	EdaCorrection::Status enterUpdateMode(uint8_t emotiBitVersion, EdaCorrection::OtpDataFormat dataFormat);

	void normalModeOperations(float &vref1, float &vref2, float &Rfeedback);

	/*
	usage: returns if the program is in update mode.
	
	note:in update mode: EmotiBit periodically checks for Serial buffers and if data is found, it updates SI-7013 OTP
	*/
	EdaCorrection::Mode getMode();

	EdaCorrection::Status getFloatFromString();

	/*
	usage: called in emotibit.update
	on every call, checks if the Serial input buffers have any data
	*/
	EdaCorrection::Status monitorSerial(); // change the name to make it more "special purpose"
	
	
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
	void echoEdaReadingsOnScreen();


	/*
	usage: once the class has been updated with the correct values from the serial input, ask user permission to write to the otp
	checks the Serial on every emotibit.update() for Serial available. if user approved, then change progress to WRITING_TO_OTP
	*/
	bool getUserApproval();
	
	
	/*
	usage: set the approvalStatus
	*/
	void setApprovalStatus(bool response); // see if this can be absorbed by the getUserApproval or make it private
	
	
	/*
	usage: used to determineapproval status
	*/
	bool getApprovalStatus();

	bool checkSensorConnection(Si7013* si7013);

	EdaCorrection::Status writeEmotiBitVersionToOtp(Si7013 *si7013, int8_t version = -1);

	EdaCorrection::Status writeToOtp(Si7013* si7013, uint8_t addr, char val, uint8_t mask=0xFF);
	/*
	usage:
	called in the ISR. writes data to the OTP
	sets progress to FINISH
	*/
	EdaCorrection::Status writeToOtp(Si7013* si7013);


	//uint8_t readFromOtp(TwoWire* emotibit_i2c, uint8_t addr);
	/*
	usage: read from OTP
	*/
	EdaCorrection::Status readFromOtp(Si7013* si7013, bool isOtpOperation = true);


	/*
	usage: solve for EmotiBit variables based on the data stored in OTP
	*/
	EdaCorrection::Status calcEdaCorrection();
	
	bool isOtpRegWritten(Si7013* si7013, uint8_t addr, bool isOtpOperation = true);

	void displayCorrections();
};