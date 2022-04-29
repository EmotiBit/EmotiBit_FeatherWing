/*!
	Example to perform all tests on the NvmController

*/

#include "EmotiBitNvmController.h"
#include "EmotiBitVersionController.h"
#include "Wire.h"
#include "wiring_private.h"

enum TEST_NUMBER
{
	ONE=0,
	TWO,
	THREE,
	FOUR,
	FIVE,
	SIX,
	SEVEN
};

const uint8_t NUM_TESTS = 7;
const char* RESULT_PASS = "PASS\0";
const char* RESULT_FAIL = "FAIL\0";
const char* TEST_1_NAME = "NVM Test 1- Access Memory Without setting HW version\0";
const char* TEST_2_NAME = "NVM Test 2- Writing to All Memory Locations\0";
const char* TEST_3_NAME = "NVM Test 3- Erasing NVM\0";
const char* TEST_4_NAME = "NVM Test 4- Writing VARIANT INFO\0";
const char* TEST_5_NAME = "NVM Test 5- Writing EDA DATA\0";
const char* TEST_6_NAME = "NVM Test 6- Reading VARIANT_INFO \0";
const char* TEST_7_NAME = "NVM Test 7- Reading EDA DATA\0";

const char* testName[NUM_TESTS] = { TEST_1_NAME, TEST_2_NAME, TEST_3_NAME, TEST_4_NAME, TEST_5_NAME, TEST_6_NAME, TEST_7_NAME };


void testDatatypeWrite(EmotiBitNvmController *emotibitNvmController, EmotiBitNvmController::DataType datatype, uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data);
bool compareSampleDataFromNvm(uint8_t* data);
void logTestResult(uint8_t testNumber, const char* testResult, uint8_t failException = 0);

struct SampleData
{
	static const uint8_t F_DATA_SIZE = 5;
	static const uint8_t I_DATA_SIZE = 10;
	float fData[F_DATA_SIZE] = { 1.23f,2.34f,3.45f,1.2345678f };
	uint8_t iData[I_DATA_SIZE] = { 0,1,2,3,4,5,6,7,8,9 };
	String sData = "SampleData";
}sampleData;


void setup()
{
	// Setting up Serial
	Serial.begin(115200);
	// Add this delay and comment the serial wait if using debugger.
	//delay(5000);
	while (!Serial.available())
	{
		Serial.println("enter a character to Start NVM Test");
		delay(1000);
	}
	Serial.read();
	// Setting up I2C
	// Constructor signature: TwoWire Wire(&PERIPH_WIRE, PIN_WIRE_SDA, PIN_WIRE_SCL);
	TwoWire emotibit_i2c(&sercom1, EmotiBitVersionController::EMOTIBIT_I2C_DAT_PIN, EmotiBitVersionController::EMOTIBIT_I2C_CLK_PIN);
	emotibit_i2c.begin();
	pinPeripheral(EmotiBitVersionController::EMOTIBIT_I2C_DAT_PIN, PIO_SERCOM);
	pinPeripheral(EmotiBitVersionController::EMOTIBIT_I2C_CLK_PIN, PIO_SERCOM);
	emotibit_i2c.setClock(100000);

	// Detecting EmotiBit HW version
	EmotiBitVersionController::EmotiBitVersion hwVersion;
	String sku;
	EmotiBitVersionController emotibitVersionController;
	if (!emotibitVersionController.isEmotiBitReady())
	{
		Serial.println("Check if battery and SD-Card are plugged in EmotiBit");
		while (1);
	}
	emotibitVersionController.detectVariantFromHardware(emotibit_i2c, hwVersion, sku);
	if (hwVersion == EmotiBitVersionController::EmotiBitVersion::UNKNOWN)
	{
		Serial.println("Version Detection failed. Halting test.");
		while (1);
	}

	//init NvmController
	EmotiBitNvmController emotibitNvmController;
	emotibitNvmController.init(emotibit_i2c);

	//creating variables to read
	uint8_t* data = nullptr;
	uint8_t datatypeVersion;
	uint32_t dataSize;

	uint8_t status;
	// deprecated
	//// ########################################################
	//// Testing running NvmController without setting hwVersion
	//status = emotibitNvmController.stageToRead(EmotiBitNvmController::DataType::VARIANT_INFO, datatypeVersion, dataSize, data, true);
	//if (status != 0)
	//{
	//	logTestResult((uint8_t)TEST_NUMBER::ONE, RESULT_PASS, status);
	//}
	//else
	//{
	//	logTestResult((uint8_t)TEST_NUMBER::ONE, RESULT_FAIL, status);
	//}

	// Setting HW version
	//emotibitNvmController.setHwVersion(hwVersion);

	// ########################################################
	// Testing writing to entire EEPROM
	if(hwVersion == EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		ExternalEEPROM eeprom;
		eeprom.begin(0x50, emotibit_i2c);
		eeprom.setMemorySize(256);
		eeprom.setPageSize(16);
		// Writing test Data in all Memory loations
		uint8_t testData = 0;
		for (int addr = 0; addr < eeprom.getMemorySize(); addr++)
		{
			eeprom.write(addr, testData);
			testData++;
			if (testData > 9)
				testData = 0;
		}

		testData = 0;
		for (int addr = 0; addr < eeprom.getMemorySize(); addr++)
		{
			if (testData != eeprom.read(addr))
			{
				logTestResult(TEST_NUMBER::TWO, RESULT_FAIL);
				Serial.println("Data incorrectly written");
				break;
			}
			testData++;
			if (testData > 9)
				testData = 0;
		}
		logTestResult(TEST_NUMBER::TWO, RESULT_PASS);
	}
	else
	{
		Serial.print("\n#######################################################\n");
		Serial.print(TEST_2_NAME); Serial.println(" ::: Test supported only by EmotiBit V04");
		Serial.print("\n#######################################################\n");
	}
	Serial.print("HW Version: "); Serial.println((uint8_t)hwVersion);
	Serial.print("Reading entire NVM");
	emotibitNvmController.printEntireNvm(true);

	// ########################################################
	// Testing Clearing all Memory
	if (hwVersion == EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		emotibitNvmController.eraseEeprom(true, true);
		emotibitNvmController.stageToRead(EmotiBitNvmController::DataType::ENTIRE_NVM, datatypeVersion, dataSize, data, true);
		for (uint16_t addr = 0; addr < emotibitNvmController.emotibitEepromSettings.capacityBytes; addr++)
		{
			if (data[addr] != 255)
			{
				logTestResult(TEST_NUMBER::THREE, RESULT_FAIL);
				Serial.println("Not all Locations are 255");
				break;
			}
		}
		delete[] data;
		data = nullptr;
		logTestResult(TEST_NUMBER::THREE, RESULT_PASS);
	}
	else
	{
		Serial.print("\n#######################################################\n");
		Serial.print(TEST_3_NAME); Serial.println(" :::  Test supported only by EmotiBit V04");
		Serial.print("\n#######################################################\n");
	}
	// ########################################################
	// Testing Writing Variant info
	SampleData* sData;
	sData = &sampleData;
	testDatatypeWrite(&emotibitNvmController, EmotiBitNvmController::DataType::VARIANT_INFO, 0, sizeof(SampleData), (uint8_t*)sData);
	
	// ########################################################
	// Testing Writing EDA info
	testDatatypeWrite(&emotibitNvmController, EmotiBitNvmController::DataType::EDA, 77, sizeof(SampleData), (uint8_t*)sData);

	// Printing whole NVM
	emotibitNvmController.printEntireNvm(true);

	// ########################################################
	// Testing reading VARIANT_INFO from NVM
	status = emotibitNvmController.stageToRead(EmotiBitNvmController::DataType::VARIANT_INFO, datatypeVersion, dataSize, data, true);
	if (status == 0)
	{
		logTestResult(TEST_NUMBER::SIX, RESULT_PASS, status);
	}
	else
	{
		logTestResult(TEST_NUMBER::SIX, RESULT_FAIL, status);
	}
	delete[] data;
	data = nullptr;
	// ########################################################
	// Testing reading VARIANT_INFO from NVM
	status = emotibitNvmController.stageToRead(EmotiBitNvmController::DataType::EDA, datatypeVersion, dataSize, data, true);
	if (status == 0)
	{
		logTestResult(TEST_NUMBER::SEVEN, RESULT_PASS, status);
	}
	else
	{
		logTestResult(TEST_NUMBER::SEVEN, RESULT_FAIL, status);
	}
	// Erasing All Data
	emotibitNvmController.eraseEeprom(true);

	delete[] data;
	data = nullptr;
	Serial.println("End of test");
	while (1);
}

void testDatatypeWrite(EmotiBitNvmController *emotibitNvmController, EmotiBitNvmController::DataType datatype, uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data)
{
	TEST_NUMBER testNumber;
	if (datatype == EmotiBitNvmController::DataType::VARIANT_INFO)
	{
		testNumber = TEST_NUMBER::FOUR;
	}
	else if (datatype == EmotiBitNvmController::DataType::EDA)
	{
		testNumber = TEST_NUMBER::FIVE;
	}
	uint8_t status;
	status = emotibitNvmController->stageToWrite(datatype, datatypeVersion, dataSize, data, true, false);
	if (status == 0)
	{
		uint8_t* nvmData = nullptr;
		datatypeVersion = 0;
		dataSize = 0;
		status = emotibitNvmController->stageToRead(datatype, datatypeVersion, dataSize, nvmData, true);
		if (status == 0)
		{
			if (compareSampleDataFromNvm(nvmData))
			{
				logTestResult(testNumber, RESULT_PASS);
			}
			else
			{
				logTestResult(testNumber, RESULT_FAIL, status);
			}
			delete[] nvmData;
			nvmData = nullptr;
		}
		else
		{
			logTestResult(testNumber, RESULT_FAIL, status);
		}
	}
	else
	{
		logTestResult(testNumber, RESULT_FAIL, status);
	}
	
}

bool compareSampleDataFromNvm(uint8_t* data)
{
	SampleData* nvmData;
	nvmData = (SampleData*)data;
	for (uint8_t i = 0; i < SampleData::F_DATA_SIZE; i++)
	{
		if (sampleData.fData[i] != nvmData->fData[i])
		{
			return false;
		}
	}

	for (uint8_t i = 0; i < SampleData::I_DATA_SIZE; i++)
	{
		if (sampleData.iData[i] != nvmData->iData[i])
		{
			return false;
		}
	}

	if (!sampleData.sData.equals(nvmData->sData))
	{
		return false;
	}

	return true;
}

void logTestResult(uint8_t testNumber, const char* testResult, uint8_t exception)
{
	Serial.println("\n#######################################################");
	Serial.print("["); Serial.print(testName[testNumber]); Serial.print("]: "); Serial.print(testResult); 
	if (exception != 0)
	{
		Serial.print(" - Exception thrown: "); Serial.print(exception);
	}
	Serial.println("\n#######################################################\n");
}

void loop()
{

}