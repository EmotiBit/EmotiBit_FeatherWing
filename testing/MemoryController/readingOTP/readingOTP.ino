#include <Wire.h>
#include "wiring_private.h"
#include "EmotiBitNvmController.h"
#include "EmotiBitVersionController.h"

struct OtpData
{
	float edl0R;
	float edl10K;
	float edl100K;
	float edl1M;
	float edl10M;
	float edr;
};

void printData(OtpData* otpData);

void setup()
{
	Serial.begin(115200);
	while (!Serial.available())
	{
		Serial.println("Enter a key to begin");
		delay(500);
	}
	Serial.read();
	TwoWire emotibit_i2c(&sercom1, 11, 13);
	emotibit_i2c.begin();
	pinPeripheral(11, PIO_SERCOM);
	pinPeripheral(13, PIO_SERCOM);
	emotibit_i2c.setClock(100000);
	Serial.println("Test for Reading OTP");
	EmotiBitVersionController emotibitVersionController;
	EmotiBitVersionController::EmotiBitVersion version;
	version = emotibitVersionController.detectEmotiBitVersion(&emotibit_i2c, 0x50);
	Serial.print("EmotiBit version detected: ");
	Serial.println(EmotiBitVersionController::getHardwareVersion(version));
	Serial.println("EmotiBit powered up");
	EmotiBitNvmController emotiBitNvmController;
	emotiBitNvmController.init(emotibit_i2c, version);
	emotiBitNvmController.setHwVersion(version);

	// reading the EmotiBit version
	uint8_t* versionOnOtp = nullptr;
	uint32_t size;
	uint8_t dataFormatversion;
	emotiBitNvmController.stageToRead(EmotiBitNvmController::DataType::VARIANT_INFO, dataFormatversion, size, versionOnOtp,true);
	Serial.print("Version on OTP:"); Serial.println(*versionOnOtp);
	Serial.print("DataForamtVersion :"); Serial.println(dataFormatversion);

	delete versionOnOtp;
	
	// reading EDA data
	uint8_t* otpByteArray = nullptr;
	emotiBitNvmController.stageToRead(EmotiBitNvmController::DataType::EDA, dataFormatversion, size, otpByteArray, true);
	uint8_t *index;
	Serial.print("DataForamtVersion :"); Serial.println(dataFormatversion);
	Serial.print("Byte array: ");
	index = otpByteArray;
	for (int i = 0; i < EmotiBitNvmController::Si7013OtpMemoryMap::EDL_DATA_SIZE + EmotiBitNvmController::Si7013OtpMemoryMap::EDR_DATA_SIZE; i++)
	{
		Serial.print("\t");
		Serial.print(*index);
		index++;
	}

	OtpData* otpStruct;
	otpStruct = (OtpData*)otpByteArray;

	printData(otpStruct);
	
	// delete data after using
	delete[] otpByteArray;
	delete index;
	otpByteArray = nullptr;
	otpStruct = nullptr;
	index = nullptr;
	Serial.println("Reached end of code.");
	

	while (1);
}

void printData(OtpData* otpData)
{
	Serial.print("\nedl0R: "); Serial.println(otpData->edl0R, 7);
	Serial.print("edl10K: "); Serial.println(otpData->edl10K, 7);
	Serial.print("edl100K: "); Serial.println(otpData->edl100K, 7);
	Serial.print("edl1M: "); Serial.println(otpData->edl1M, 7);
	Serial.print("edl10M: "); Serial.println(otpData->edl10M, 7);
	Serial.print("edr: "); Serial.println(otpData->edr, 7);
}

void loop()
{

}