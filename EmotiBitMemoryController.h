#ifndef EMOTIBIT_MEMORY_CONTROLLER_H
#define EMOTIBIT_MEMORY_CONTROLLER_H

#include "EmotiBitVersionController.h"
#include "EmotiBit_Si7013.h"
#include "SparkFun_External_EEPROM.h"

#define EMOTIBIT_EEPROM_I2C_ADDRESS 0x50

//#define DEBUG_SERIAL

class EmotiBitMemoryController
{
public:
	EmotiBitVersionController::EmotiBitVersion _hwVersion = EmotiBitVersionController::EmotiBitVersion::UNKNOWN;
	struct ConstEepromAddr
	{
		static const size_t NUM_MAP_ENTRIES = 0;
		static const size_t MEMORY_MAP_BASE = 1;
	};
	
	enum class DataType
	{
		VARIANT_INFO = 0,
		EDA = 1,
		length
	};
	struct EepromMemoryMap
	{
		uint32_t address = 0;
		uint32_t size = 0;
	}map[(uint8_t)DataType::length];

	struct Si7013OtpMemoryMap
	{
		static const uint8_t EDL_DATA_START_ADDR = 0x82;
		static const uint8_t EDL_DATA_SIZE = 20; // 5 floats
		static const uint8_t EDR_DATA_START_ADDR = 0xB0;
		static const uint8_t EDR_DATA_SIZE = 4;  // 1 float
		static const uint8_t EMOTIBIT_VERSION_ADDR = 0xB7;
	};

	struct EmotiBitEepromSettings
	{
		size_t capacityBytes;
		size_t pageSizeBytes;
	}emotibitEepromSettings;

	enum class Status
	{
		SUCCESS = 0,
		FAILURE,
		MEMORY_NOT_UPDATED,
		OUT_OF_BOUNDS_ACCESS,
		I2C_WRITE_ERROR,
		HARDWARE_VERSION_UNKNOWN
	};

	struct Buffer
	{
		uint8_t* data = nullptr;
		size_t dataLength = 0;
		uint8_t dataTypeVersion = 0;
		DataType datatype = DataType::length;
		Status result = Status::SUCCESS;

		void clear();
	}_writeBuffer, _readBuffer;

	enum class State
	{
		IDLE,
		BUSY_READING,
		BUSY_WRITING,
		READ_BUFFER_FULL
	}state;

	size_t _nextAvailableAddress = ConstEepromAddr::MEMORY_MAP_BASE + (sizeof(EepromMemoryMap)*(int)DataType::length);
	uint8_t _numMapEntries = (uint8_t)DataType::length;
	ExternalEEPROM emotibitEeprom;
	Si7013 si7013;
	
	
	bool init(TwoWire &emotiBit_i2c, EmotiBitVersionController::EmotiBitVersion version);

	void setHwVersion(EmotiBitVersionController::EmotiBitVersion version);

	uint8_t stageToWrite(DataType datatype, uint8_t datatypeVersion, size_t dataSize = 0, uint8_t* data = nullptr, bool waitForWrite = true);

	void updateBuffer(DataType datatype, uint8_t datatypeVersion, size_t size = 0, uint8_t* data = nullptr);

	void updateMemoryMap(DataType datatype, size_t size);

	uint8_t writeToStorage();

	uint8_t requestToRead(DataType datatype, uint8_t &datatypeVersion, size_t &dataSize , uint8_t* &data, bool waitForRead = false);

	uint8_t loadMemoryMap(DataType datatype);

	uint8_t readFromStorage();
};

#endif