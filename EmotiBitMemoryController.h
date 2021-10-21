#ifndef EMOTIBIT_MEMORY_CONTROLLER_H
#define EMOTIBIT_MEMORY_CONTROLLER_H

#include "EmotiBitVersionController.h"
#include "EmotiBit_Si7013.h"
#include "SparkFun_External_EEPROM.h"

#define EMOTIBIT_EEPROM_I2C_ADDRESS 0x50
class EmotiBitMemoryController
{
public:
	EmotiBitVersionController::EmotiBitVersion _version;
	struct ConstEepromAddr
	{
		static const size_t NUM_MAP_SEGMENTS;
		static const size_t MEMORY_MAP_BASE;
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

	struct Si7013OtpmemoryMap
	{
		static const uint8_t BEGIN_ADDRESS;
		static const uint8_t SIZE;
	};

	struct EmotiBitEepromSettings
	{
		size_t capacityBytes;
		size_t pageSizeBytes;
	}emotibitEepromSettings;

	struct Buffer
	{
		uint8_t* data = nullptr;
		size_t dataLength = 0;
		uint8_t dataTypeVersion = 0;
		DataType datatype;

		void clear();
	}_buffer;

	
	
	enum class Error
	{
		SUCCESS = 0,
		FAILURE,
		MEMORY_NOT_UPDATED,
		OUT_OF_BOUNDS_ACCESS,
		I2C_WRITE_ERROR
	}writeResult;

	enum class MemoryControllerStatus
	{
		IDLE,
		BUSY
	}status;

	size_t _nextAvailableAddress = ConstEepromAddr::MEMORY_MAP_BASE + (sizeof(EepromMemoryMap)*(int)DataType::length);
	uint8_t _numMapSegments = 0;
	ExternalEEPROM emotibitEeprom;
	Si7013 si7013;
	bool init(TwoWire &emotiBit_i2c, EmotiBitVersionController::EmotiBitVersion version);

	uint8_t requestToWrite(DataType datatype, uint8_t version, size_t size = 0, uint8_t* data = nullptr, bool syncWrite = false);

	void updateBuffer(DataType datatype, uint8_t version, size_t size = 0, uint8_t* data = nullptr);

	void updateMemoryMap(DataType datatype, size_t size);

	uint8_t writeToEeprom();

	uint8_t loadMemoryMap();

	uint8_t readFromMemory(DataType datatype, uint8_t* data = nullptr);

};

#endif