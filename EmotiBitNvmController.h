/**************************************************************************/
/*!
	@file     EmotiBitNvmController.h
	This is a library to handle Read/Write Operations to NVM on EmotiBit.
	EmotiBit invests time and resources providing this open source code,
	please support EmotiBit and open-source hardware by purchasing
	products from EmotiBit!
	Written by Nitin Nair for EmotiBit.
	BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#ifndef EMOTIBIT_MEMORY_CONTROLLER_H
#define EMOTIBIT_MEMORY_CONTROLLER_H

#include "EmotiBitVersionController.h"
#include "EmotiBit_Si7013.h"
#include "SparkFun_External_EEPROM.h"

#define EMOTIBIT_EEPROM_I2C_ADDRESS 0x50

//#define DEBUG_SERIAL

class EmotiBitNvmController
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
		uint32_t dataSize = 0;
	}map[(uint8_t)DataType::length];

	struct Si7013OtpMemoryMap
	{
		static const uint8_t EDL_DATA_START_ADDR = 0x82;
		static const uint8_t EDL_DATA_SIZE = 20; // 5 floats
		static const uint8_t EDR_DATA_START_ADDR = 0xB0;
		static const uint8_t EDR_DATA_SIZE = 4;  // 1 float
		static const uint8_t EMOTIBIT_VERSION_ADDR = 0xB7;
		static const uint8_t DATATYPE_VERSION_ADDR = 0xB6;
	};

	struct EmotiBitEepromSettings
	{
		size_t capacityBytes;
		size_t pageSizeBytes;
	}emotibitEepromSettings;

	enum class Status
	{
		SUCCESS = 0,
		HARDWARE_VERSION_UNKNOWN,
		MEMORY_NOT_UPDATED,
		OUT_OF_BOUNDS_ACCESS,
		I2C_WRITE_ERROR,
		CONTROLLER_BUSY,
		INVALID_DATA_TO_WRITE,
		OTHER_FAILURE
	}_writeResult, _readResult;

	struct Buffer
	{
		uint8_t* data = nullptr;
		uint32_t dataSize = 0;
		uint8_t datatypeVersion = 0;
		DataType datatype = DataType::length;

		/*!
			@brief Sets the datatype of the calling buffer.
		*/
		void setDatatype(DataType datatype);
		/*!
			@brief Updates the calling buffer with data read from or staged to be written to the NVM
		*/
		void update(uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data);
		/*!
			@brief Clears the calling buffer.
		*/
		void clear();
	}_writeBuffer, _readBuffer;

	enum class State
	{
		IDLE,
		READY_TO_WRITE,
		READY_TO_READ,
		READ_BUFFER_FULL
	}writeState, readState;

	uint32_t _nextAvailableAddress = ConstEepromAddr::MEMORY_MAP_BASE + (sizeof(EepromMemoryMap)*(int)DataType::length);
	uint8_t _numMapEntries = (uint8_t)DataType::length;
	ExternalEEPROM emotibitEeprom;
	Si7013 si7013;
	
	/*!
		@brief Initialize drivers to communicate with the onboard NVM modules.
		@return True if successful, otherwise false.
	*/
	bool init(TwoWire &emotiBit_i2c, EmotiBitVersionController::EmotiBitVersion version);

	/*!
		@brief Sets the HW version to control NVM access.
		This function Needs to be called to perform NVM access.
	*/
	void setHwVersion(EmotiBitVersionController::EmotiBitVersion version);

	/*
		@brief Updates the write buffer with data that will be written to the NVM.
		@param Type of data being written to NVM
		@param Version of datatype being written to NVM.
		@param Size of data being written in bytes.
		@param Pointer to data in the form of a byte array.
		@param set True to automatically write to NVM.
		@param Set True to validate the data written to the NVM.
		@return 0 if successful, non-zero if fail.
	*/
	uint8_t stageToWrite(DataType datatype, uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data, bool autoSync = false, bool enableValidateWrite = true);

	/*!
		@brief Function to validate data written to the NVM.
			   Performs a read immediately after write and compares data read and data written.
		@return 0 if successful, non-zero is fails
	*/
	uint8_t validateWrite();

	/*!
		@brief Records address and size for each datatype in the Memory Map.
		@param Type of data being written into the NVM.
		@param size of data being written into the NVM.
	*/
	void updateMemoryMap(DataType datatype, uint32_t dataSize);

	/*!
		@brief Performs the actual write operation and updates the NVM.
		@return 0 if successful, non-zero if fail.
	*/
	uint8_t writeToStorage();

	/*!
		@brief Call to read data from NVM of specific datatype.
		@param Type of datato be read from NVM.
		@param Version of datatype to be read from NVM.
		@param Size of data being read from NVM.
		@param Pointer to data in memory after read from NVM.
		@param set True to automatically call readFromStorage.
		@return 0 if successful, non-zero if fail.
	*/
	uint8_t stageToRead(DataType datatype, uint8_t &datatypeVersion, uint32_t &dataSize , uint8_t* &data, bool autoSync = false);

	/*!
		@brief Parses the Memory Map to identify Memory Loation to read specific dataype
		@param Type of data to be read from Memory
		@return 0 if successful, non-zero if fail.
	*/
	uint8_t loadMemoryMap(DataType datatype);

	/*!
		@brief Performs the actual read operation from NVM.
		@return 0 if successful, non-zero if fail.
	*/
	uint8_t readFromStorage();

	/*!
		@brief Calls read and write functions to sync Controller Buffers with NVM.
	*/
	void syncRW();
};

#endif