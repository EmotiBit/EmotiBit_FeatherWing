#include "EmotiBitMemoryController.h"


bool EmotiBitMemoryController::init(TwoWire &emotibit_i2c, EmotiBitVersionController::EmotiBitVersion version)
{
	if (version == EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		if (emotibitEeprom.begin(EMOTIBIT_EEPROM_I2C_ADDRESS, emotibit_i2c))
		{
			emotibitEepromSettings.capacityBytes = 256; // in bytes
			emotibitEepromSettings.pageSizeBytes = 16; // in bytes
			emotibitEeprom.setMemorySize(emotibitEepromSettings.capacityBytes);
			emotibitEeprom.setPageSize(emotibitEepromSettings.pageSizeBytes);
			state = State::IDLE;
			return true;
		}
		else // Flash module failed to init. check i2c
		{
			return false;
		}
	}
	else if ((int)version < (int)EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		if (si7013.setup(emotibit_i2c))
		{
			state = State::IDLE;
			return true;
		}
		else // could not comm. with IC. check i2c line
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

void EmotiBitMemoryController::setHwVersion(EmotiBitVersionController::EmotiBitVersion hwVersion)
{
	_hwVersion = hwVersion;
}

uint8_t EmotiBitMemoryController::stageToWrite(DataType datatype, uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data, bool callWriteToStorage)
{
	if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		if (data != nullptr)
		{
			if (datatype != DataType::length)
			{
				updateBuffer(datatype, datatypeVersion, dataSize, data);
			}
			else
			{
				return (uint8_t)Status::OUT_OF_BOUNDS_ACCESS;
			}
		}
		else
		{
			return (uint8_t)Status::FAILURE;
		}

		if (callWriteToStorage)
		{
			// ToDo: plan to make it asynchronous
			// wait till data is written in the ISR
			writeToStorage();
			return (uint8_t)_writeBuffer.result;  // returns SUCCESS if write complete
		}
		else
		{
			state = State::BUSY_WRITING;
			while (state == State::BUSY_WRITING);
			return (uint8_t)_writeBuffer.result;
		}
	}
	else if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::UNKNOWN)
	{
		return (uint8_t)Status::HARDWARE_VERSION_UNKNOWN;
	}
}

void EmotiBitMemoryController::updateBuffer(DataType datatype, uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data)
{
	updateMemoryMap(datatype, dataSize + 1); // the version information requires an additoinal byte
	_writeBuffer.datatype = datatype;
	_writeBuffer.dataTypeVersion = datatypeVersion;
	_writeBuffer.dataSize = dataSize;
	// update in the end bacause ISR can hit anytime
	_writeBuffer.data = data;
}

void EmotiBitMemoryController::Buffer::clear()
{
	data = nullptr;
	dataSize = 0;
	dataTypeVersion = 0;
	datatype = DataType::length;
	result = Status::SUCCESS;
}

void EmotiBitMemoryController::updateMemoryMap(DataType datatype, uint32_t dataSize)
{
	map[(uint8_t)datatype].address = _nextAvailableAddress;
	map[(uint8_t)datatype].dataSize = dataSize;
	_nextAvailableAddress += dataSize;
}

uint8_t EmotiBitMemoryController::writeToStorage()
{
	if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		// detect if we have something to write
		if (_writeBuffer.data != nullptr && _writeBuffer.dataSize > 0)
		{
			// ToDo: store the return value from i2c writes, afteer driver is updated
			uint8_t i2cWriteStatus = 0;
			// write numMapEntries if not written already
			uint8_t numEmtriesInEeprom = emotibitEeprom.read(ConstEepromAddr::NUM_MAP_ENTRIES);
			if (_numMapEntries != numEmtriesInEeprom) // write only if not written or if writing more datatypes
			{
				emotibitEeprom.write(ConstEepromAddr::NUM_MAP_ENTRIES, _numMapEntries);
			}
			// write the updated Map
			size_t offsetMapAddress;
			offsetMapAddress = ConstEepromAddr::MEMORY_MAP_BASE + (int)_writeBuffer.datatype * sizeof(EepromMemoryMap);
			uint8_t* mapData;
			mapData = (uint8_t*)(map + ((int)_writeBuffer.datatype));
			emotibitEeprom.write(offsetMapAddress, mapData, sizeof(EepromMemoryMap));

			// write the buffer data
			emotibitEeprom.write(map[(int)_writeBuffer.datatype].address, _writeBuffer.data, _writeBuffer.dataSize);
			// Write the datatype version
			emotibitEeprom.write(map[(int)_writeBuffer.datatype].address+_writeBuffer.dataSize, _writeBuffer.dataTypeVersion);
			
			// clear the buffer after data has been written
			_writeBuffer.clear();
			state = State::IDLE;
			_writeBuffer.result = Status::SUCCESS;
			return 0;
		}
	}
	else if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::UNKNOWN)
	{
		return (uint8_t)Status::HARDWARE_VERSION_UNKNOWN;
	}
}


uint8_t EmotiBitMemoryController::stageToRead(DataType datatype, uint8_t &datatypeVersion, uint32_t &dataSize, uint8_t* &data, bool callReadFromStorage)
{
	if (state == State::IDLE)
	{
		if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			_readBuffer.datatype = datatype;
			if (callReadFromStorage)
			{
				state = State::BUSY_READING;
				readFromStorage();
				if (_readBuffer.result == Status::SUCCESS && state == State::READ_BUFFER_FULL)
				{
					data = _readBuffer.data;
					dataSize = _readBuffer.dataSize - 1;
					datatypeVersion = *(data + dataSize);
					state = State::IDLE;
					_readBuffer.clear();
					return 0;
				}
				else
				{
					return (uint8_t)_readBuffer.result;
				}
			}
			else
			{
				state = State::BUSY_READING;
				while (state != State::READ_BUFFER_FULL);
				data = _readBuffer.data;
				dataSize = _readBuffer.dataSize - 1;
				datatypeVersion = *(data + dataSize);
				state = State::IDLE;
				_readBuffer.clear();
				return 0;
			}
		}
		else if ((int)_hwVersion < (int)EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			_readBuffer.datatype = datatype;
			uint8_t readStatus;
			readStatus = readFromStorage();
			if (readStatus == 0)
			{
				data = _readBuffer.data;
				dataSize = _readBuffer.dataSize;
				_readBuffer.clear();
				return readStatus;
			}
		}
		else
		{
			return (uint8_t)Status::HARDWARE_VERSION_UNKNOWN;
		}
	}
	else
	{
		return (uint8_t)Status::FAILURE;
	}
}

uint8_t EmotiBitMemoryController::loadMemoryMap(DataType datatype)
{
	_numMapEntries = emotibitEeprom.read(ConstEepromAddr::NUM_MAP_ENTRIES);
	if (_numMapEntries == 255)
	{
		// EEPROM not written
		return (uint8_t)Status::MEMORY_NOT_UPDATED;
	}
	if ((uint8_t)datatype < _numMapEntries)
	{
		EepromMemoryMap *mapPtr;
		uint8_t *eepromMapData = new uint8_t[sizeof(EepromMemoryMap)];
		size_t offsetreadAddr = ConstEepromAddr::MEMORY_MAP_BASE + ((uint8_t)datatype * sizeof(EepromMemoryMap));
		emotibitEeprom.read(offsetreadAddr, eepromMapData, sizeof(EepromMemoryMap));
		mapPtr = (EepromMemoryMap*)eepromMapData;
		map[(uint8_t)datatype].address = mapPtr->address;
		map[(uint8_t)datatype].dataSize = mapPtr->dataSize;
		return 0;
	}
	else
	{
		// specific data type not updated
		return (uint8_t)Status::MEMORY_NOT_UPDATED;
	}
}

uint8_t EmotiBitMemoryController::readFromStorage()
{
	if (_readBuffer.datatype != DataType::length)
	{
		if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			// Load the correct memory-map from EEPROM
			uint8_t mapLoadStatus;
			mapLoadStatus = loadMemoryMap(_readBuffer.datatype);
			if (mapLoadStatus != 0)
			{
				_readBuffer.result = Status::MEMORY_NOT_UPDATED;
				return mapLoadStatus;
			}
			else
			{
				if (map[(uint8_t)_readBuffer.datatype].dataSize != 0 && map[(uint8_t)_readBuffer.datatype].dataSize != 255)
				{
					uint8_t *eepromData = new uint8_t[map[(uint8_t)_readBuffer.datatype].dataSize];
					emotibitEeprom.read(map[(uint8_t)_readBuffer.datatype].address, eepromData, map[(uint8_t)_readBuffer.datatype].dataSize);
					_readBuffer.data = eepromData;
					_readBuffer.dataSize = map[(uint8_t)_readBuffer.datatype].dataSize;

					// resetting dataType
					_readBuffer.datatype = DataType::length;
					_readBuffer.result = Status::SUCCESS;
					state = State::READ_BUFFER_FULL;
					return 0;
				}
				else
				{
					_readBuffer.result = Status::MEMORY_NOT_UPDATED;
					return (uint8_t)Status::MEMORY_NOT_UPDATED;
				}
			}
		}
		else if ((int)_hwVersion < (int)EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			if (_readBuffer.datatype == DataType::VARIANT_INFO)
			{
				uint8_t* otpData = new uint8_t;
				*otpData = si7013.readRegister8(Si7013OtpMemoryMap::EMOTIBIT_VERSION_ADDR, true);
				_readBuffer.data = otpData;
				_readBuffer.dataSize = 1;
				_readBuffer.result = Status::SUCCESS;
				return 0; // success
			}
			else if (_readBuffer.datatype == DataType::EDA)
			{
				uint8_t* otpData = new uint8_t[Si7013OtpMemoryMap::EDL_DATA_SIZE + Si7013OtpMemoryMap::EDR_DATA_SIZE];
				uint8_t *index;
				index = otpData;
				for (uint8_t addr = Si7013OtpMemoryMap::EDL_DATA_START_ADDR; addr < Si7013OtpMemoryMap::EDL_DATA_START_ADDR + Si7013OtpMemoryMap::EDL_DATA_SIZE; addr++)
				{
					*index = si7013.readRegister8(addr, true);
#ifdef DEBUG_SERIAL
					Serial.print("Addr: 0x"); Serial.print(addr, HEX); Serial.print("\t Value:"); Serial.println(*index);
#endif
					index++;
				}
				for (uint8_t addr = Si7013OtpMemoryMap::EDR_DATA_START_ADDR; addr < Si7013OtpMemoryMap::EDR_DATA_START_ADDR + Si7013OtpMemoryMap::EDR_DATA_SIZE; addr++)
				{
					*index = si7013.readRegister8(addr, true);
#ifdef DEBUG_SERIAL
					Serial.print("Addr: 0x"); Serial.print(addr, HEX); Serial.print("\t Value:"); Serial.println(*index);
#endif
					index++;
				}
				_readBuffer.data = otpData;
				_readBuffer.dataSize = Si7013OtpMemoryMap::EDL_DATA_SIZE + Si7013OtpMemoryMap::EDR_DATA_SIZE;
				_readBuffer.result = Status::SUCCESS;
				return 0; // success
			}
			else
			{
				return (uint8_t)Status::FAILURE;
			}
		}
		else if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::UNKNOWN)
		{
			return (uint8_t)Status::HARDWARE_VERSION_UNKNOWN;
		}
	}
}