#include "EmotiBitNvmController.h"


bool EmotiBitNvmController::init(TwoWire &emotibit_i2c, EmotiBitVersionController::EmotiBitVersion version)
{
	if (version == EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		if (emotibitEeprom.begin(EMOTIBIT_EEPROM_I2C_ADDRESS, emotibit_i2c))
		{
			emotibitEepromSettings.capacityBytes = 256; // in bytes
			emotibitEepromSettings.pageSizeBytes = 16; // in bytes
			emotibitEeprom.setMemorySize(emotibitEepromSettings.capacityBytes);
			emotibitEeprom.setPageSize(emotibitEepromSettings.pageSizeBytes);
			writeState = State::IDLE;
			readState = State::IDLE;
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
			writeState = State::IDLE;
			readState = State::IDLE;
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

void EmotiBitNvmController::setHwVersion(EmotiBitVersionController::EmotiBitVersion hwVersion)
{
	_hwVersion = hwVersion;
}

uint8_t EmotiBitNvmController::stageToWrite(DataType datatype, uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data, bool autoSync, bool enableValidateWrite)
{
	if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::V04A)
	{
		if (writeState == State::IDLE)
		{
			if (data != nullptr && dataSize != 0)
			{
				if (datatype != DataType::length)
				{
					_writeBuffer.setDatatype(datatype);
					_writeBuffer.update(datatypeVersion, dataSize, data);
					updateMemoryMap(datatype, dataSize + 1); // the version information requires an additoinal byte
				}
				else
				{
					return (uint8_t)Status::OUT_OF_BOUNDS_ACCESS;
				}
			}
			else
			{
				return (uint8_t)Status::INVALID_DATA_TO_WRITE;
			}
			_writeResult = Status::SUCCESS;
			_validateWrite = enableValidateWrite;
			if (autoSync)
			{
				// ToDo: plan to make it asynchronous
				// wait till data is written in the ISR
				writeState = State::READY_TO_WRITE;
				writeToStorage();
			}
			else
			{
				writeState = State::READY_TO_WRITE;
				while (writeState == State::READY_TO_WRITE || writeState == State::BUSY_WRITING);
			}
			_writeBuffer.clear();
			_validateWrite = false;
			return (uint8_t) _writeResult;
		}
		else
		{
			return (uint8_t)Status::CONTROLLER_BUSY;
		}
	}
	else if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::UNKNOWN)
	{
		return (uint8_t)Status::HARDWARE_VERSION_UNKNOWN;
	}
}

uint8_t EmotiBitNvmController::validateWrite()
{
	uint8_t* data = nullptr;
	uint8_t datatypeVersion = 0;
	uint32_t dataSize = 0;
	uint8_t status;
	
	status = stageToRead(_writeBuffer.datatype, datatypeVersion, dataSize, data, true);
	if (status != 0)
	{
		return status;
	}

	if (datatypeVersion != _writeBuffer.datatypeVersion || dataSize != _writeBuffer.dataSize)
	{
		return (uint8_t)Status::I2C_WRITE_ERROR;
	}

	for (uint32_t iter = 0; iter < dataSize; iter++)
	{
		if (data[iter] != _writeBuffer.data[iter])
		{
			return (uint8_t)Status::I2C_WRITE_ERROR;
		}
	}
	delete[] data;
	return (uint8_t)Status::SUCCESS;
}


void EmotiBitNvmController::Buffer::setDatatype(DataType datatype)
{
	this->datatype = datatype;
}


void EmotiBitNvmController::Buffer::update(uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data)
{
	this->datatypeVersion = datatypeVersion;
	this->dataSize = dataSize;
	this->data = data;
}

void EmotiBitNvmController::Buffer::clear()
{
	this->data = nullptr;
	this->dataSize = 0;
	this->datatypeVersion = 0;
	this->datatype = DataType::length;
}

void EmotiBitNvmController::updateMemoryMap(DataType datatype, uint32_t dataSize)
{
	map[(uint8_t)datatype].address = _nextAvailableAddress;
	map[(uint8_t)datatype].dataSize = dataSize;
	_nextAvailableAddress += dataSize;
}

uint8_t EmotiBitNvmController::writeToStorage()
{
	if (writeState == State::READY_TO_WRITE)
	{
		writeState = State::BUSY_WRITING;
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
		emotibitEeprom.write(map[(int)_writeBuffer.datatype].address+_writeBuffer.dataSize, _writeBuffer.datatypeVersion);

		if (_validateWrite)
		{
			uint8_t status;
			status = validateWrite();
			if (status != 0)
			{
				writeState = State::IDLE;
				_writeResult = (Status)status;
				return (uint8_t)_writeResult;
			}
		}
		// clear the buffer after data has been written
		writeState = State::IDLE;
		_writeResult = Status::SUCCESS;
		return 0;
	}
}


uint8_t EmotiBitNvmController::stageToRead(DataType datatype, uint8_t &datatypeVersion, uint32_t &dataSize, uint8_t* &data, bool autoSync)
{
	if (readState == State::IDLE)
	{
		if (datatype == DataType::length)
		{
			return (uint8_t)Status::OUT_OF_BOUNDS_ACCESS;
		}
		else
		{
			_readBuffer.setDatatype(datatype);
		}
		_readResult = Status::SUCCESS;
		if (autoSync)
		{
			readState = State::READY_TO_READ;
			readFromStorage();
		}
		else
		{
			readState = State::READY_TO_READ;
			while (readState != State::READ_BUFFER_FULL && readState != State::IDLE);
		}
		if (_readResult == Status::SUCCESS && readState == State::READ_BUFFER_FULL)
		{
			data = _readBuffer.data;
			dataSize = _readBuffer.dataSize;
			datatypeVersion = _readBuffer.datatypeVersion;
			_readBuffer.clear();
			readState = State::IDLE;
			return (uint8_t)_readResult;
		}
		else
		{
			return (uint8_t)_readResult;
		}
	}
	else
	{
		return (uint8_t)Status::CONTROLLER_BUSY;
	}
}

uint8_t EmotiBitNvmController::loadMemoryMap(DataType datatype)
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

uint8_t EmotiBitNvmController::readFromStorage()
{
	if (readState == State::READY_TO_READ)
	{
		readState = State::BUSY_READING;
		if (_hwVersion == EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			// Load the correct memory-map from EEPROM
			uint8_t mapLoadStatus;
			mapLoadStatus = loadMemoryMap(_readBuffer.datatype);
			if (mapLoadStatus != 0)
			{
				_readResult = Status::MEMORY_NOT_UPDATED;
				return (uint8_t)_readResult;
			}
			else
			{
				if (map[(uint8_t)_readBuffer.datatype].dataSize != 0 && map[(uint8_t)_readBuffer.datatype].dataSize != 255)
				{
					uint8_t *eepromData = new uint8_t[map[(uint8_t)_readBuffer.datatype].dataSize];
					uint8_t datatypeVersion;
					uint32_t dataSize;
					emotibitEeprom.read(map[(uint8_t)_readBuffer.datatype].address, eepromData, map[(uint8_t)_readBuffer.datatype].dataSize);
					dataSize = map[(uint8_t)_readBuffer.datatype].dataSize - 1; // 1 Additional Byte is required to store version.
					datatypeVersion = *(eepromData + dataSize);
					_readBuffer.update(datatypeVersion, dataSize, eepromData);
					_readResult = Status::SUCCESS;
					readState = State::READ_BUFFER_FULL;
					return (uint8_t)_readResult;
				}
				else
				{
					readState = State::IDLE;
					_readResult = Status::MEMORY_NOT_UPDATED;
					return (uint8_t)_readResult;
				}
			}
		}
		else if ((int)_hwVersion < (int)EmotiBitVersionController::EmotiBitVersion::V04A)
		{
			if (_readBuffer.datatype == DataType::VARIANT_INFO)
			{
				uint8_t* otpData = new uint8_t;
				uint8_t datatypeVersion;
				uint32_t dataSize;
				*otpData = si7013.readRegister8(Si7013OtpMemoryMap::EMOTIBIT_VERSION_ADDR, true);
				datatypeVersion = si7013.readRegister8(Si7013OtpMemoryMap::DATATYPE_VERSION_ADDR, true);
				dataSize = 1;
				_readBuffer.update(datatypeVersion, dataSize, otpData);
				_readResult = Status::SUCCESS;
				readState = State::READ_BUFFER_FULL;
				return (uint8_t)_readResult; // success
			}
			else if (_readBuffer.datatype == DataType::EDA)
			{
				uint8_t* otpData = new uint8_t[Si7013OtpMemoryMap::EDL_DATA_SIZE + Si7013OtpMemoryMap::EDR_DATA_SIZE];
				uint8_t datatypeVersion;
				uint32_t dataSize;
				datatypeVersion = si7013.readRegister8(Si7013OtpMemoryMap::DATATYPE_VERSION_ADDR, true);
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
				dataSize = Si7013OtpMemoryMap::EDL_DATA_SIZE + Si7013OtpMemoryMap::EDR_DATA_SIZE;
				_readBuffer.update(datatypeVersion, dataSize, otpData);
				_readResult = Status::SUCCESS;
				readState = State::READ_BUFFER_FULL;
				return (uint8_t)_readResult; // success
			}
			else
			{
				readState = State::IDLE;
				return (uint8_t)Status::OUT_OF_BOUNDS_ACCESS;
			}
		}
	}
}

void EmotiBitNvmController::syncRW()
{
	writeToStorage();

	readFromStorage();
}