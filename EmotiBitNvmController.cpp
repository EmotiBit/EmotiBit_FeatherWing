/**************************************************************************/
/*!
	@file     EmotiBitNvmController.cpp
	@author   Nitin Nair (EmotiBit)

	@mainpage NVM access handler for EmotiBit

	@section intro_sec Introduction

	This is a library to handle NVM acces on EmotiBit.

		EmotiBit invests time and resources providing this open source code,
	please support EmotiBit and open-source hardware by purchasing
	products from EmotiBit!

	@section author Author

	Written by Nitin Nair for EmotiBit.

	@section  HISTORY

	v1.0  - First release

	@section license License

	BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/

#include "EmotiBitNvmController.h"

bool EmotiBitNvmController::init(TwoWire &emotibit_i2c)
{

	if (emotibitEeprom.begin(EMOTIBIT_EEPROM_I2C_ADDRESS, emotibit_i2c))
	{
		emotibitEepromSettings.capacityBytes = 256; // in bytes
		emotibitEepromSettings.pageSizeBytes = 16; // in bytes
		emotibitEeprom.setMemorySize(emotibitEepromSettings.capacityBytes);
		emotibitEeprom.setPageSize(emotibitEepromSettings.pageSizeBytes);
		writeState = State::IDLE;
		readState = State::IDLE;
		_nvmType = NvmType::EEPROM;
		return true;
	}

	if (si7013.setup(emotibit_i2c))
	{
		writeState = State::IDLE;
		readState = State::IDLE;
		_nvmType = NvmType::OTP;
		return true;
	}

	_nvmType = NvmType::UNKNOWN;
	return false;

}

uint8_t EmotiBitNvmController::stageToWrite(DataType datatype, uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data, bool autoSync, bool enableValidateWrite)
{
	if (_nvmType == NvmType::EEPROM)
	{
		if (writeState == State::IDLE)
		{
			if (data != nullptr && dataSize != 0)
			{
				if ((int)datatype < (int)DataType::length)
				{
					_writeBuffer.setDatatype(datatype);
					_writeBuffer.update(datatypeVersion, dataSize, data);
					if (datatype != DataType::ENTIRE_NVM)
					{
						updateMemoryMap(datatype, dataSize + 1); // the version information requires an additional byte
					}
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
	else if (_nvmType == NvmType::OTP)
	{
		Serial.println("Use EmotiBit FW V1.2.86 to perform Write to NVM on HW V2/V3");
		return (uint8_t)Status::NVM_TYPE_NOT_SUPPORTED;
	}
	else
	{
		Serial.println("");
		return (uint8_t)Status::NVM_TYPE_UNKNOWN;
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


void EmotiBitNvmController::NvmBuffer::setDatatype(DataType datatype)
{
	this->datatype = datatype;
}


void EmotiBitNvmController::NvmBuffer::update(uint8_t datatypeVersion, uint32_t dataSize, uint8_t* data)
{
	this->datatypeVersion = datatypeVersion;
	this->dataSize = dataSize;
	this->data = data;
}

void EmotiBitNvmController::NvmBuffer::clear()
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
		// ToDo: store the return value from i2c writes, after driver is updated
		uint8_t i2cWriteStatus = 0;
		if (_writeBuffer.datatype == DataType::ENTIRE_NVM)
		{
			emotibitEeprom.write(0, _writeBuffer.data, _writeBuffer.dataSize);
		}
		else
		{
			// write numMapEntries if not written already
			uint8_t numEmtriesInEeprom = emotibitEeprom.read(ConstEepromAddr::NUM_MAP_ENTRIES);
			if (_numMapEntries != numEmtriesInEeprom) // write only if not written or if writing more datatypes
			{
				emotibitEeprom.write(ConstEepromAddr::NUM_MAP_ENTRIES, _numMapEntries);
			}
			// write the updated Map
			size_t offsetMapAddress;
			offsetMapAddress = ConstEepromAddr::MEMORY_MAP_START + (int)_writeBuffer.datatype * sizeof(EepromMemoryMap);
			uint8_t* mapData;
			mapData = (uint8_t*)(map + ((int)_writeBuffer.datatype));
			emotibitEeprom.write(offsetMapAddress, mapData, sizeof(EepromMemoryMap));
			// write the buffer data
			emotibitEeprom.write(map[(int)_writeBuffer.datatype].address, _writeBuffer.data, _writeBuffer.dataSize);
			// Write the datatype version
			emotibitEeprom.write(map[(int)_writeBuffer.datatype].address+_writeBuffer.dataSize, _writeBuffer.datatypeVersion);
		}

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
	return ((uint8_t)State::BUSY_READING);
}


uint8_t EmotiBitNvmController::stageToRead(DataType datatype, uint8_t &datatypeVersion, uint32_t &dataSize, uint8_t* &data, bool autoSync)
{
	if (readState == State::IDLE)
	{
		_readBuffer.setDatatype(datatype);
		_readResult = Status::SUCCESS;
		if (autoSync)
		{
			readState = State::READY_TO_READ;
			readFromStorage();
		}
		else
		{
			readState = State::READY_TO_READ;
			while (readState != State::READ_BUFFER_FILLED && readState != State::IDLE);
		}
		if (_readResult == Status::SUCCESS && readState == State::READ_BUFFER_FILLED)
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
	uint8_t numMapEntries = emotibitEeprom.read(ConstEepromAddr::NUM_MAP_ENTRIES);
	if (numMapEntries == 255)
	{
		// EEPROM not written
		return (uint8_t)Status::MEMORY_NOT_UPDATED;
	}
	if ((uint8_t)datatype < numMapEntries)
	{
		EepromMemoryMap *mapPtr;
		uint8_t *eepromMapData = new uint8_t[sizeof(EepromMemoryMap)];
		size_t offsetreadAddr = ConstEepromAddr::MEMORY_MAP_START + ((uint8_t)datatype * sizeof(EepromMemoryMap));
		emotibitEeprom.read(offsetreadAddr, eepromMapData, sizeof(EepromMemoryMap));
		mapPtr = (EepromMemoryMap*)eepromMapData;
		map[(uint8_t)datatype].address = mapPtr->address;
		map[(uint8_t)datatype].dataSize = mapPtr->dataSize;
		mapPtr = nullptr;
		delete[] eepromMapData;
		eepromMapData = nullptr;
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
		if (_nvmType == NvmType::EEPROM)
		{
			if (_readBuffer.datatype == DataType::ENTIRE_NVM)
			{
				uint8_t *eepromData = new uint8_t[emotibitEepromSettings.capacityBytes];
				emotibitEeprom.read(0, eepromData, emotibitEepromSettings.capacityBytes-1); // driver limits read of max 255 bytes
				eepromData[emotibitEepromSettings.capacityBytes-1] = emotibitEeprom.read(emotibitEepromSettings.capacityBytes-1); // read the last location
				_readBuffer.data = eepromData;
				_readBuffer.dataSize = (uint32_t)emotibitEepromSettings.capacityBytes;
				_readResult = Status::SUCCESS;
				readState = State::READ_BUFFER_FILLED;
				return (uint8_t)Status::SUCCESS;
			}
			else
			{
				// Load the correct memory-map from EEPROM
				uint8_t mapLoadStatus;
				mapLoadStatus = loadMemoryMap(_readBuffer.datatype);

				if (mapLoadStatus != 0)
				{
					_readResult = Status::MEMORY_NOT_UPDATED;
					readState = State::IDLE;
					return (uint8_t)_readResult;
				}
				else
				{
					if (map[(uint8_t)_readBuffer.datatype].dataSize != 0 && map[(uint8_t)_readBuffer.datatype].dataSize != UINT32_MAX)  // MAX changes with datatype set in the MemoryMap
					{
						uint8_t *eepromData = new uint8_t[map[(uint8_t)_readBuffer.datatype].dataSize];
						uint8_t datatypeVersion;
						uint32_t dataSize;
						emotibitEeprom.read(map[(uint8_t)_readBuffer.datatype].address, eepromData, map[(uint8_t)_readBuffer.datatype].dataSize);
						dataSize = map[(uint8_t)_readBuffer.datatype].dataSize - 1; // 1 Additional Byte is required to store version.
						datatypeVersion = *(eepromData + dataSize);
						_readBuffer.update(datatypeVersion, dataSize, eepromData);
						_readResult = Status::SUCCESS;
						readState = State::READ_BUFFER_FILLED;
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
		}
		else if (_nvmType == NvmType::OTP)
		{
			if (_readBuffer.datatype == DataType::ENTIRE_NVM)
			{
				uint8_t* otpData = new uint8_t[Si7013OtpMemoryMap::OTP_SIZE_BYTES];
				uint8_t* index;
				index = otpData;
				for (uint8_t addr = Si7013OtpMemoryMap::EDL_DATA_START_ADDR,i=0; i<Si7013OtpMemoryMap::OTP_SIZE_BYTES; i++, addr++)
				{
					*index = si7013.readRegister8(addr, true);
#ifdef DEBUG_SERIAL
					Serial.print("Addr: 0x"); Serial.print(addr, HEX); Serial.print("\t Value:"); Serial.println(*index);
#endif
					index++;
				}
				_readBuffer.data = otpData;
				_readBuffer.dataSize = Si7013OtpMemoryMap::OTP_SIZE_BYTES;
				_readResult = Status::SUCCESS;
				readState = State::READ_BUFFER_FILLED;
				return (uint8_t)_readResult; // success
			}
			else
			{
				if (_readBuffer.datatype == DataType::VARIANT_INFO)
				{
					uint8_t* otpData = new uint8_t;
					uint8_t datatypeVersion;
					uint32_t dataSize;
					datatypeVersion = si7013.readRegister8(Si7013OtpMemoryMap::DATATYPE_VERSION_ADDR, true);
					if (datatypeVersion == 255)
					{
						_readResult = Status::MEMORY_NOT_UPDATED;
						readState = State::IDLE;
						return (uint8_t)_readResult;
					}
					else
					{
						*otpData = si7013.readRegister8(Si7013OtpMemoryMap::EMOTIBIT_VERSION_ADDR, true);
						dataSize = 1;
						_readBuffer.update(datatypeVersion, dataSize, otpData);
						_readResult = Status::SUCCESS;
						readState = State::READ_BUFFER_FILLED;
						return (uint8_t)_readResult; // success
					}
				}
				else if (_readBuffer.datatype == DataType::EDA)
				{
					uint8_t* otpData = new uint8_t[Si7013OtpMemoryMap::EDL_DATA_SIZE + Si7013OtpMemoryMap::EDR_DATA_SIZE];
					uint8_t datatypeVersion;
					uint32_t dataSize;
					datatypeVersion = si7013.readRegister8(Si7013OtpMemoryMap::DATATYPE_VERSION_ADDR, true);
					if (datatypeVersion == 255)
					{
						_readResult = Status::MEMORY_NOT_UPDATED;
						readState = State::IDLE;
						return (uint8_t)_readResult;
					}
					else
					{
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
						readState = State::READ_BUFFER_FILLED;
						return (uint8_t)_readResult; // success
					}
				}
				else
				{
					readState = State::IDLE;
					return (uint8_t)Status::OUT_OF_BOUNDS_ACCESS;
				}
			}
		}
		else if (_nvmType == NvmType::UNKNOWN)
		{
			_readResult = Status::NVM_TYPE_UNKNOWN;
			readState = State::IDLE;
			return (uint8_t)_readResult; // success
		}
	}
	return ((uint8_t)State::BUSY_READING);
}

void EmotiBitNvmController::syncRW()
{
	writeToStorage();

	readFromStorage();
}

void EmotiBitNvmController::printEntireNvm(bool autoSync)
{
	uint8_t* data = nullptr;
	uint8_t datatypeVersion = 0;
	uint32_t dataSize = 0;
	stageToRead(DataType::ENTIRE_NVM, datatypeVersion, dataSize, data, autoSync);
	Serial.print("NVM Content("); Serial.print("size: "); Serial.print(dataSize); Serial.println("): ");
	for (int i = 0; i <dataSize; i++)
	{
		Serial.print("|"); Serial.print(i); Serial.print(": "); Serial.print(data[i]);
		if ((i + 1) % 8 == 0)
		{
			Serial.println("|");
		}
	}
	Serial.println();
	delete[] data;
}

void EmotiBitNvmController::eraseEeprom(bool autoSync, bool printAfterErase)
{
	if (_nvmType == NvmType::EEPROM)
	{
		Serial.println("Erasing Entire EEPROM(defaults to 255 in each location)");
		const uint32_t NVM_SIZE = emotibitEepromSettings.capacityBytes;
		uint8_t emptyDataArray[NVM_SIZE];
		for (int i = 0; i < NVM_SIZE; i++)
			emptyDataArray[i] = 255;
		stageToWrite(DataType::ENTIRE_NVM, 0, emotibitEepromSettings.capacityBytes, emptyDataArray, autoSync, false);
		if (printAfterErase)
		{
			Serial.println("Printing After Erase");
			printEntireNvm(autoSync);
		}
	}
	else if (_nvmType == NvmType::OTP)
	{
		Serial.println("NVM erase not supported for EmotiBit V3 and below");
	}
	else if (_nvmType == NvmType::UNKNOWN)
	{
		Serial.println("NVM Type UNKNOWN. Please Init. NVM controller.");
	}
}