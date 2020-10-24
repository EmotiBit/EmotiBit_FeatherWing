class AdcCorrection
{
private:
	const uint16_t _ATWINC_MEM_LOC_FW_VER;
	const uint16_t _ATWINC_MEM_LOC_DATABASE_SZ;
	const uint16_t _ATWINC_MEM_START_LOC;
	uint16_t _gainCorr;
	uint16_t _offsetCorr;

public:
	enum status {
		SUCCESS,
		FAILURE
	};


public:
	/*
	@f: writeAtwincFlash
	@description:
		Needs to put the AT-WINC in download mode.
		read the flash size to perform inital checks
		write the data onto the flash
	
	@input:
		1: Mem loc to write
		2: Data to write
	*/
	bool writeAtwincFlash(uint16_t memLoc, char* data);
	/*
	@f: updateAtwincFlash
	@description:
		Reads data on teh Atwinc flash from a memory location for a given data length

	@input:
		1: mem loc
		2: data length to read in bytes
	*/
	void readAtwincFlash(uint16_t readLoc, char* data);
	/*
	@f: isSamdFlashWritten
	@description:
		checks if the data structure storing the constants exists in the samd flash

	@returns: true if data is stored on the flash
	*/
	bool isSamdFlashWritten();
	/*
	@f: writeSamdFlashAdcCorr
	@description:
		Writes the correction constants in the SAMD flash
	@inputs:
		1. gain correction value
		2. offset correction value
	@ret:
		returns true if write successful
	@notes:
		You should only write to samd when reading the raw values from the ATWINC flash and performing calculations on that.
		use cases:
		1. At production
		2. if samd is reprogrammed
	*/
	bool writeSamdFlashAdcCorr(uint16_t gainCorr, uint16_t offsetCorr);
	/*
	@f: readSamdFlashAdcCorr
	@description:
		reads data from the ADC correction structure written in flash on every bootup
	@inputs:
		1: gain Correction value
		2: offset correction value
	*/
	void readSamdFlashAdcCorr(uint16_t &gainCorr, uint16_t &offsetCorr);
	/*
	@f:calcCorrectionValues
	@description: 
		Calculates the correction values from the raw vaues stored in the ATWINC flash
	@input: 
		1: pointer to array of raw values
	@returns: 
		1: gain correction value
		2: offset correction value	
	@notes:
		this function should only be called if
		1. calculating the correction values at production
		2. SAMD has been flashed and needs to be updated with constants from ATWINC
	*/
	bool calcCorrectionValues(char* rawData, uint16_t &gainCorr, uint16_t &offsetCorr);
	/*
	@f:getGainCorrection
	@description:
		returns the ADC gain correction
	*/
	uint16_t getGainCorrection();
	/*
	@f: getOffsetCorrection
	@description:
		returns the offset correcction
	*/
	uint16_t getOffsetCorrection()

};

typedef struct {
	bool valid;
	uint16_t _gainCorrection;
	uint16_t _offsetCorrection;
}AdcCorrectionValues;
// Create a global obect to store data in the flash
FlashStorage(ADC_samdFlashDatabase, AdcCorrectionValues);