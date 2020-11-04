/*
change the name of the highPrecisionRig
Add a function to write the rig metadata
*/

/*
Workflow:
Bootup
	[TESTER] Entering the ADC correction provision mode(pre-shipping)
		(the highPrecisionRig is plugged in with the emotibit)
		1. Read the values on the analog pins
		2. At this point, we have:
			1. N-array: stored in the code
			2. D-array: stored in the code
			3. ADCvalues: read in the previous step
		3. The whole HighPrecisionRig struct is populated
		4. call the updateAtwincDataArray function
			combines all the individual arrays to make one single uint8 array to write into the flash
				N[0]-D[0]-AdcHigh[0]-AdcLow[0]-N[1]-D[1]-AdcHigh[1]-AdcLow[1]-N[2]-D[2]-AdcHigh[2]-AdcLow[2]
		5. write to the flash at primary and secondary locations
		6. write rig metadata to the ATWINC flash
			1. rig version [1 BYTE]
			2. data format version of the ADC point(s) stored on flash[1 BYTE]
			3. numAdcPoints[1 BYTE]
		7. (calcCorrectionValues) Calculate the correction values based on the values stored in the highPrecisionRig struct
		8. Store the correction values in the SAMD flash
		9. Write raw ADC values and correction values on the SD-Card
			1. For personal record. This data will be added to the feather correction data database
		10. leave special provision mode to continue normal execution

	[USER] Follow normal execution
		1. Check SAMD flash for correction values (if vaild( in the flash storage struct))
			1. [Values FOUND]continue normal execution if correction values found
				1. analogReadCorrection(AdcCorrection.offsetCorr, AdcCorrection.gainCorr);
			2. [Values NOT FOUND] if correction values not found
				1. read the ATWINC flash metadata(the numPoints)
				3. do a "primary-secondary" data integrety check
					1. [PASS INTEGRITY check] read the ATWINC flash primary for the data. add
						1. Populate the highPrecisionRig struct
						2. <Calculate store> goto point 7
							1. No need to store in the SD-Card
					2. [FAIL INTEGRITY check] Contant info@emotiBit
Changes:
1. drop locRigVer
*/

#include "Arduino.h"
#include <WiFI101.h>
#include "spi_flash/include/spi_flash.h"
#include <FlashStorage.h>
#include "SAMD_AnalogCorrection.h"

//#define ADC_CORRECTION_VERBOSE
#define ATWINC_FLASH_4M_SZ (512 * 1024UL)

typedef struct {
	bool valid;
	uint16_t _gainCorrection;
	uint16_t _offsetCorrection;
}SamdStorageAdcValues;

class AdcCorrection
{
private:
	uint16_t _gainCorr; //drop these. the global variable has the values needed
	uint16_t _offsetCorr; //drop these. the global variable has the values needed
	bool _isAtwincDownloadMode = 0;
	uint8_t  _atwincFlashSize;
	bool _isupdatedAtwincArray = false;
	bool _isupdatedAtwincMetadataArray = false;
public:
	
	const size_t ATWINC_MEM_LOC_PRIMARY_DATA = 448*1024;  // primary start address for storing the adc values
	const size_t ATWINC_MEM_LOC_DUPLICATE_DATA = 480 * 1024; // secondary start address for storing the adc values
	const size_t ATWINC_MEM_LOC_METADATA_LOC = (512 * 1024) - 3; // stores the metadata 
	const size_t ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE = (508 * 1024);

	enum class DataFormatVersion
	{
		DATA_FORMAT_0,
		DATA_FORMAT_1,
		UNKNOWN
	};

	static const uint8_t MAX_ADC_POINTS = 10;
	static const uint8_t RIG_METADATA_SIZE = 3; // the size in bytes for the rig metedata
	uint8_t BYTES_PER_ADC_DATA_POINT; // based on the dataformat, the number of bytes required for each adc datapoint
	uint8_t numAdcPoints;
	DataFormatVersion dataFormatVersion;
	uint8_t ATWINC_DATA_ARRAY_SIZE; // number of bytes in the data array stored in at-winc
	uint8_t rigMetadata[RIG_METADATA_SIZE] = {0}; // ToDo: think whether the metaData length should be hardcoded or declared a const
	uint8_t adcInputPins[MAX_ADC_POINTS] = { 0 };
	//ToDo: change the 4 below to use a const 
	uint8_t atwincDataArray[4 * MAX_ADC_POINTS];

	
	enum Status {
		SUCCESS,
		FAILURE
	};
	Status atwincAdcMetaDataCorruptionTest, atwincAdcDataCorruptionTest;
	enum class AdcCorrectionRigVersion
	{
		VER_0,
		VER_1,
		UNKNOWN // used if the feather is starting with no correctoin values on the SAMD flash
	};

	struct AdcCorrectionRig {
		uint8_t N[MAX_ADC_POINTS];
		uint8_t D[MAX_ADC_POINTS];
		uint8_t AdcHigh[MAX_ADC_POINTS];
		uint8_t AdcLow[MAX_ADC_POINTS];
		// uint16_t idealAdc[255];// can calculate using the N and D values
	}adcCorrectionRig = { {0}, {0}, {0}, {0} };

private:
	AdcCorrectionRigVersion _version;

public:
	/*
	@Constructor 
	@usage:
		Initializes the N's and D's in the class
	*/
	AdcCorrection(AdcCorrection::AdcCorrectionRigVersion version, AdcCorrection::DataFormatVersion dataFormatVer);

	
	/*
	@f: updateAtwincDataArray
	@usage:
		conbine the N,D and ADCvalue arrays into one (uint8_t)array which can be written into the flash
		the flash_write API accpets the pointer of type uint8_t
	*/
	AdcCorrection::Status updateAtwincDataArray();
	
	
	/*
	@f: writeAtwincFlash
	@description:
		write the data onto the flash
	@usage:
		Used only in lab before shipping to write to the AT-WINC flash.
		Used in the special "provision-mode" for adc correction
	
	*/
	AdcCorrection::Status writeAtwincFlash();
	
	
	/*
	@f: readAtwincFlash
	@description:
		Reads data on the Atwinc flash from a memory location for a given data length
	@input:
		1: mem loc
		2: array pointer
	@usage:
		Used if the samd-flash does not contain correction values on startup.
		Updates the HighPrecisionRig struct with the ADC values
	*/
	AdcCorrection::Status readAtwincFlash(size_t readMemLoc, uint16_t dataSize, uint8_t* data, uint8_t isReadData = 1);
	
	
	/*
	@f: isSamdFlashWritten
	@description:
		checks if the data structure on the flash has been written to.
		It is taken from the sample usage of the API to write to SAMD flash
	@returns: true if data is stored on the flash
	@usage:
		used on every boot-up to check if the samd flash has been written
	*/
	bool isSamdFlashWritten();
	
	
	/*
	@f: writeSamdFlash
	@description:
		Writes the correction constants in the SAMD flash
	@inputs:
		1. gain correction value
		2. offset correction value
	@ret:
		returns true if write successful
	@usage;
		used when the samd flash does not contain the correction values
		used in combination with readAtwincFlash and calcCorrectionValues 
	*/
	bool writeSamdFlash(uint16_t gainCorr, uint16_t offsetCorr);
	
	
	/*
	@f: readSamdFlash
	@description:
		reads data from the ADC correction structure written in flash on every bootup
	@return:
		1: gain Correction value
		2: offset correction value
	@usage:	
		used on every boot-up to read the correction values from the flash
	*/
	void readSamdFlash(uint16_t &gainCorr, uint16_t &offsetCorr);
	
	
	/*
	@f:calcCorrectionValues
	@description: 
		Calculates the correction values from the raw vaues stored highPrecisionRig struct
	@notes:
		1. calculating the correction values before shipping
		2. Reads the raw values from the highPrecisionRig struct and updates the gain and offset correction values
	*/
	bool calcCorrectionValues();
	
	
	/*
	@f:getGainCorrection
	@description:
		returns the ADC gain correction
	*/
	uint16_t getGainCorrection();
	
	
	/*
	@f: getOffsetCorrection
	@description:
		returns the ADC offset correcction
	*/
	uint16_t getOffsetCorrection();

	void setGainCorrection(uint16_t gainCorr);

	void setOffsetCorrection(uint16_t offsetCorr);

	void readAdcPins();

	int getAverageAnalogInput(uint8_t inputPin);

	void begin();

	AdcCorrectionRigVersion getRigVersion();

	void setRigVersion(AdcCorrection::AdcCorrectionRigVersion version);

	uint16_t int8Toint16(uint8_t highByte, uint8_t lowByte);
	
	// moved this to the constructor
	// AdcCorrection::Status updateAtwincMetadataArray();

	AdcCorrection::Status initWifiModule();

	AdcCorrection::Status atwincFlashIntegrityCheck();
};

