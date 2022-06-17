#ifdef ADAFRUIT_FEATHER_M0
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
			1. N-array: stored in the code - N array stores the Numerator values in the resistor divider
			2. D-array: stored in the code - D array stores the Denominator values in the resistor divider
			Example:
			Resistor divider 1K/10K, N=1, D=11(1+10 in mathematical representation)
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
		9. leave special provision mode to continue normal execution

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
					2. [FAIL INTEGRITY check] Use ADC without correction

*/
// below table taken from WiFi101/src/spi_flash/include/spi_flash.h in WiFi101 Library by Arduino
/*
 * Detailed Sizes and locations for Flash Memory:
 *  ____________________ ___________ ____________________________________________________________________________
 * | Starting Address	|	Size	|	Location's Name			|	Description						   			|
 * |____________________|___________|___________________________|_______________________________________________|
 * |	  0 K  			|	  4	K	| 	Boot Firmware			|	Firmware to select which version to run		|
 * |	  4	K 			|	  8 K	|	Control Section			|	Structured data used by Boot firmware		|
 * |	 12 K			|     4	K	|	PLL+GAIN :				|	LookUp Table for PLL and Gain calculations	|
 * |	  				|     		|	PLL  Size = 1K			|		PLL				 						|
 * |	  				|     		|	GAIN Size = 3K			|		Gain configuration				 		|
 * |	 16	K			|	  4	K	|	CERTIFICATE				|	X.509 Certificate storage					|
 * |	 20	K			|	  8	K	|	TLS Server				|	TLS Server Private Key and certificates		|
 * |	 28	K			|	  8	K	|	HTTP Files				|	Files used with Provisioning Mode			|
 * |	 36	K			|	  4	K	|	Connection Parameters	|	Parameters for success connection to AP		|
 * |	 40	K			|	236 K 	|	Main Firmware/program	|	Main Firmware to run WiFi Chip				|
 * |	276	K			|	236 K	|	OTA Firmware		    |	OTA firmware								|
 * |    512 K                                                       Total flash size							|
 * |____________________|___________|___________________________|_______________________________________________|
 *
 */
// for mode details refer: http://ww1.microchip.com/downloads/en/devicedoc/atmel-42420-winc1500-software-design-guide_userguide.pdf section13.4
#include "Arduino.h"
#include <WiFi101.h>
#include "spi_flash/include/spi_flash.h"
#include "SAMD_AnalogCorrection.h"
#include "wiring_private.h"

// use the macro below to print different log messages on the serial when debugging
//#define ADC_CORRECTION_VERBOSE
#define ATWINC_FLASH_4M_SZ (512 * 1024UL)


class AdcCorrection
{
private:
	uint16_t _gainCorr; 
	uint16_t _offsetCorr; 
	float _measuredAdcInIsr = 0;
	//int8_t _isrOffsetCorr = 0;
	bool _isAtwincDownloadMode = 0;
	uint8_t  _atwincFlashSize;
	bool _isupdatedAtwincArray = false;
	bool _isupdatedAtwincMetadataArray = false;
	union IsrOffsetCorr {
		float inFloat;
		char inBytes[4];
	}_isrOffsetCorr;
public:
	
	const size_t ATWINC_MEM_LOC_DUPLICATE_DATA = 448*1024;  // primary start address for storing the adc values
	const size_t ATWINC_MEM_LOC_PRIMARY_DATA = 480 * 1024; // secondary start address for storing the adc values
	const size_t ATWINC_MEM_LOC_METADATA = (512 * 1024) - 3; // stores the metadata 
	const size_t ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE = (508 * 1024);
	const size_t METADATA_LOC_NUM_ADC = 0;
	const size_t METADATA_LOC_RIG_VERSION = 1;
	const size_t METADATA_LOC_DATA_FORMAT = 2;
    //ToDo: add METADATA_LOC_RIG_BOARD_ID

	struct AdcCorrectionValues {
		bool valid = false;
		uint16_t _gainCorrection;
		uint16_t _offsetCorrection;
		float _isrOffsetCorr = 0;
	};

	enum class DataFormatVersion
	{
		DATA_FORMAT_0 = 0,
		DATA_FORMAT_1 = 1,
		COUNT,
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
	AdcCorrection();

	/*
	@Constructor
	@usage: Called from setup when the Correction values are not stored in the SAMD flash
	*/
	AdcCorrection(AdcCorrection::AdcCorrectionRigVersion version, uint16_t &gainCorr, uint16_t &offsetCorr, bool &valid, float &isrOffsetCorr);

	/*
	@usage: This function calls various other class functions to 
	1. Acquire ADC data
	2. Store data on the At-Winc Flash
	3. Calculate Corretion data and pass it to the setup function in EmotiBit.cpp
	*/
	bool begin(uint16_t &gainCorr, uint16_t &offsetCorr, bool &valid);

	bool updateIsrOffsetCorr();

	/*
	@usage:
	Called to display ADC correction results on the Serial monitor after ADC has been corrected
	*/
	void echoResults(uint16_t gainCorr, uint16_t offsetCorr);
	
	/*
	@f: updateAtwincDataArray
	@usage:
		conbine the N,D and ADCvalue arrays into one (uint8_t)array which can be written into the flash
		the flash_write API accpets the pointer of type uint8_t
	*/
	AdcCorrection::Status updateAtwincDataArray();
	
	void parseAtwincDataArray();
	
	/*
	@f: writeAtwincFlash
	@description:
		write the data onto the flash
	@usage:
		Used only in lab before shipping to write to the AT-WINC flash.
		Used in the special "provision-mode" for adc correction
	@Notes: Based on the function spi_flash_write in WiFi101\src\spi_flash\source\sspi_flash.c in WiFi101 Library by Arduino 
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
	@Notes: Based on the function spi_flash_read in WiFi101\src\spi_flash\source\sspi_flash.c in WiFi101 Library by Arduino
	*/
	AdcCorrection::Status readAtwincFlash(size_t readMemLoc, uint16_t dataSize, uint8_t* data);
	
	
	
	/*
	@f:calcCorrectionValues
	@description: 
		Calculates the correction values from the raw vaues stored highPrecisionRig struct
	@notes:
		1. calculating the correction values before shipping
		2. Reads the raw values from the highPrecisionRig struct and updates the gain and offset correction values
	*/
	void calcCorrectionValues();
	
	
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
	/*
	@f: setGainCorrection
	@description:
		sets the ADC gain correcction to the class variable
	*/
	void setGainCorrection(uint16_t gainCorr);

	/*
	@f: seOffsetCorrection
	@description:
		sets the ADC offset correcction to the class variable
	*/
	void setOffsetCorrection(uint16_t offsetCorr);

	/*
	@uasge: Read the avrage analog input on the ADC pins and stores the corresting uint16 ADC values in the class variables
	*/
	void readAdcPins();

	/*
	Averages the input on the analog pins to get a smooth voltage value
	*/
	int getAverageAnalogInput(uint8_t inputPin);

	/* 
	returns the rig version
	*/
	AdcCorrectionRigVersion getRigVersion();

	void setRigVersion(AdcCorrection::AdcCorrectionRigVersion version);

	/*
	combines 2 uint8 integers to corresponding uint16 integer
	*/
	uint16_t int8Toint16(uint8_t highByte, uint8_t lowByte);

	/*
	Sets the WifiModule in download mode. important to set it in download mode to access the SPI flash
	*/
	AdcCorrection::Status initWifiModule();

	bool isAtWincMetadataUpdated();

	/*
	Cheack the data and metadate location in the SPI flash to make sure data is not corrupt
	*/
	AdcCorrection::Status atwincFlashIntegrityCheck();

	/*
	outputs the SAMD chip ID.
	*/
	void printChipId();

	/*
	* returns an int after reading characters from Serial input. Used to  convert the Rig version input by the user.
	*/
	int serialToInt();
};

#endif