#include "ADC_Correction.h"
// Create a global obect to store data in the flash
FlashStorage(samdFlashStorage, SamdStorageAdcValues);

//#define SPECIAL_PROVISION_MODE
void setup()
{
	analogReadResolution(12);
	SamdStorageAdcValues samdStorageAdcValues;
#ifdef SPECIAL_PROVISION_MODE
	AdcCorrection adcCorrection(AdcCorrection::AdcCorrectionRigVersion::VER_0, AdcCorrection::DataFormatVersion::DATA_FORMAT_0);
	/*
	Serial.print("N[0]:"); Serial.println(adcCorrection.adcCorrectionRig.N[0]);
	Serial.print("N[1]:"); Serial.println(adcCorrection.adcCorrectionRig.N[1]);
	Serial.print("N[2]:"); Serial.println(adcCorrection.adcCorrectionRig.N[2]);
	*/
	adcCorrection.begin();
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("atwincDataArray");
	for (int i = 0; i < 12; i++)
	{
		Serial.print("  "); Serial.print(adcCorrection.atwincDataArray[i]);
	}
	// Serial.println("Saving data to the Atwinc flash");
#endif
	
	// adcCorrection.writeAtwincFlash();


#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("ToDo: Storing data on the SAMD flash");
#endif  ADC_CORRECTION_VERBOSE
	samdStorageAdcValues = samdFlashStorage.read();
	if (!samdStorageAdcValues.valid)
	{
#ifdef ADC_CORRECTION_VERBOSE
		Serial.println("Storing on the SAMD flash for the first time");
#endif
		samdStorageAdcValues._gainCorrection = adcCorrection.getGainCorrection();
		samdStorageAdcValues._offsetCorrection = adcCorrection.getOffsetCorrection();
		samdStorageAdcValues.valid = true;
		samdFlashStorage.write(samdStorageAdcValues);
	}
	else
	{
		Serial.println("data exists on the samd flash");
		Serial.println("reading from the samd flash");
		Serial.print("Gain correction:"); Serial.println(samdStorageAdcValues._gainCorrection);
		Serial.print("offset correction:"); Serial.println(samdStorageAdcValues._offsetCorrection);
	}
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Enabling the ADC to use the correction values");
#endif
	uint16_t adcBeforeCorrection[3], adcAfterCorrection[3];
	adcBeforeCorrection[0] = analogRead(A0);
	adcBeforeCorrection[1] = analogRead(A1);
	adcBeforeCorrection[2] = analogRead(A2);

#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("Comparing results before and after correction");
#endif
	analogReadCorrection(samdStorageAdcValues._offsetCorrection, samdStorageAdcValues._gainCorrection);
	adcAfterCorrection[0] = analogRead(A0);
	adcAfterCorrection[1] = analogRead(A1);
	adcAfterCorrection[2] = analogRead(A2);
	Serial.print("True Value"); Serial.print("\tBefore Correction"); Serial.println("\tAfter Correction");
	Serial.print(round((float)(1.f / 11.f) * 4096.f)); Serial.print("\t\t"); Serial.print(adcBeforeCorrection[0]); Serial.print("\t\t"); Serial.println(adcAfterCorrection[0]);
	Serial.print(round((float)(1.f / 2.f) * 4096.f)); Serial.print("\t\t"); Serial.print(adcBeforeCorrection[1]); Serial.print("\t\t"); Serial.println(adcAfterCorrection[1]);
	Serial.print(round((float)(10.f / 11.f) * 4096.f)); Serial.print("\t\t"); Serial.print(adcBeforeCorrection[2]); Serial.print("\t\t"); Serial.println(adcAfterCorrection[2]);
#ifdef ADC_CORRECTION_VERBOSE
	Serial.println("ToDo: Writing all the information to the Sd-Card");
#endif  ADC_CORRECTION_VERBOSE
	// ToDo:Add code to save the data on the Sd-Card
	
	uint8_t tempData[12], tempdata2[3];
	Serial.println("\ntesting AT-WINC flash read");
	adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_PRIMARY_DATA, 12, tempData);
	adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_DUPLICATE_DATA, 12, tempData);
	adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_METADATA_LOC, 3, tempData, 0);
	/*
	Serial.println("\nTesting write to the atWINC flash");
	adcCorrection.writeAtwincFlash();
	Serial.println("reading the flash after writing the dataArray");
	adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_PRIMARY_DATA, tempData);
	adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_DUPLICATE_DATA, tempData);
	*/
#else
	// here because you are in the normal mode
	samdStorageAdcValues = samdFlashStorage.read();
	if (!samdStorageAdcValues.valid)
	{
#ifdef ADC_CORRECTION_VERBOSE
			Serial.println("No correction found on SAMD. Calculating correction by reading ATWINC");
#endif
		AdcCorrection adcCorrection(AdcCorrection::AdcCorrectionRigVersion::UNKNOWN, AdcCorrection::DataFormatVersion::UNKNOWN);
		adcCorrection.calcCorrectionValues();
		// Store the values on the flash
		samdStorageAdcValues._gainCorrection = adcCorrection.getGainCorrection();
		samdStorageAdcValues._offsetCorrection = adcCorrection.getOffsetCorrection();
		samdStorageAdcValues.valid = true;
		samdFlashStorage.write(samdStorageAdcValues);
		// uint8_t adcCorrectionRigMetadata[3];
		/*
			// adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_METADATA_LOC, adcCorrectionRigMetadata, 0);
		// status = adcCorrection.atwincFlashIntegrityCheck();
		if (adcCorrection.atwincFlashIntegrityCheck() == AdcCorrection::Status::SUCCESS)
		{
			adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_PRIMARY_DATA, adcCorrection.atwincDataArray);
			adcCorrection.calcCorrectionValues();
			// Store the values on the flash
			samdStorageAdcValues._gainCorrection = adcCorrection.getGainCorrection();
			samdStorageAdcValues._offsetCorrection = adcCorrection.getOffsetCorrection();
			samdStorageAdcValues.valid = true;
			samdFlashStorage.write(samdStorageAdcValues);
		}
		else
		{
			Serial.print("Contact us @ info@emotibit.com");
		}
		*/
	}
	else
	{
		Serial.println("Correction data exists on the samd flash");
		analogReadCorrection(samdStorageAdcValues._offsetCorrection, samdStorageAdcValues._gainCorrection);
	}

#endif
	Serial.println("\nreached end of setup");
	while (1);
}

void loop()
{

}