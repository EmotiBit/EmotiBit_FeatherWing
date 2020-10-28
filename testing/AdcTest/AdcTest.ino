#include "ADC_Correction.h"

void setup()
{
	analogReadResolution(12);
	AdcCorrection adcCorrection(AdcCorrection::AdcCorrectionRigVersion::VER_0);
	/*/
	Serial.print("N[0]:"); Serial.println(adcCorrection.adcCorrectionRig.N[0]);
	Serial.print("N[1]:"); Serial.println(adcCorrection.adcCorrectionRig.N[1]);
	Serial.print("N[2]:"); Serial.println(adcCorrection.adcCorrectionRig.N[2]);
	*/
	adcCorrection.correctAdc();
	Serial.println("Done with correction");
	Serial.print("gain correction:"); Serial.print(adcCorrection.getGainCorrection());
	Serial.print("\toffset correction:"); Serial.println(adcCorrection.getOffsetCorrection());
	Serial.println("atwincDataArray");
	for (int i = 0; i < 12; i++)
	{
		Serial.print("\t"); Serial.print(adcCorrection.atwincDataArray[i]);
	}
	/*
	// Test for converting 2 8 bits to one 16 bit

	Serial.print("\ntesting 8 bit to 16 bit:");
	Serial.print(adcCorrection.int8Toint16(170,85));
	*/
	Serial.println("\nreached end of setup");
	while (1);
}

void loop()
{

}