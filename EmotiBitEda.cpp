/**************************************************************************/
/*!
    @file     EmotiBitEda.cpp
    @author   Sean Montgomery (EmotiBit)

    @mainpage Electrodermal activity (EDA) handler for EmotiBit

    @section intro_sec Introduction

    This is a library to handle electrodermal activity (EDA) on EmotiBit.

		EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    @section author Author

    Written by Sean Montgomery for EmotiBit.

    @section  HISTORY

    v1.0  - First release

    @section license License

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/


/**************************************************************************/
/*!
    @brief  Sets EmotiBit to read EDA data

    @param version of EmotiBit
    @param emotibit_i2c I2C bus

    @return true if successful, otherwise false
*/
/**************************************************************************/
bool EmotiBitEda::Setup(EmotiBitVersionController::EmotiBitVersion version, TwoWire* emotibit_i2c)
{
	// ToDo: add sampling rate inputs
	
	if (version == EmotiBitVersionController::EmotiBitVersion::V04A)
	{	
		ads.setDataRate(RATE_ADS1115_475SPS);	// set to 475Hz to allow for 300Hz oversampling
		ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
		return ads.begin(0x48, emotibit_i2c);
	}
	else
	{
		// ToDo: Handle other EmotiBit versions
		return true;
	}
	return false;
}

