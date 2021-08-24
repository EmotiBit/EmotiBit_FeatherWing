/**************************************************************************/
/*!
    @file     EmotiBitEda.h

    This is a library to handle electrodermal activity (EDA) on EmotiBit.

    EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    Written by Sean Montgomery for EmotiBit.

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#ifndef _EMOTIBIT_EDA_H_
#define _EMOTIBIT_EDA_H_


#include EmotiBitVersionController.h
#include "Adafruit_ADS1X15.h"


class EmotiBitEda
{
private:
	EmotiBitVersionController::EmotiBitVersion _emotibitVersion;	
	
public:
	
	Adafruit_ADS1115 ads;
	bool setup(EmotiBitVersionController::EmotiBitVersion version, TwoWire* emotibit_i2c = nullptr);
};

#endif