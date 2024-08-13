/**************************************************************************/
/*!
    @file     EmotiBitLedController.cpp
    @author   Nitin Nair (EmotiBit)

    @mainpage LED handler for EmotiBit

    @section intro_sec Introduction

    This class handles LEDs on EmotiBit.

		EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    @section author Author

    Written by Nitin Nair for EmotiBit.

    @section  HISTORY

    v0.1  - First release

    @section license License

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#include "EmotiBitLedController.h"


bool EmotiBitLedController::begin(TwoWire* emotibitI2c, EmotiBitVersionController::EmotiBitVersion hwVersion)
{
    bool status;
    if((uint8_t) hwVersion > (uint8_t)EmotiBitVersionController::EmotiBitVersion::V05C)
    {
        // EmotiBit HW v6+
        status = ncp5623.begin(*emotibitI2c, NCP5623_C_DEFAULT_ADDR);
    }
    else
    {
        status = ncp5623.begin(*emotibitI2c, NCP5623_B_DEFAULT_ADDR);
    }
    
    // ToDo: version control this setup specific to EmotiBit HW with NCP
    // set NCP5623 settings
    // ToDo: expose NCP driver settings to function signature
    if (settingsNCP5623.driverCurrent > 0)
    {
        // ToDo: NCP driver update to avoid I2C communication in setCurrent
        ncp5623.setCurrent(settingsNCP5623.driverCurrent);
    }
    ncp5623.setLEDpwm((uint8_t)Led::RED, settingsNCP5623.pwmVal);
    ncp5623.setLEDpwm((uint8_t)Led::BLUE, settingsNCP5623.pwmVal);
    ncp5623.setLEDpwm((uint8_t)Led::YELLOW, settingsNCP5623.pwmVal);
    setState(Led::RED, false);
    setState(Led::BLUE, false);
    setState(Led::YELLOW, false);
    update();
    // ToDo: expose other elements of the NCP settings to the LedController
    return status;
}

bool EmotiBitLedController::setState(Led led, bool state, bool updateNow)
{
    _ledStatus[(int)led].state = state;
    _ledStatus[(int)led].isChanged = true;
    if(updateNow)
    {
        return update();
    }
    return true;
}

bool EmotiBitLedController::getState(Led led)
{
    return _ledStatus[(int)led].state;
}

bool EmotiBitLedController::_updateNcp()
{
    // update the internal class of the driver
    for (uint8_t i = 0;i<Led::length;i++)
    {
        if(_ledStatus[i].isChanged)
        {
            // ToDo: implement bounds check when referencing array
            ncp5623.setLED(ledToNcpMap[i], _ledStatus[i].state);
        }
    }
    
    // Communicate with the driver and write the changes to chip 
    ncp5623.send();
    // ToDo: change the function signature of NCP5623::send() to return i2c error so that it can be propagated upstream.
    return true;
}

bool EmotiBitLedController::update()
{
    // this will become version controlled if we change the LED driver on EmotiBit
    return _updateNcp();
}