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
    _hwVersion = hwVersion;
    
    #ifdef TEST_KTD2026
        _hwVersion = EmotiBitVersionController::EmotiBitVersion::V07A;
    #endif
    if((uint8_t) _hwVersion > (uint8_t)EmotiBitVersionController::EmotiBitVersion::V06A)
    {
        Serial.println("initializing KTD2026");
        ktd2026b = new KTD2026(KTD2026B_I2C_ADDRESS, emotibitI2c);
        status = ktd2026b->begin();
        ktd2026b->setEnabled();
        // todo: move the following to update()
        ktd2026b->setChannel1();
        delay(500);
        ktd2026b->setChannel2();
        delay(500);
        ktd2026b->setChannel3();
        delay(500);
        ktd2026b->resetChannel1();
        delay(500);
        ktd2026b->resetChannel2();
        delay(500);
        ktd2026b->resetChannel3();
        delay(500);
    }
    else
    {
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
    }
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

bool EmotiBitLedController::_updateKtd2026()
{
    // ToDo: do a better job
    for (uint8_t i = 0;i<Led::length;i++)
    {
        if(_ledStatus[i].isChanged)
        {
            if (i==0)
            {
                if( _ledStatus[i].state)
                    ktd2026b->setChannel1();
                else
                    ktd2026b->resetChannel1();
            }
            else if (i==1)
            {
                if( _ledStatus[i].state)
                    ktd2026b->setChannel3();
                else
                    ktd2026b->resetChannel3();
            }
            else if (i==2)
            {
                if( _ledStatus[i].state)
                    ktd2026b->setChannel2();
                else
                    ktd2026b->resetChannel2();
            }
            _ledStatus[i].isChanged = false;
        }
    }
    return true;
}

bool EmotiBitLedController::update()
{
    // this will become version controlled if we change the LED driver on EmotiBit
    if((uint8_t) _hwVersion > (uint8_t)EmotiBitVersionController::EmotiBitVersion::V06A)
    {
        return _updateKtd2026();
    }
    else
    {
        return _updateNcp();
    }
}