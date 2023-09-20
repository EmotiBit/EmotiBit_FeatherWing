#include "EmotiBitLedController.h"

bool EmotiBitLedController::init(EmotiBitVersionController::EmotiBitVersion hwVersion, TwoWire &_i2c, int driverCurrent)
{
     _hwVersion = hwVersion;
    if((int)_hwVersion == (int)EmotiBitVersionController::EmotiBitVersion::AGAVE_REVB)
    {
        // No IC to initialize. LEDs are controlled using GPIOs.
        // Set GPIOs and set type
        // ToDo: Maybe pin assignment information shoulde live in versionController
        _ledPin[Led::RECORDING] = 25;
        _ledPin[Led::WIFI] = 33;
        _ledPin[Led::BATTERY] = 26;
        pinMode(_ledPin[Led::RECORDING], OUTPUT);
        pinMode(_ledPin[Led::WIFI], OUTPUT);
        pinMode(_ledPin[Led::BATTERY], OUTPUT);
        return true;
    }
    else
    {
        // ToDo: check for other boards except EmotiBit
        Serial.print("Initializing NCP5623....");
        // ToDo: add a success or fail return statement for LED driver
        bool status = ncp.begin(_i2c);
        if (status)
        {
            // check if the LED current level is valid.
            if (driverCurrent > 0)
            {
                ncp.setCurrent(driverCurrent);
            }
            ncp.setLEDpwm(NCP5623::LED1, 8);
            ncp.setLEDpwm(NCP5623::LED2, 8);
            ncp.setLEDpwm(NCP5623::LED3, 8);
            ncp.setLED(NCP5623::LED1, false);
            ncp.setLED(NCP5623::LED2, false);
            ncp.setLED(NCP5623::LED3, false);
            ncp.send();
            Serial.println("Completed");
            return true;
        }
        return false;
    }

}

void EmotiBitLedController::setState(Led led, bool state)
{
    if((int)_hwVersion  == (int)EmotiBitVersionController::EmotiBitVersion::AGAVE_REVB)
    {
        // set state of LED
        if(_ledState[led] != state)
        {
            _ledState[led] = state;
            _ledChanged[led] = true;
        }
    }
    else
    {
        int ncpLed;
        if(led == Led::RECORDING)
        {
            ncpLed = 1;
        }
        else if (led == Led::WIFI)
        {
            ncpLed = 2;
        }
        else if (led == Led::BATTERY)
        {
            ncpLed = 3;
        }
        ncp.setLED(ncpLed, state);
    }
}

bool EmotiBitLedController::update()
{
    if((uint8_t)_hwVersion  == (uint8_t)EmotiBitVersionController::EmotiBitVersion::AGAVE_REVB)
    {
        for(uint8_t led = 0; led < LED_COUNT; led++)
        {
            if(_ledChanged[led])
            {
                digitalWrite(_ledPin[led], _ledState[led]);
            }
        }
    }
    else
    {
        ncp.send();
    }
    return true;
}

bool EmotiBitLedController::getState(Led led)
{
    return _ledState[led];
}