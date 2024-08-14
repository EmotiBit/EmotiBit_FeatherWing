/**************************************************************************/
/*!
    @file     EmotiBitLedController.h

    This class handles LEDs on EmotiBit.

    EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    Written by Nitin Nair for EmotiBit.

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#ifndef EMOTIBIT_LED_CONTROLLER
#define EMOTIBIT_LED_CONTROLLER
#include <Arduino.h>
#include "EmotiBitVersionController.h"
#include "EmotiBit_NCP5623.h"

class EmotiBitLedController
{
public:
    /*!
     * @brief enumerator for LEDs on EmotiBit 
    */
    enum Led
    {
        RED = 0,
        BLUE, 
        YELLOW,
        length 
    };

    /*!
     * @brief struct to hold current state of LEDs on EmotiBit
    */
    struct LedStatus
    {
        bool state = false;
        bool isChanged = false;
    } _ledStatus[Led::length];
    
    /*!
     * @brief Map LedController to NCP led numbers.
              The NCP led number are taken from the driver implementation.
    */
    const uint8_t ledToNcpMap[Led::length] = 
                                    { 
                                        1,  // Red
                                        2,  // Blue
                                        3   // Yellow
                                    }; 

    struct SettingsNCP5623{
        uint8_t driverCurrent = 1;
        uint8_t pwmVal = 8;
    } settingsNCP5623;
    
    NCP5623 ncp5623;

public:

    /*!
     * @brief Setup the LedController
     * @param emotibitI2c i2c instance on EmotiBit
     * @param hwVersion EmotiBit Hardware Version
     * @return true if setup successful, else false
    */
   bool begin(TwoWire* emotibitI2c, EmotiBitVersionController::EmotiBitVersion hwVersion);

    /*!
     * @brief set the state of the LED
     * @param led the led to set
     * @param state State LED is set to
     * @param updateNow Specify if the physical state should be updated immediately. Warning: EmotiBit LED driver should only be communicated with in the ISR if running stock EmotiBit FW.
     * @return true if success, else false
    */
    bool setState(Led led, bool state, bool updateNow = false);

    /*!
     * @brief Function to get the State of the LED
     * @param led Get state of this led
     * @return state of the LED can be true (ON) or false (OFF)
    */
    bool getState(Led led);
    
    /*!
     * @brief Function to update the LEDs with the internal LED state. Call with caution as it may involve communicating with the driver.
              EmotiBit stock FW only allows I2C communication ONLY DURING ISR. Communicating outside ISR may cause collision on the I2C line.
     * @return true if successful, else false
    */
    bool update();

private:
    /*!
     * @brief Function to communicate with the NCP5623 driver
     * @return true if successful, else false
    */
    bool _updateNcp();
};
#endif