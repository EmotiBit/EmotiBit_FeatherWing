#pragma once
#include <EmotiBit_NCP5623.h>
#include "EmotiBitVersionController.h"
#include "Arduino.h"
class EmotiBitLedController
{
public:
    enum Led{
        RECORDING = 0,
        WIFI = 1,
        BATTERY = 2,
    };
    const static uint8_t LED_COUNT = 3;  // ToDO: this needs to change as itmay depend on the board version
    bool _ledState[LED_COUNT];
    bool _ledChanged[LED_COUNT];
    uint8_t _ledPin[LED_COUNT];
    EmotiBitVersionController::EmotiBitVersion _hwVersion;
    NCP5623 ncp;

    bool init(EmotiBitVersionController::EmotiBitVersion hwVersion, TwoWire &_i2c, int driverCurrent);
    uint8_t getNcpMappedLed(Led led);
    void setState(Led led, bool state);
    bool getState(Led led);
    bool update();
};