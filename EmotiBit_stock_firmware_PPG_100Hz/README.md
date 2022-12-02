# EmotiBit_stock_firmware_PPG_100Hz
### Description
This firmware file is provided to enable 100Hz sampling of the PPG sensor.
### WARNING
Adequately buffering 100Hz PPG uses a LOT more RAM on the device. On the `Feather M0 WiFi`, this requires reducing the overall data buffer durations that the device can handle. **If you find `Overflow Events` are occurring regularly with PPG at 100Hz sampling, you may wish to go back to stock 25Hz PPG firmware or purchase the HUZZAH32 Feather from Adafruit for expanded MCU RAM.** [ToDo: Add link to HUZZAH32]
### Steps required
- Program your Feather with EmotiBit_stock_firmware_PPG_100Hz. [ToDo: describe steps in detail]
  - release
  - vMicro
- Replace ofxOscilloscopeSettings.xml with provided file [ToDo: describe steps in detail]
  - windows
  - mac
