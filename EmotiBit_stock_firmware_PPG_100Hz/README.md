# EmotiBit_stock_firmware_PPG_100Hz
### Description
This firmware file is provided to enable 100Hz sampling of the PPG sensor.
### WARNING
Adequately buffering 100Hz PPG uses a LOT more RAM on the device. On the `Feather M0 WiFi`, this requires reducing the overall data buffer durations that the device can handle. **If you find `Overflow Events` are occurring regularly with PPG at 100Hz sampling, you may wish to go back to stock 25Hz PPG firmware or purchase the HUZZAH32 Feather from Adafruit for expanded MCU RAM.** [ToDo: Add link to HUZZAH32]
### Steps required

### Regarding firmware
- We recommend using the compiled binary file provided with the release.
- If you wish to compile from source, then you will need to install and compile using platform IO.
  - See our instructions here. [ToDo] Add a link once documentation has been merged to master.
  - NOTE: We are still testing our process and support for platformIO, so we recommend building from source only if you are experienced with embedded system workflows. 
### Regarding software
- Replace `ofxOscilloscopeSettings.xml` with provided file [ToDo: describe steps in detail]
  - windows
  - mac
