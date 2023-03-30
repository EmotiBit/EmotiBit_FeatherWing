# EmotiBit_stock_firmware_PPG_100Hz
### Description
This firmware file is provided to enable 100Hz sampling of the PPG sensor.
### WARNING
Adequately buffering 100Hz PPG uses a LOT more RAM on the device. On the `Feather M0 WiFi`, this requires reducing the overall data buffer durations that the device can handle. **If you find TypeTag `DO`s occur with PPG at 100Hz sampling, you may wish to go back to stock 25Hz PPG firmware or purchase the HUZZAH32 Feather from Adafruit for expanded MCU RAM.** [ToDo: Add link to HUZZAH32]

### Installing firmware
- We recommend using the compiled binary file provided with the release.
- Download the appropriate firmware binary from the firmware [release page](https://github.com/EmotiBit/EmotiBit_FeatherWing/releases).
  - Be careful to match the firmware to the Feather board version you are using.
- Use the latest Firmware Installer to flash the downloaded firmware binary.
  - Open the Firmware installer.
  - Press `L` on the opening screen. A file explorer window will open. 
  - Navigate to the correct `bin` file. Choose the file and click on Open.
  - Continue with the instructions on the Firmware installer.

- Note
  - If you wish to compile from source, then you will need to install and compile using platform IO.
    - See our instructions [here](https://github.com/EmotiBit/EmotiBit_Docs/blob/master/Keep_emotibit_up_to_date.md#Building-firmware-using-PlatformIO).
    - We are still testing our process and support for platformIO, so we recommend building from source only if you are experienced with embedded system workflows. 

### Software settings
- Replace `ofxOscilloscopeSettings.xml` with provided file. Alternatively, you can find the instructions in our [documentation](https://github.com/EmotiBit/EmotiBit_Docs/blob/master/Working_with_emotibit_data.md#emotibit-oscilloscope-display-settings) to update the existing file accordingly.
