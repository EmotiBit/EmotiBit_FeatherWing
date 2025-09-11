# EmotiBit FeatherWing
## Getting Started with EmotiBit
See https://github.com/EmotiBit/EmotiBit_Docs/blob/master/Getting_Started.md

## Compile from source
### Downloading all dependencies
#### Using Arduino IDE
- The required dependencies are automatically downloaded when EmotiBit_FeatherWing library is installed using Arduino.
- The detailed steps can be found in our documentation:[Keeping EmotiBit upto date > Update firmware using Arduino IDE](https://github.com/EmotiBit/EmotiBit_Docs/blob/master/Keep_emotibit_up_to_date.md#update-firmware-using-arduino-ide)

#### Using github (shell script)
- Note: requires `jq`(command line JSON processor)
- Run the `download_dependencies.sh` script to download all the required libraries.
- Alternatively, you can manually download the github repositories for the dependencies listed in the `library.properties` file.

### Build
- To build on Arduino, Open the EmotiBit_stock_firmware > EmotiBit_stock_firmware.ino. Press the build button.
- To build using PlatformIO, follow the instructions in our documentation: [`Building firmware using PlatformIO`]( https://github.com/EmotiBit/EmotiBit_Docs/blob/master/Keep_emotibit_up_to_date.md#building-firmware-using-platformio)
