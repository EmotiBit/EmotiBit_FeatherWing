#ifndef EMOTIBIT_CONFIG_MANAGER_H
#define EMOTIBIT_CONFIG_MANAGER_H
#include <Arduino.h>
#include <EmotiBitSerial.h>
#include <EmotiBitFactoryTest.h>
#include <ArduinoJson.h>
#include <EmotiBitPacket.h>
#ifdef ARDUINO_FEATHER_ESP32
#include <SD.h>
#else
#include <SdFat.h>
#endif

// ToDo: This may not be the best place for this functionality. Consider a better place in the codebase.
class EmotiBitConfigManager
{
public:
#ifdef ARDUINO_FEATHER_ESP32
	// SD is already defined in ESP32 
    fs::SDFS* SD;
    /*!
    * @brief initialize sd card
    * @param sd pointer to emotibit SD
    * @return true if pointer initialized
    */
    bool init(fs::SDFS* sd);
#else
	SdFat* SD;	// Use SdFat library
    /*!
    * @brief initialize sd card
    * @param sd pointer to emotibit SD
    * @return true if pointer initialized
    */
    bool init(SdFat* sd);
#endif
    

    
    /*! 
    * @brief create a new config file from JsonDocument element
    * @param configFilename Name of config file
    * @param file instance of file that will handle file I/O
    * @param configAsJson Json Element that holds credentials data
    * @return returns true, is config file is successfully created.
    */
    bool createNewConfigFile(String configFilename, File& file, JsonDocument& configAsJson);
    /*! 
    * @brief Function to update WiFi credentials over Serial.
    * @param emotibitFwVersion Current EmotiBit firmware version. Added for future compatibility for software-firmware handshaking
    * @param configFilename name of the config file on the SD card
    * @param maxCreds Maximum allowed credentials on the SD card
    */
    void updateWiFiCredentials(String emotibitFwVersion, String configFilename, const uint8_t maxCreds);
    // ToDo: In the future, we may want to update creds through WiFi, so this functio can act as a wrapper to call other functions that handle WiFi/serial process.
};
#endif