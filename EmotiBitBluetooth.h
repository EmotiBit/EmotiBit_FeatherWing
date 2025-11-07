/**************************************************************************/
/*!
    @file     EmotiBitBluetooth.h

    This file facilitates the use of Bluetooth on the EmotiBit

    EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    Written by Joseph Jacobson for EmotiBit.

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#ifdef ARDUINO_FEATHER_ESP32
#pragma once
/*!
* @brief inclusions for BLE Device, Server, Utils, 2902, and Arduino   
*/
#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "EmotiBitPacket.h"

#define EMOTIBIT_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define EMOTIBIT_DATA_RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define EMOTIBIT_DATA_TX_CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/*!
 * @brief Handles Bluetooth communication for EmotiBit.
*/
class EmotiBitBluetooth {
        public:
        BLEServer* pServer = nullptr; ///points to the server
        
        BLECharacteristic* pDataTxCharacteristic = nullptr; ///points to the data tx characteristic
        BLECharacteristic* pDataRxCharacteristic = nullptr; ///points to the data rx characteristic

        bool deviceConnected = false; ///boolean to check if device is connected
        String _emotibitDeviceId = ""; ///string to hold device id
        String _receivedControlMessage = "";
        bool _bluetoothOff = true;
        bool _bluetoothReconnect = false;

        /*!
        * @brief Server callbacks for connections
        */
        class MyServerCallbacks: public BLEServerCallbacks {
                public:
                MyServerCallbacks(EmotiBitBluetooth* server) : server(server) {}
                void onConnect(BLEServer* pServer);
                void onDisconnect(BLEServer* pServer);
                private:
                EmotiBitBluetooth* server;
        };
        
        /*!
        * @brief Characteristic callbacks for data transfer
        */
        class MyCallbacks : public BLECharacteristicCallbacks {
                void onWrite(BLECharacteristic *pCharacteristic);
        };

        /*!
        * @brief Initializes the BLE device and starts advertising
        * @param emotibitDeviceId ID from setDeviceId
        * @return 1 on success, 0 on failure
        */
        //TO DO use int for error handling
        uint8_t begin(const String& emotibitDeviceId);
        
        /*!
        * @brief Sends data over BLE
        * @param message data to be sent
        */
        void sendData(const String &message);
        
        /*!
        * @brief Checks if the device is connected to a BLE client  
        * @param emotibitDeviceId
        */
        void setDeviceId(const String& emotibitDeviceId); 

        /*!
        * @brief Reads control messages from the BLE characteristic
        * @param packet the control message packet
        */
        uint8_t readControl(String& packet);

        //void update();for when we sync data over BLE
        //move to emotibit
        //void sdCardFileNaming(); for when we choose bluetooth and there is no rb start time

        /*!
        * @brief Ends the BLE connection
        */
        void end();

        /*!
        * @brief Checks if the Bluetooth is off
        * @return if Bluetooth is off, returns true, otherwise false
        */
        bool isOff();

        /*!
        * @brief Reconnects the BLE server if disconnected
        */
        void reconnect();

        /*!
        * @brief Starts Advertising
        */
        void startAdvertising();
};

#endif //ARDUINO_FEATHER_ESP32