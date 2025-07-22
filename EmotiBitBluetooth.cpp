#ifdef ARDUINO_FEATHER_ESP32
#include "EmotiBitBluetooth.h"

uint8_t EmotiBitBluetooth::begin(const String& emotibitDeviceId)
{
    if (pServer)
    {
        
        EmotiBitBluetooth::reconnect();
        Serial.println("Bluetooth already initialized, reconnecting...");
        return 1; // Success
    }

//IF BLUETOOTH
//#ifdef BLUETOOTH_ENABLED
    _emotibitDeviceId = emotibitDeviceId;
    
    //btStart();
    Serial.println("Bluetooth tag detected, turning on bluetooth.");
    BLEDevice::init(("EmotiBit: " + _emotibitDeviceId).c_str());

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(this));
    BLEService* pService = pServer->createService(EMOTIBIT_SERVICE_UUID);

    pDataTxCharacteristic = pService->createCharacteristic(EMOTIBIT_DATA_TX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pDataTxCharacteristic->addDescriptor(new BLE2902());

    pDataRxCharacteristic = pService->createCharacteristic(EMOTIBIT_DATA_RX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
    pDataRxCharacteristic->setCallbacks(new MyCallbacks());

    //pSyncTxCharacteristic = pService->createCharacteristic(EMOTIBIT_SYNC_TX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    //pSyncTxCharacteristic->addDescriptor(new BLE2902());

    //pSyncRxCharacteristic = pService->createCharacteristic(EMOTIBIT_SYNC_RX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
    //pSyncRxCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();

    pServer->getAdvertising()->start();
    //_bluetoothOff = false; //was true??
    _bluetoothReconnect = true; // Allow reconnection after disconnection
    Serial.println("BLE advertising started");

    return 1;
//#else
//    Serial.println("Bluetooth disabled.");
//    return 0;
//#endif
}

//ToDO: consider splitting this function into two: one for sending data and another for sending sync data
void EmotiBitBluetooth::MyServerCallbacks::onConnect(BLEServer* pServer)
{
    server -> deviceConnected = true;
    Serial.println("BLE client connected"); 
}

void EmotiBitBluetooth::MyServerCallbacks::onDisconnect(BLEServer* pServer)
{

    server -> deviceConnected = false;
    Serial.println("BLE client disconnected");
    //need to restart advertising to allow new connections after disconnection if accidental
    //ToDo gracefully handle this
    if (server -> _bluetoothReconnect)
    {
        server -> reconnect();
        Serial.println("Restarted BLE advertising");
    }
}


void EmotiBitBluetooth::MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) 
{
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
    //Serial.print("Received: ");
    //Serial.println(rxValue.c_str());
    }
}

void EmotiBitBluetooth::setDeviceId(const String emotibitDeviceId)
{
	_emotibitDeviceId = emotibitDeviceId;
}

/*
void EmotiBitBluetooth::sendData(const String &message)
{
    if (deviceConnected) {
        pDataTxCharacteristic->setValue(message.c_str());
        pDataTxCharacteristic->notify();

        //Serial.print("Sent: ");
        //Serial.println(message.c_str());
    }
    else {
        Serial.println("unable to send data.");
    }
}
*/

void EmotiBitBluetooth::sendData(const String &message)
{
    if (deviceConnected) {
        if (pDataTxCharacteristic == nullptr) {
            //Serial.println("ERROR: pDataTxCharacteristic is NULL!");
            return;
        }
        
        //Serial.print("BLE TX: Message length=");
        //Serial.print(message.length());
        
        // Set the value
        pDataTxCharacteristic->setValue(message.c_str());
        
        // Call notify - note: doesn't return success/failure status
        pDataTxCharacteristic->notify();
        
        // Check descriptor status
        BLEDescriptor* p2902 = pDataTxCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        if (p2902) {
            const uint8_t* val = p2902->getValue();
            if (val) {
                //Serial.print(" | Notifications enabled=");
                //Serial.print((val[0] & 0x01) ? "YES" : "NO");
            }
        }
        
        //Serial.print(" | Char ptr=0x");
        //Serial.print((uint32_t)pDataTxCharacteristic, HEX);
        //Serial.print(" | Message: ");
        //Serial.println(message.c_str());
    }
    else {
        Serial.println("unable to send data: deviceConnected=false");
    }
}

uint8_t EmotiBitBluetooth::readControl(String& packet)
{
    uint8_t numPackets = 0;
//#ifdef BLUETOOTH_ENABLED
    packet = "";
    if (deviceConnected)
    {

        std::string rxValue = pDataRxCharacteristic->getValue();
        if (!rxValue.empty())
        {
            //Serial.print("Received: ");
            //Serial.println(rxValue.c_str());
            _receivedControlMessage += String(rxValue.c_str());

            // *** CLEAR THE CHAR VALUE SO WE DON’T REUSE IT ***
            pDataRxCharacteristic->setValue("");  
        }

        String tempPacket = "";
        while (_receivedControlMessage.length() > 0)
        {
            int c = _receivedControlMessage[0];
            _receivedControlMessage.remove(0, 1);

            if (c == (int)EmotiBitPacket::PACKET_DELIMITER_CSV)
            {
                numPackets++;
                packet = tempPacket;
                tempPacket = "";
				_receivedControlMessage = "";
                return numPackets;
            }
            else
            {
                if (c == 0) {
                    // Throw out null term
                }
                else
                {
                    tempPacket += (char)c;
                }
            }
        }
    }
//#endif
    return numPackets;
}

bool EmotiBitBluetooth::isOff()
{
	return _bluetoothOff;
}

/*
void EmotiBitBluetooth::end()
{
    //if (pServer && deviceConnected) {
    //    // Disconnect all connected clients
    //    pServer->disconnect(0); // 0 = first client, or use the correct connection ID if you track it
        deviceConnected = false;
    //    Serial.println("BLE client disconnected by end()");
    //}
    if (pServer) {
        pServer->getAdvertising()->stop();
        Serial.println("BLE advertising stopped");
    }

    //esp_bt_controller_disable();

    //pServer = nullptr;
    //pDataTxCharacteristic = nullptr;
    //pDataRxCharacteristic = nullptr;
    _bluetoothOff = true;
    _bluetoothReconnect = false; // No longer allow reconnection
    //btStop();
    //esp_bt_controller_deinit();
}
*/
void EmotiBitBluetooth::end()
{
    if (pServer && deviceConnected)
    {
        // gracefully tear down the old connection
        pServer->disconnect(0);  
        Serial.println("BLE client disconnected by end()");
    }
    if (pServer)
    {
        pServer->getAdvertising()->stop();
        Serial.println("BLE advertising stopped");
    }
    _bluetoothOff      = true;
    _bluetoothReconnect = false;
    // NOTE: do *not* touch deviceConnected here—let onDisconnect handle it
}


void EmotiBitBluetooth::reconnect()
{
    if (!pServer) return;
    pServer->getAdvertising()->start();
    Serial.println("BLE advertising restarted after reconnect");
    _bluetoothOff      = false;
    _bluetoothReconnect = true;
}
#endif //ARDUINO_FEATHER_ESP32