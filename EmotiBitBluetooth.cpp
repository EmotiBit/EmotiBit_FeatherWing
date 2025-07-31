#ifdef ARDUINO_FEATHER_ESP32
#include "EmotiBitBluetooth.h"

uint8_t EmotiBitBluetooth::begin(const String& emotibitDeviceId)
{
        if (pServer)
        {
                EmotiBitBluetooth::reconnect();
                Serial.println("Bluetooth already initialized, reconnecting...");
                return 0; // Success
        }

        _emotibitDeviceId = emotibitDeviceId;
    
        Serial.println("Bluetooth tag detected, turning on bluetooth.");
        BLEDevice::init(("EmotiBit: " + _emotibitDeviceId).c_str());

        pServer = BLEDevice::createServer();

        if (!pServer) {
                Serial.println("ERROR: Failed to create BLE server");
                return 1;
        }

        pServer->setCallbacks(new MyServerCallbacks(this));
        BLEService* pService = pServer->createService(EMOTIBIT_SERVICE_UUID);

        pDataTxCharacteristic = pService->createCharacteristic(EMOTIBIT_DATA_TX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
        pDataTxCharacteristic->addDescriptor(new BLE2902());

        pDataRxCharacteristic = pService->createCharacteristic(EMOTIBIT_DATA_RX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
        pDataRxCharacteristic->setCallbacks(new MyCallbacks());

        pService->start();

        EmotiBitBluetooth::startAdvertising();
        _bluetoothReconnect = true; // Allow reconnection after disconnection
        Serial.println("BLE advertising started");

        return 0;
}

void EmotiBitBluetooth::MyServerCallbacks::onConnect(BLEServer* pServer)
{
        server -> deviceConnected = true;
        Serial.println("BLE client connected"); 
}

void EmotiBitBluetooth::MyServerCallbacks::onDisconnect(BLEServer* pServer)
{

        server -> deviceConnected = false;
        Serial.println("BLE client disconnected");
        //need to restart advertising to allow new connections after disconnection if accidentally disconnected
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
        packet = "";
        if (deviceConnected)
        {
                std::string rxValue = pDataRxCharacteristic->getValue();
                if (!rxValue.empty())
                {
                        //Serial.print("Received: ");
                        //Serial.println(rxValue.c_str());
                        _receivedControlMessage += String(rxValue.c_str());

                        //CLEAR THE CHAR VALUE SO WE DON’T REUSE IT
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
        return numPackets;
}

bool EmotiBitBluetooth::isOff()
{
	return _bluetoothOff;
}


void EmotiBitBluetooth::end()
{
        if (pServer && deviceConnected)
        {
                //tear down the old connection
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
}


void EmotiBitBluetooth::reconnect()
{
        if (pServer) 
        {
                EmotiBitBluetooth::startAdvertising();
                Serial.println("BLE advertising restarted after reconnect");
                _bluetoothOff      = false;
                _bluetoothReconnect = true;
        }
        else
        {
                Serial.println("ERROR: pServer is NULL, cannot reconnect");
        }
}

void EmotiBitBluetooth::startAdvertising()
{
        if (pServer)
        {
                pServer->getAdvertising()->start();
                Serial.println("BLE advertising started");
        }
        else
        {
        Serial.println("ERROR: pServer is NULL, cannot start advertising");
        }
}

#endif //ARDUINO_FEATHER_ESP32