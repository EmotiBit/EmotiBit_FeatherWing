#include "EmotiBitBluetooth.h"
//NOTE, remove ifdefs for bluetooth when we can manually choose to enable/disable bluetooth



uint8_t EmotiBitBluetooth::begin(const String& emotibitDeviceId)
{
//IF BLUETOOTH
#ifdef BLUETOOTH_ENABLED
    _emotibitDeviceId = emotibitDeviceId;
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
    Serial.println("BLE advertising started");
    return 1;
#else
    Serial.println("Bluetooth disabled.");
    return 0;
#endif
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
    //need to restart advertising to allow new connections after disconnection
    pServer->getAdvertising()->start();
    Serial.println("Restarted BLE advertising");
}


void EmotiBitBluetooth::MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) 
{
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
    Serial.print("Received: ");
    Serial.println(rxValue.c_str());
    }
}

void EmotiBitBluetooth::setDeviceId(const String emotibitDeviceId)
{
	_emotibitDeviceId = emotibitDeviceId;
}

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

uint8_t EmotiBitBluetooth::readControl(String& packet)
{
    uint8_t numPackets = 0;
#ifdef BLUETOOTH_ENABLED
    packet = "";
    if (deviceConnected)
    {

        std::string rxValue = pDataRxCharacteristic->getValue();
        if (!rxValue.empty())
        {
            Serial.print("Received: ");
            Serial.println(rxValue.c_str());
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
#endif
    return numPackets;
}