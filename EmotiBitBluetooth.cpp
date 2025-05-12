#include "EmotiBitBluetooth.h"




uint8_t EmotiBitBluetooth::begin(const String& emotibitDeviceId)
{
//IF BLUETOOTH
#ifdef BLUETOOTH_ENABLED
    _emotibitDeviceId = emotibitDeviceId;
    Serial.println("Bluetooth tag detected, turning on bluetooth.");
    BLEDevice::init(("EmotiBit: " + _emotibitDeviceId).c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(this));
    BLEService* pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    pTxCharacteristic = pService->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    pServer->getAdvertising()->start();
    return 1;
#else
    Serial.println("Bluetooth disabled.");
    return 0;
#endif
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
        pTxCharacteristic->setValue(message.c_str());
        pTxCharacteristic->notify();
        //Serial.print("Sent: ");
        //Serial.println(message.c_str());
    }
    else {
        Serial.println("unable to send data.");
    }
}