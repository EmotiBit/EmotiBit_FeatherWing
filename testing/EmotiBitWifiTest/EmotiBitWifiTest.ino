/// EmotiBitWifiTest01
///
/// Example using EmotiBitWiFi to manage WiFi connections
/// between the EmotiBit and EmotiBit data visualizer

//#define ARDUINO
#include "EmotiBitWiFi.h"
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h

EmotiBitWiFi emotibitWifi;
uint16_t controlPacketNumber = 0;
String dataMessage;
const uint16_t DATA_SEND_INTERVAL = 100;
const uint16_t DATA_MESSAGE_RESERVE_SIZE = 4096;
String updatePackets;


void setup() 
{
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
	uint32_t serialTimer = millis();
  while (!Serial) 
	{
		// wait for serial port to connect. Needed for native USB port only
		if (millis() - serialTimer > 5000)
		{
			break; 
		}
  }

	dataMessage.reserve(DATA_MESSAGE_RESERVE_SIZE);

	emotibitWifi.addCredential(SECRET_SSID_0, SECRET_PASS_0);
	emotibitWifi.addCredential(SECRET_SSID_1, SECRET_PASS_1);

	emotibitWifi.setup();
	emotibitWifi.begin();
}

void loop() { 
	
	emotibitWifi.update(updatePackets);
	//Serial.print(updatePackets);

  if (emotibitWifi._isConnected) {
		// Read control packets
		String controlPacket;
		while (emotibitWifi.readControl(controlPacket))
		{
			// ToDo: handling some packets (e.g. disconnect behind the scenes)
			Serial.print("Receiving control msg: ");
			Serial.println(controlPacket);
			EmotiBitPacket::Header header;
			EmotiBitPacket::getHeader(controlPacket, header);
			if (header.typeTag.equals(EmotiBitPacket::TypeTag::EMOTIBIT_DISCONNECT))
			{
				emotibitWifi.disconnect();
			}
			dataMessage += controlPacket;
		}

	  // Send data periodically
	  static uint32_t dataSendTimer = millis();
	  if (millis() - dataSendTimer > DATA_SEND_INTERVAL) {
			dataSendTimer = millis();
			String data = "0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9";
			static uint16_t counter = 0;
			for (int i = 0; i < 20; i++)
			{
				dataMessage += EmotiBitPacket::createPacket(EmotiBitPacket::TypeTag::DEBUG, emotibitWifi.dataPacketCounter++, data, 50);
			}

			emotibitWifi.sendData(dataMessage);
			dataMessage = "";
	  }
  }
}


