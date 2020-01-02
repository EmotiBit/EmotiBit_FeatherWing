/// EmotiBitWifiTest01
///
/// Example using EmotiBitWiFi to manage WiFi connections
/// between the EmotiBit and EmotiBit data visualizer

//#define ARDUINO
#include "EmotiBitWiFi.h"
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h

EmotiBitWiFi emotibitWiFi;
uint16_t controlPacketNumber = 0;
String dataMessage;
const uint16_t DATA_SEND_INTERVAL = 100;
const uint16_t DATA_MESSAGE_RESERVE_SIZE = 4096;
String updatePackets;
uint16_t dataPacketCounter = 0;

enum class DataType {
	PPG_INFRARED,
	PPG_RED,
	PPG_GREEN,
	EDA,
	EDL,
	EDR,
	TEMPERATURE_0,
	THERMOPILE,
	HUMIDITY_0,
	ACCELEROMETER_X,
	ACCELEROMETER_Y,
	ACCELEROMETER_Z,
	GYROSCOPE_X,
	GYROSCOPE_Y,
	GYROSCOPE_Z,
	MAGNETOMETER_X,
	MAGNETOMETER_Y,
	MAGNETOMETER_Z,
	BATTERY_PERCENT,
	length
};

const char *typeTags[(uint8_t)DataType::length];

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

	emotibitWiFi.addCredential(SECRET_SSID_0, SECRET_PASS_0);
	emotibitWiFi.addCredential(SECRET_SSID_1, SECRET_PASS_1);

#if defined(ADAFRUIT_FEATHER_M0)
	WiFi.setPins(8, 7, 4, 2);
#endif
	emotibitWiFi.begin();

	setupDataTypes();
}

void loop() { 
	
	emotibitWiFi.update(updatePackets, dataPacketCounter);
	//Serial.print(updatePackets);

  if (emotibitWiFi.isConnected()) {
		// Read control packets
		String controlPacket;
		while (emotibitWiFi.readControl(controlPacket))
		{
			// ToDo: handling some packets (e.g. disconnect behind the scenes)
			Serial.print("Receiving control msg: ");
			Serial.println(controlPacket);
			EmotiBitPacket::Header header;
			EmotiBitPacket::getHeader(controlPacket, header);
			if (header.typeTag.equals(EmotiBitPacket::TypeTag::EMOTIBIT_DISCONNECT))
			{
				emotibitWiFi.disconnect();
			}
			dataMessage += controlPacket;
		}

	  // Send data periodically
	  static uint32_t dataSendTimer = millis();
	  if (millis() - dataSendTimer > DATA_SEND_INTERVAL) 
		{
			dataSendTimer = millis();
			for (uint8_t t = 0; t < (uint8_t)DataType::length; t++)
			{
				for (int i = 0; i < 5; i++)
				{
					//String data = "0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9";
					String data;
					int k;
					int nData = 3;
					for (int j = 0; j < nData; j++)
					{
						if (j != 0)
						{
							data += ",";
						}
						k = i * 50 + random(50);
						data += k;
					}
					dataMessage += EmotiBitPacket::createPacket(typeTags[t], dataPacketCounter++, data, nData);
				}
			}

			emotibitWiFi.sendData(dataMessage);
			dataMessage = "";
	  }
  }
}

void setupDataTypes()
{
	typeTags[(uint8_t)DataType::EDA] = EmotiBitPacket::TypeTag::EDA;
	typeTags[(uint8_t)DataType::EDL] = EmotiBitPacket::TypeTag::EDL;
	typeTags[(uint8_t)DataType::EDR] = EmotiBitPacket::TypeTag::EDR;
	typeTags[(uint8_t)DataType::PPG_INFRARED] = EmotiBitPacket::TypeTag::PPG_INFRARED;
	typeTags[(uint8_t)DataType::PPG_RED] = EmotiBitPacket::TypeTag::PPG_RED;
	typeTags[(uint8_t)DataType::PPG_GREEN] = EmotiBitPacket::TypeTag::PPG_GREEN;
	typeTags[(uint8_t)DataType::TEMPERATURE_0] = EmotiBitPacket::TypeTag::TEMPERATURE_0;
	typeTags[(uint8_t)DataType::THERMOPILE] = EmotiBitPacket::TypeTag::THERMOPILE;
	typeTags[(uint8_t)DataType::HUMIDITY_0] = EmotiBitPacket::TypeTag::HUMIDITY_0;
	typeTags[(uint8_t)DataType::ACCELEROMETER_X] = EmotiBitPacket::TypeTag::ACCELEROMETER_X;
	typeTags[(uint8_t)DataType::ACCELEROMETER_Y] = EmotiBitPacket::TypeTag::ACCELEROMETER_Y;
	typeTags[(uint8_t)DataType::ACCELEROMETER_Z] = EmotiBitPacket::TypeTag::ACCELEROMETER_Z;
	typeTags[(uint8_t)DataType::GYROSCOPE_X] = EmotiBitPacket::TypeTag::GYROSCOPE_X;
	typeTags[(uint8_t)DataType::GYROSCOPE_Y] = EmotiBitPacket::TypeTag::GYROSCOPE_Y;
	typeTags[(uint8_t)DataType::GYROSCOPE_Z] = EmotiBitPacket::TypeTag::GYROSCOPE_Z;
	typeTags[(uint8_t)DataType::MAGNETOMETER_X] = EmotiBitPacket::TypeTag::MAGNETOMETER_X;
	typeTags[(uint8_t)DataType::MAGNETOMETER_Y] = EmotiBitPacket::TypeTag::MAGNETOMETER_Y;
	typeTags[(uint8_t)DataType::MAGNETOMETER_Z] = EmotiBitPacket::TypeTag::MAGNETOMETER_Z;
	typeTags[(uint8_t)DataType::BATTERY_PERCENT] = EmotiBitPacket::TypeTag::BATTERY_PERCENT;
}

