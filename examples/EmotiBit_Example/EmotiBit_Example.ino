#include "EmotiBit.h"

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];

void onShortButtonPress()
{
	// toggle wifi on/off
	if (emotibit.getWiFiMode() == EmotiBit::WiFiMode::NORMAL)
	{
		emotibit.setWiFiMode(EmotiBit::WiFiMode::OFF);
		Serial.println("WiFiMode::OFF");
	}
	else
	{
		emotibit.setWiFiMode(EmotiBit::WiFiMode::NORMAL);
		Serial.println("WiFiMode::NORMAL");
	}
}

void onLongButtonPress()
{
	emotibit.hibernate();
}

void setup() 
{
	Serial.begin(SERIAL_BAUD);
	Serial.println("Serial started");
	delay(2000);	// short delay to allow user to connect to serial, if desired

	emotibit.setup(EmotiBit::Version::V02H);

	// Attach callback functions
	//emotibit.attachShortButtonPress(&onShortButtonPress);
	//emotibit.attachLongButtonPress(&onLongButtonPress);
}

void loop()
{
	emotibit.update();

	//size_t dataAvailable = emotibit.readData(EmotiBit::DataType::EDA, data, dataSize);
	//if (dataAvailable > 0)
	//{
	//	// Hey cool, I got some data! Maybe I can light up my shoes whenever I get excited!

	//	// print the data to view in the serial plotter
	//	bool printData = false;
	//	if (printData)
	//	{
	//		for (size_t i; i < dataAvailable && i < dataSize; i++)
	//		{
	//			Serial.println(data[i]);
	//		}
	//	}
	//}
}