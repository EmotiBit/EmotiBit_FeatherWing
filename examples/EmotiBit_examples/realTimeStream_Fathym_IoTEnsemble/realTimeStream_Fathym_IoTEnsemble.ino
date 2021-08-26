#include "EmotiBit.h"
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "Esp32MQTTClient.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <algorithm>

Adafruit_7segment matrix = Adafruit_7segment();

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];

//	Fathym Cloud Connect
char fathymConnectionString[] = "";
char fathymDeviceID[] = "";
char fathymReadings[][] = {};
static bool hasIoTHub = false;
static int readingsInterval = 5000;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
char metadataTypeTags[] = { "BV", "BP", "DO", "DC", "DB" };

void onShortButtonPress()
{
	// toggle wifi on/off
	if (emotibit.getPowerMode() == EmotiBit::PowerMode::NORMAL_POWER)
	{
		emotibit.setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
		Serial.println("PowerMode::WIRELESS_OFF");
	}
	else
	{
		emotibit.setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
		Serial.println("PowerMode::NORMAL_POWER");
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
	Serial.println("EmotiBit Labs - Cloud Streaming");
	
	delay(2000);	// short delay to allow user to connect to serial, if desired

	emotibit.setup(EmotiBit::Version::V02H);
	// emotibit.setup();

	//	Connecting to time server for timestamping
	timeClient.begin();
	timeClient.update();

	if (!loadConfigFile(_configFilename)) {
		Serial.println("SD card configuration file parsing failed.");
		Serial.println("Create a file 'config.txt' with the following JSON:");
		Serial.println("{\"WifiCredentials\": [{\"ssid\": \"SSSS\", \"password\" : \"PPPP\"}],\"Fathym\":{\"ConnectionString\": \"xxx\", \"DeviceID\": \"yyy\"}}");
		while (true) {
			hibernate();
		}
	}

	if (!Esp32MQTTClient_Init((const uint8_t*)fathymConnectionString, true))
	{​​​​​​​​
		hasIoTHub = false;

		Serial.println("Initializing IoT hub failed.");

		return;
	}​​​​​​​​

	hasIoTHub = true;

	// Attach callback functions
	emotibit.attachShortButtonPress(&onShortButtonPress);
	emotibit.attachLongButtonPress(&onLongButtonPress);
}

void loop()
{
	//Serial.println("emotibit.update()");
	emotibit.update();

	// allocate the memory for the document
	const size_t CAPACITY = JSON_OBJECT_SIZE(1);
	StaticJsonDocument<CAPACITY> doc;

	// create an object
	JsonObject payload = doc.to<JsonObject>();

	payload["DeviceID"] = fathymDeviceID;

	payload["DeviceType"] = "emotibit";

	payload["Version"] = "1";

	// object["Timestamp"] = timeClient.getFormattedDate();

	JsonObject payloadDeviceData = payload.createNestedObject("DeviceData");

	payloadDeviceData["EpochTime"] = timeclient.getEpochTime();

	payloadDeviceData["Timestamp"] = timeclient.getFormattedDate();

	JsonObject payloadSensorReadings = payload.createNestedObject("SensorReadings");

	JsonObject payloadSensorMetadata = payload.createNestedObject("SensorMetadata");

	JsonObject payloadSensorMetadataRoot = payloadSensorMetadata.createNestedObject("_");

	for (char typeTag[] : fathymReadings) {
		uint8_t dataType = loadDataTypeFromTypeTag(typeTag);

		size_t dataAvailable = emotibit.readData(dataType, &data[0], dataSize);
		
		if (dataAvailable > 0)
		{
			if (!contains(metadataTypeTags, typeTag))
			{
				payloadSensorReadings[typeTag] = data;
			}
			else
			{
				payloadSensorMetadataRoot[typeTag] = data;
			}

			// print the data to view in the serial plotter
			bool printData = false;
			if (printData)
			{
				for (size_t i = 0; i < dataAvailable && i < dataSize; i++)
				{
					// Note that dataAvailable can be larger than dataSize
					Serial.println(data[i]);
				}
			}
		}
	}
	
	char messagePayload[];

	// serialize the payload for sending
	serializeJson(doc, messagePayload);

	Serial.println(messagePayload);

	EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);

	Esp32MQTTClient_SendEventInstance(message);

	delay(readingsInterval);
}

// Loads the configuration from a file
bool loadConfigFile(const String &filename) {
	// Open file for reading
	File file = SD.open(filename);

	if (!file) {
		Serial.print("File ");
		Serial.print(filename);
		Serial.println(" not found");
		return false;
	}

	Serial.print("Parsing: ");
	Serial.println(filename);

	// Allocate the memory pool on the stack.
	// Don't forget to change the capacity to match your JSON document.
	// Use arduinojson.org/assistant to compute the capacity.
	//StaticJsonBuffer<1024> jsonBuffer;
	StaticJsonBuffer<1024> jsonBuffer;

	// Parse the root object
	JsonObject &root = jsonBuffer.parseObject(file);

	if (!root.success()) {
		Serial.println(F("Failed to parse config file"));
		return false;
	}

	fathymConnectionString = root["Fathym"]["ConnectionString"] | "";
	
	fathymDeviceID = root["Fathym"]["DeviceID"] | "";

	fathymReadings = root["Fathym"]["Readings"] | "";

	readingsInterval = root["Fathym"]["ReadingsInterval"] | 5000;

	//strlcpy(config.hostname,                   // <- destination
	//	root["hostname"] | "example.com",  // <- source
	//	sizeof(config.hostname));          // <- destination's capacity

	// Close the file (File's destructor doesn't close the file)
	// ToDo: Handle multiple credentials

	file.close();
	return true;
}

uint8_t loadDataTypeFromTypeTag(char typeTag[]) {
	uint8_t dataType;

	switch (typeTag) {
		case "AX":
			dataType = EmotiBit::DataType::ACCELEROMETER_X;
			break;
			
		case "AY":
			dataType = EmotiBit::DataType::ACCELEROMETER_Y;
			break;
			
		case "AZ":
			dataType = EmotiBit::DataType::ACCELEROMETER_Z;
			break;
			
		case "GX":
			dataType = EmotiBit::DataType::GYROSCOPE_X;
			break;
			
		case "GY":
			dataType = EmotiBit::DataType::GYROSCOPE_Y;
			break;
			
		case "GZ":
			dataType = EmotiBit::DataType::GYROSCOPE_Z;
			break;
			
		case "MX":
			dataType = EmotiBit::DataType::MAGNETOMETER_X;
			break;
			
		case "MY":
			dataType = EmotiBit::DataType::MAGNETOMETER_Y;
			break;
			
		case "MZ":
			dataType = EmotiBit::DataType::MAGNETOMETER_Z;
			break;
			
		case "EA":
			dataType = EmotiBit::DataType::EDA;
			break;
			
		case "EL":
			dataType = EmotiBit::DataType::EDL;
			break;
			
		case "ER":
			dataType = EmotiBit::DataType::EDR;
			break;
			
		case "H0":
			dataType = EmotiBit::DataType::HUMIDITY_0;
			break;
			
		case "T0":
			dataType = EmotiBit::DataType::TEMPERATURE_0;
			break;
			
		case "TH":
			dataType = EmotiBit::DataType::THERMOPILE;
			break;
			
		case "PI":
			dataType = EmotiBit::DataType::PPG_INFRARED;
			break;
			
		case "PR":
			dataType = EmotiBit::DataType::PPG_RED;
			break;
			
		case "PG":
			dataType = EmotiBit::DataType::PPG_GREEN;
			break;
			
		case "BV":
			dataType = EmotiBit::DataType::BATTERY_VOLTAGE;
			break;
			
		case "BP":
			dataType = EmotiBit::DataType::BATTERY_PERCENT;
			break;
			
		case "DO":
			dataType = EmotiBit::DataType::DATA_OVERFLOW;
			break;
			
		case "DC":
			dataType = EmotiBit::DataType::DATA_CLIPPING;
			break;
			
		case "DB":
			dataType = EmotiBit::DataType::DEBUG;
			break;
	}

	return dataType;
}

bool contains(C&& c, T e) { 
    return std::find(std::begin(c), std::end(c), e) != std::end(c);
};
