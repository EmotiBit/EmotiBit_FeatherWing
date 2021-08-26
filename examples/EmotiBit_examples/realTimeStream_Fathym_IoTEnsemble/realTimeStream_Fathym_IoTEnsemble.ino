#include "EmotiBit.h"
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "Esp32MQTTClient.h"

Adafruit_7segment matrix = Adafruit_7segment();

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];
char fathymConnectionString[] = "";
char fathymDeviceID[] = "";

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

	if (!loadConfigFile(_configFilename)) {
		Serial.println("SD card configuration file parsing failed.");
		Serial.println("Create a file 'config.txt' with the following JSON:");
		Serial.println("{\"WifiCredentials\": [{\"ssid\": \"SSSS\", \"password\" : \"PPPP\"}],\"Fathym\":{\"ConnectionString\": \"xxx\", \"DeviceID\": \"yyy\"}}");
		while (true) {
			hibernate();
		}
	}

	// Attach callback functions
	emotibit.attachShortButtonPress(&onShortButtonPress);
	emotibit.attachLongButtonPress(&onLongButtonPress);
}

void loop()
{
	//Serial.println("emotibit.update()");
	emotibit.update();

	// TODO: Grab desired data types from config (someday)
	
	//	TODO:  Read all datatypes

	//
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDA] = &eda;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDL] = &edl;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::EDR] = &edr;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = &ppgInfrared;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_RED] = &ppgRed;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::PPG_GREEN] = &ppgGreen;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = &temp0;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::THERMOPILE] = &therm0;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = &humidity0;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = &accelX;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = &accelY;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = &accelZ;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = &gyroX;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = &gyroY;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = &gyroZ;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = &magX;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = &magY;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = &magZ;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = &batteryVoltage;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = &batteryPercent;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW] = &dataOverflow;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DATA_CLIPPING] = &dataClipping;
	// dataDoubleBuffers[(uint8_t)EmotiBit::DataType::DEBUG] = &debugBuffer;


	size_t dataAvailable = emotibit.readData(EmotiBit::DataType::THERMOPILE, &data[0], dataSize);
	if (dataAvailable > 0)
	{
		// Print temperature on the 7 segment display!
		static float smoothData = -1;
		float smoother = 0.95f;
		for (size_t i = 0; i < dataAvailable && i < dataSize; i++)
		{
			if (smoothData < 0 )
			{
				// handle initial condition
				smoothData = data[i];
			}
			else
			{
				smoothData = smoothData * smoother + data[i] * (1 - smoother);
			}
		}
		
		//	TODO:  Send data to IoT Ensemble

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
	
	fathymDeviceID = root["Fathym"]["ConnectionString"] | "";

	//strlcpy(config.hostname,                   // <- destination
	//	root["hostname"] | "example.com",  // <- source
	//	sizeof(config.hostname));          // <- destination's capacity

	// Close the file (File's destructor doesn't close the file)
	// ToDo: Handle multiple credentials

	file.close();
	return true;
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
	if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
	{
		Serial.println("Send Confirmation Callback finished.");
	}
}

static void MessageCallback(const char* payLoad, int size)
{
	Serial.println("Message callback:");
	
	Serial.println(payLoad);
}

static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
	char *temp = (char *)malloc(size + 1);
	if (temp == NULL)
	{
		return;
	}

	memcpy(temp, payLoad, size);

	temp[size] = '\0';

	// Display Twin message.
	Serial.println(temp);

	free(temp);
}
static int DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
{
	LogInfo("Try to invoke method %s", methodName);

	const char *responseMessage = "\"Successfully invoke device method\"";

	int result = 200;

	if (strcmp(methodName, "start") == 0)
	{
		LogInfo("Start sending temperature and humidity data");

		messageSending = true;
	}
	else if (strcmp(methodName, "stop") == 0)
	{
		LogInfo("Stop sending temperature and humidity data");
		
		messageSending = false;
	}
	else
	{
		LogInfo("No method %s found", methodName);

		responseMessage = "\"No method found\"";

		result = 404;
	}

	*response_size = strlen(responseMessage) + 1;

	*response = (unsigned char *)strdup(responseMessage);

	return result;
}