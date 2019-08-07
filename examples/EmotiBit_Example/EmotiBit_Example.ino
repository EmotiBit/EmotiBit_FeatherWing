//#define DEBUG_GET_DATA

#include <DoubleBufferFloat.h>
#include <BufferFloat.h>
#define SEND_UDP
//#define SEND_TCP;
bool sendSerial = false;
volatile bool sdWrite = false;
bool sendConsole = false;

#include "EmotiBit.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SleepyDog.h>
#include <SdFat.h>
#include <ArduinoJson.h>

SdFat SD;

typedef struct EmotibitConfig {
	String ssid = "";
	String password = "";
};
size_t configSize;
size_t configPos;
EmotibitConfig configList[12];
bool switchCred = false;
bool getMomentLost;
uint32_t momentLost;
unsigned short int attempts = 0;




#ifdef SEND_TCP
#include <WiFi101.h> 
//#include "arduino_secrets.h" 
//char ssid[] = SECRET_SSID;        // your network SSID (name)
//char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)
int wifiStatus = WL_IDLE_STATUS;
WiFiClient client;
IPAddress server(192,168,0,106);
int portNum = 11999;
uint32_t refreshConnection = 100;
uint32_t refreshCount = 0;
bool sendTCP = true;
#else // SEND_TCP
bool sendTCP = false;
#endif

#ifdef SEND_UDP
#include <WiFi101.h>
#include <WiFiUdp.h>
//#include "arduino_secrets.h" 

int wifiStatus = WL_IDLE_STATUS;
//char ssid[] = SECRET_SSID;        // your network SSID (name)
//char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

uint16_t localPort = 30000;      // local port to listen on
IPAddress remoteIp;
bool gotIP = false;
uint16_t happy;
const uint16_t MAX_SEND_LEN = 512;
const uint16_t MAX_SD_WRITE_LEN = 256; // 512 is the size of the sdFat buffer
const uint16_t OUT_MESSAGE_RESERVE_SIZE = 5000;
const uint16_t MAX_INCOMING_PACKET_LEN = 256;
char packetBuffer[MAX_INCOMING_PACKET_LEN]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

uint8_t nUDPSends = 1; // Number of times to send the same packet (e.g. for UDPx3

WiFiUDP Udp;
bool sendUDP = true;
#else // SEND_UDP
bool sendUDP = false;
#endif

uint16_t packetCount = 0;
uint32_t charCount = 0;
String outputMessage;
String receivedMessage;
String consoleMessage;
bool socketReady = false;
bool wifiReady = false;
uint32_t wifiRebootCounter = 0;
uint32_t wifiRebootTarget = 20000;				/*Set to 250 for WiFi debugging, 20000 for Release*/
uint32_t networkBeginStart;
uint32_t WIFI_BEGIN_ATTEMPT_DELAY = 5000;
uint32_t WIFI_BEGIN_SWITCH_CRED = 300000;      //Set to 30000 for debug, 300000 for Release
bool hibernateButtonPressed = false;
uint32_t hibernateButtonStart;
uint32_t hibernateButtonDelay = 2000;
uint32_t hibernateBeginStart;
uint32_t hibernateBeginDelay = 1000;
uint8_t protocolVersion = 1;
String sdCardFilename = "datalog.csv";
uint32_t sendTimerStart;
const uint32_t NORMAL_MIN_SEND_DELAY = 100;
const uint32_t SYNC_MIN_SEND_DELAY = 100;
uint32_t minSendDelay = NORMAL_MIN_SEND_DELAY;
uint32_t requestTimestampTimerStart;
const uint32_t REQUEST_TIMESTAMP_DELAY = 5107; // Milliseconds: choose a prime number to avoid spurocity
int32_t waitingForSyncData = -1;
bool startHibernate = false;
bool stopSDWrite = false;
volatile bool ledOn = false;
volatile bool ledPinBusy = false;
uint8_t defaultDataReliabilityScore = 100;
uint32_t setupTimerStart = 0;
const uint32_t SETUP_TIMEOUT = 61500;          //Enough time to run through list of network credentials twice
bool sendResetPacket = false;

String configFilename = "config.txt";

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE

//#define BOARD_ADAFRUIT_FEATHER_M0
//#define BOARD_ADAFRUIT_FEATHER_NRF52
//#define EMOTIBIT_V01B

// Timer constants
#define TIMER_PRESCALER_DIV 1024
const uint32_t CPU_HZ = 48000000;
const uint32_t SERIAL_BAUD = 2000000; //115200
uint16_t loopCount = 0;

#define BASE_SAMPLING_FREQ 60
#define EDA_SAMPLING_DIV 1
#define TEMPERATURE_SAMPLING_DIV 2
#define BATTERY_SAMPLING_DIV 60
//#define N_DATA_TYPES 17

bool errorStatus = false;

EmotiBit emotibit;
File dataFile;

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

//struct DataStatus {
//	int8_t eda = 0;
//	int8_t tempHumidity = 0;
//	int8_t imu = 0;
//	int8_t ppg = 0;
//} dataStatus;

struct AcquireData {
	bool eda = true;
	bool tempHumidity = true;
	bool imu = true;
	bool ppg = true;
} acquireData;



String typeTags[(uint8_t) EmotiBit::DataType::length];
uint8_t printLen[(uint8_t)EmotiBit::DataType::length];
bool sendData[(uint8_t)EmotiBit::DataType::length];

void setup() {
	Serial.println("setup()");

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	ledOn = true;

	setupTimerStart = millis();

	if (!outputMessage.reserve(OUT_MESSAGE_RESERVE_SIZE)) {
		Serial.println("Failed to reserve memory for output");
		while (true) {
			hibernate();
		}
	}

	delay(500);

	Serial.begin(SERIAL_BAUD);
	//while (!Serial);
	Serial.println("Serial started");

	delay(500);

	emotibit.setSensorTimer(EmotiBit::SensorTimer::MANUAL);
	emotibit.setup(EmotiBit::Version::V01B, 16);
	EmotiBit::SamplingRates samplingRates;
	samplingRates.accelerometer = BASE_SAMPLING_FREQ;
	samplingRates.gyroscope = BASE_SAMPLING_FREQ;
	samplingRates.magnetometer = BASE_SAMPLING_FREQ;
	samplingRates.eda = BASE_SAMPLING_FREQ / EDA_SAMPLING_DIV;
	samplingRates.humidity = BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2;
	samplingRates.temperature = BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2;
	samplingRates.thermistor = BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2;
	emotibit.setSamplingRates(samplingRates);
	EmotiBit::SamplesAveraged samplesAveraged;
	samplesAveraged.eda = BASE_SAMPLING_FREQ / EDA_SAMPLING_DIV / 15;
	samplesAveraged.humidity = (float)BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2 / 7.5f;
	samplesAveraged.temperature = (float)BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2 / 7.5f;
	samplesAveraged.thermistor = (float)BASE_SAMPLING_FREQ / TEMPERATURE_SAMPLING_DIV / 2 / 7.5f;
	emotibit.setSamplesAveraged(samplesAveraged);

	delay(500);

	Serial.print("Initializing SD card...");
	// see if the card is present and can be initialized:
	if (!SD.begin(emotibit._sdCardChipSelectPin)) {
		Serial.print("Card failed, or not present on chip select ");
		Serial.println(emotibit._sdCardChipSelectPin);
		// don't do anything more:
		// ToDo: Handle case where we still want to send network data
		while (true) {
			hibernate();
		}
	}
	Serial.println("card initialized.");
	SD.ls(LS_R);

	Serial.print(F("Loading configuration file: "));
	Serial.println(configFilename);
	if (!loadConfigFile(configFilename)) {
		Serial.println("SD card configuration file parsing failed.");
		Serial.println("Create a file 'config.txt' with the following JSON:");
		Serial.println("{\"WifiCredentials\": [{\"ssid\":\"SSSS\",\"password\" : \"PPPP\"}]}");
		while (true) {
      hibernate();
		}
	}


	//File root = SD.open("/");
	//printDirectory(root, 0);
	//Serial.println("done!");

#if defined(SEND_UDP) || defined(SEND_TCP)
	//Configure pins for Adafruit ATWINC1500 Feather
	WiFi.setPins(8, 7, 4, 2);
	//WiFi.noLowPowerMode();
	WiFi.lowPowerMode();
  
	//WiFi.maxLowPowerMode();
	// check for the presence of the shield:
	if (WiFi.status() == WL_NO_SHIELD) {
		Serial.println("WiFi shield not present");
		// don't continue:
		while (true) {
			hibernate();
		}
	}
#endif

#ifdef SEND_UDP
	// attempt to connect to WiFi network:
	configPos = configSize-1;
	while (wifiStatus != WL_CONNECTED) {
		if (millis() - setupTimerStart > SETUP_TIMEOUT) {
			Serial.println("*********** Setup Timeout **************");
			while (true) {
				hibernate();
			}
		}

		if (configPos == configSize - 1) { configPos = 0; }
		else {
			configPos++;
		}
		
		// Connect to WPA/WPA2 network. Change this line if using open or WEP network:
		Serial.print("Attempting to connect to SSID: ");
		Serial.println(configList[configPos].ssid);
		wifiStatus = WiFi.begin(configList[configPos].ssid, configList[configPos].password);
		// wait for connection:
		Serial.println(wifiStatus);
		
		if (wifiStatus == WL_CONNECTED) {
			break;
		}
		
		delay(1000);
		wifiStatus = WiFi.status();
		Serial.println(wifiStatus);
	}
	//Serial.print("Time2WiFiConnect: ");
	//Serial.println(millis() - setupTimerStart);
	wifiReady = true;
	getMomentLost = true;
	Serial.println("Connected to wifi");
	printWiFiStatus();
	Serial.println("\nStarting connection to server...");
	// if you get a connection, report back via serial:
	Udp.begin(localPort);
	socketReady = true;
	networkBeginStart = millis();
	WiFi.setTimeout(15);			/*Fixes loop delay due to WiFi.begin()*/
#endif

#ifdef SEND_TCP
	// attempt to connect to WiFi network:
	while (wifiStatus != WL_CONNECTED) {
    if (millis() - setupTimerStart > SETUP_TIMEOUT) {
      while(true) {
        hibernate();
      }
    }
		Serial.print("Attempting to connect to SSID: ");
		Serial.println(ssid);
		// Connect to WPA/WPA2 network. Change this line if using open or WEP network:
		wifiStatus = WiFi.begin(ssid, pass);

		// wait 1 second for connection:
		delay(1000);
	}
	wifiReady = true;
	Serial.println("Connected to wifi");
	printWiFiStatus();
	Serial.println(WiFi.gatewayIP());
	Serial.println(server);
	Serial.println("\nStarting connection to server...");
	// if you get a connection, report back via serial:
	while (!client.connect(server, 11999)) {
    if (millis() - setupTimerStart > SETUP_TIMEOUT) {
      while(true) {
        hibernate();
      }
    }
		Serial.println(server);
		delay(100);
	}
	socketReady = true;
	Serial.println("connected to server");
	networkBeginStart = millis();
#endif

	// Initialise I2C communication as MASTER
	//Wire.begin();



	typeTags[(uint8_t)EmotiBit::DataType::EDA] = "EA";
	typeTags[(uint8_t)EmotiBit::DataType::EDL] = "EL";
	typeTags[(uint8_t)EmotiBit::DataType::EDR] = "ER";
	typeTags[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = "PI";
	typeTags[(uint8_t)EmotiBit::DataType::PPG_RED] = "PR";
	typeTags[(uint8_t)EmotiBit::DataType::PPG_GREEN] = "PG";
	typeTags[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = "T0";
	typeTags[(uint8_t)EmotiBit::DataType::TEMPERATURE_HP0] = "TH";
	typeTags[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = "H0";
	typeTags[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = "AX";
	typeTags[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = "AY";
	typeTags[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = "AZ";
	typeTags[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = "GX";
	typeTags[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = "GY";
	typeTags[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = "GZ";
	typeTags[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = "MX";
	typeTags[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = "MY";
	typeTags[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = "MZ";
	typeTags[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = "MZ";
	typeTags[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = "BV";
	typeTags[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = "B%";
	typeTags[(uint8_t)EmotiBit::DataType::DATA_CLIPPING] = "DC";
	typeTags[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW] = "DO";
	//typeTags[(uint8_t)EmotiBit::DataType::PUSH_WHILE_GETTING] = "PW";

	printLen[(uint8_t)EmotiBit::DataType::EDA] = 6;
	printLen[(uint8_t)EmotiBit::DataType::EDL] = 6;
	printLen[(uint8_t)EmotiBit::DataType::EDR] = 6;
	printLen[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = 0;
	printLen[(uint8_t)EmotiBit::DataType::PPG_RED] = 0;
	printLen[(uint8_t)EmotiBit::DataType::PPG_GREEN] = 0;
	printLen[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = 3;
	printLen[(uint8_t)EmotiBit::DataType::TEMPERATURE_HP0] = 0;
	printLen[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = 3;
	printLen[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = 3;
	printLen[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = 3;
	printLen[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = 3;
	printLen[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = 3;
	printLen[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = 3;
	printLen[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = 3;
	printLen[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = 0;
	printLen[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = 0;
	printLen[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = 0;
	printLen[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = 2;
	printLen[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = 0;

	sendData[(uint8_t)EmotiBit::DataType::EDA] = true;
	sendData[(uint8_t)EmotiBit::DataType::EDL] = true;
	sendData[(uint8_t)EmotiBit::DataType::EDR] = true;
	sendData[(uint8_t)EmotiBit::DataType::PPG_INFRARED] = true;
	sendData[(uint8_t)EmotiBit::DataType::PPG_RED] = true;
	sendData[(uint8_t)EmotiBit::DataType::PPG_GREEN] = true;
	sendData[(uint8_t)EmotiBit::DataType::TEMPERATURE_0] = true;
	sendData[(uint8_t)EmotiBit::DataType::TEMPERATURE_HP0] = true;
	sendData[(uint8_t)EmotiBit::DataType::HUMIDITY_0] = true;
	sendData[(uint8_t)EmotiBit::DataType::ACCELEROMETER_X] = true;
	sendData[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Y] = true;
	sendData[(uint8_t)EmotiBit::DataType::ACCELEROMETER_Z] = true;
	sendData[(uint8_t)EmotiBit::DataType::GYROSCOPE_X] = true;
	sendData[(uint8_t)EmotiBit::DataType::GYROSCOPE_Y] = true;
	sendData[(uint8_t)EmotiBit::DataType::GYROSCOPE_Z] = true;
	sendData[(uint8_t)EmotiBit::DataType::MAGNETOMETER_X] = true;
	sendData[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Y] = true;
	sendData[(uint8_t)EmotiBit::DataType::MAGNETOMETER_Z] = true;
	sendData[(uint8_t)EmotiBit::DataType::BATTERY_VOLTAGE] = true;
	sendData[(uint8_t)EmotiBit::DataType::BATTERY_PERCENT] = true;
	sendData[(uint8_t)EmotiBit::DataType::DATA_CLIPPING] = true;
	sendData[(uint8_t)EmotiBit::DataType::DATA_OVERFLOW] = true;

	sendTimerStart = millis();
	requestTimestampTimerStart = millis();


	//Serial.println("Free Ram :" + String(FreeRam(), DEC) + " bytes");
	Serial.println("Setup complete");
	Serial.println("Starting interrupts");
	startTimer(BASE_SAMPLING_FREQ);

	// ToDo: utilize separate strings for SD card writable messages vs transmit acks

	sendResetPacket = true;
}

String createPacketHeader(uint32_t timestamp, String typeTag, size_t dataLen) {
	static String header;
	header = "";
	header += timestamp;
	header += ",";
	header += packetCount;
	header += ",";
	header += dataLen;
	header += ",";
	header += typeTag;
	header += ",";
	header += protocolVersion;
	header += ",";
	header += defaultDataReliabilityScore;
	//createPacketHeader(tempHeader, timestamp, typeTag, dataLen);
	return header;
}


//bool createPacketHeader(String &header, uint32_t timestamp, String typeTag, size_t dataLen) {
//	header += timestamp;
//	header += ",";
//	header += packetCount;
//	header += ",";
//	header += dataLen;
//	header += ",";
//	header += typeTag;
//	header += ",";
//	header += protocolVersion;
//	header += ",";
//	header += defaultDataReliabilityScore;
//	return true;
//}


bool addPacket(uint32_t timestamp, EmotiBit::DataType t, float * data, size_t dataLen, uint8_t precision = 4) {
#ifdef DEBUG_GET_DATA
	Serial.print("addPacket: ");
	Serial.println(typeTag);
#endif // DEBUGs

	if (dataLen > 0) {
		// ToDo: Consider faster ways to populate the outputMessage
		outputMessage += "\n";
		outputMessage += createPacketHeader( timestamp, typeTags[(uint8_t)t], dataLen);

		for (uint16_t i = 0; i < dataLen; i++) {
			outputMessage += ",";
			if (t == EmotiBit::DataType::DATA_CLIPPING || t == EmotiBit::DataType::DATA_OVERFLOW) {
				// If it's a clipping/overflow type, write the data as a string rather than float
				// ToDo: consider just storing the data as a string instead of a float to avoid steps
				outputMessage += typeTags[(uint8_t)data[i]];
			}
			else {
				outputMessage += String(data[i], precision);
			}
		}

		if (sendConsole && !sendSerial) {
			consoleMessage = "";
			consoleMessage += packetCount;
			consoleMessage += ",";
			consoleMessage += dataLen;
			Serial.println(consoleMessage);
		}
		else {
			//Serial.print("*");
		}
		packetCount++;
		loopCount = 0;
		return true;
	}
	return false;
}

bool addPacket(EmotiBit::DataType t) {
#ifdef DEBUG_GET_DATA
	Serial.print("addPacket: ");
	Serial.println((uint8_t) t);
#endif // DEBUG
	float * data;
	uint32_t timestamp;
	size_t dataLen;

	dataLen = emotibit.getData(t, &data, &timestamp);

	if (sendData[(uint8_t)t]) {
		return addPacket(timestamp, t, data, dataLen, printLen[(uint8_t)t]);
	}
	return false;
}

void parseIncomingMessages() {
	// Handle incoming messages
	if (socketReady) {
#ifdef DEBUG_GET_DATA
		Serial.println("parseIncomingMessages()");
#endif // DEBUG
#ifdef SEND_UDP
		// if there's data available, read a packet
		int packetSize = Udp.parsePacket();
		if (packetSize) {
			//Serial.print("\nIncoming packet size: ");
			//Serial.println(packetSize);

			// read the packet into packetBufffer
			int len = Udp.read(packetBuffer, MAX_INCOMING_PACKET_LEN);
			if (len > 0) {
				packetBuffer[len] = '\0';
				receivedMessage = String(packetBuffer);
				//Serial.println(receivedMessage);

				// Log these messages to the SD card (and send back in a return packet)
				static uint16_t recPacketCount;
				static String typeTag;
				static uint16_t dataLength;
				static uint16_t commaN;
				static uint16_t commaN1;
				static uint16_t dataStartChar;
				static uint16_t dataEndChar;
				// timestamp
				commaN = 0;
				commaN1 = receivedMessage.indexOf(',', commaN);
				// packetCount
				commaN = commaN1 + 1;
				commaN1 = receivedMessage.indexOf(',', commaN);
				recPacketCount = receivedMessage.substring(commaN, commaN1).toInt();
				// dataLength
				commaN = commaN1 + 1;
				commaN1 = receivedMessage.indexOf(',', commaN);
				dataLength = receivedMessage.substring(commaN, commaN1).toInt();
				// typeTag
				commaN = commaN1 + 1;
				commaN1 = receivedMessage.indexOf(',', commaN);
				typeTag = receivedMessage.substring(commaN, commaN1);
				// protocolVersion
				commaN = commaN1 + 1;
				commaN1 = receivedMessage.indexOf(',', commaN);
				// reliability
				commaN = commaN1 + 1;
				commaN1 = receivedMessage.indexOf(',', commaN);
				// data payload
				dataStartChar = commaN1 + 1;
				//dataEndChar = receivedMessage.length() - 1;

				bool isLoggableMessage = false;
				bool isAckableMessage = false;
				if (typeTag.equals("HE")) { // ACK
					if (!gotIP) {
						remoteIp = Udp.remoteIP();
						gotIP = true;
					}
					if (sendResetPacket) {
						String tempMessage;
						tempMessage = createPacketHeader(millis(), "RS", 0);
						Serial.println(tempMessage);
						outputMessage += "\n";
						outputMessage += tempMessage;
						packetCount++;
						sendResetPacket = false;
					}
				}
				else if (typeTag.equals("AK")) { // ACK
					isLoggableMessage = true;
					commaN1 = receivedMessage.indexOf(',', dataStartChar);
					uint16_t ackPacketNumber = receivedMessage.substring(dataStartChar, commaN1).toInt();
					//Serial.println(waitingForSyncData);
					//Serial.println(ackPacketNumber);
					if (waitingForSyncData == ackPacketNumber) {
						waitingForSyncData = -1;
					}
				}
				else if (typeTag.equals("TL")) { // Timestamp Local
					isLoggableMessage = true;
					//isAckableMessage = true;
				}
				else if (typeTag.equals("TU")) { // Timestamp UTC
					isLoggableMessage = true;
				}
				else if (typeTag.equals("TX")) { // Timestamp UTC
					isLoggableMessage = true;
				}
				else if (typeTag.equals("LM")) { // Timestamp UTC
					isLoggableMessage = true;
				}
				else if (typeTag.equals("RB")) { // Recording begin
					stopTimer();
					isLoggableMessage = true;
					String datetimeString = receivedMessage.substring(dataStartChar, receivedMessage.length() - 1);
					// Write the configuration info to json file
					String infoFilename = datetimeString + "_info.json";
					dataFile = SD.open(infoFilename, FILE_WRITE);
					if (dataFile) {
						if (!emotibit.printConfigInfo(dataFile, datetimeString)) {
							Serial.println(F("Failed to write to info file"));
						}
						dataFile.close();
					}
					// Try to open the data file to be sure we can write
					sdCardFilename = datetimeString + ".csv";
					dataFile = SD.open(sdCardFilename, FILE_WRITE);
					if (dataFile) {
						sdWrite = true;
						Serial.print("** Recording Begin: ");
						Serial.print(sdCardFilename);
						Serial.println(" **");
						dataFile.close();
						isAckableMessage = true; // only ACK if we were able to open a file
					}
					else {
						Serial.println("Failed to open data file for writing");
					}
					sendTimerStart = millis();
					requestTimestampTimerStart = millis();
					startTimer(BASE_SAMPLING_FREQ);
				}
				else if (typeTag.equals("RE")) { // Recording end
					isLoggableMessage = true;
					isAckableMessage = true;
					stopSDWrite = true;
					Serial.println("** Recording End **");
				}
				else if (typeTag.equals("UN")) { // User note
					isLoggableMessage = true;
					isAckableMessage = true;
				}
				else if (typeTag.equals("MH")) { // Mode Hibernate
					isLoggableMessage = true;
					isAckableMessage = true;
					startHibernate = true; // hibernate after writing data
					hibernateBeginStart = millis();
				}

				static String tempMessage;
				if (isLoggableMessage) {
					// Create message for logging to sd card
					tempMessage = "\n";
					tempMessage += createPacketHeader(millis(), typeTag, dataLength);
					tempMessage += ',';
					tempMessage += receivedMessage.substring(dataStartChar, receivedMessage.length() - 1);
					//Serial.println("** Logable Message **");
					Serial.print(tempMessage);
					outputMessage += tempMessage;
					packetCount++;
				}
				if (isAckableMessage) {
					// Create ACK message
					// ToDo: utilize separate strings for SD card writable messages vs transmit acks
					tempMessage = "\n";
					tempMessage += createPacketHeader(millis(), "AK", 2);
					tempMessage += ',';
					tempMessage += recPacketCount;
					tempMessage += ',';
					tempMessage += typeTag;
					Serial.print(tempMessage);
					outputMessage += tempMessage;
					packetCount++;
				}
			}
		}
#endif // SEND_UDP
#ifdef SEND_TCP
		String s = "";
		if (client.available()) {
			Serial.println(client.read());
			//s += client.read();
		}
#endif
	}

}

bool performTimestampSyncing() {
	static String tempMessage;
	tempMessage = "\n";
	tempMessage += createPacketHeader(millis(), "RD", 2);
	tempMessage += ',';
	tempMessage += "TL";
	tempMessage += ',';
	tempMessage += "TU";
	sendUdpMessage(tempMessage);
	outputMessage += tempMessage;
	waitingForSyncData = packetCount;
	packetCount++;

	sendTimerStart = millis();
	while (waitingForSyncData > 0 && millis() - sendTimerStart < SYNC_MIN_SEND_DELAY) {
		// Wait to get an timestamp+ACK for a short period
		//Serial.println(millis());
		parseIncomingMessages();
	}
}

void loop() {
#ifdef DEBUG_GET_DATA
	Serial.println("loop()");
#endif // DEBUG

	updateWiFi();

	parseIncomingMessages();


	// Check for hibernate button press
	if (switchRead() == 0) {
		hibernateButtonPressed = false;
	}
	else if (switchRead() == 1) {
		if (hibernateButtonPressed) {
			// hibernate button was already pressed -- check how long
			if (!startHibernate && millis() - hibernateButtonStart > hibernateButtonDelay) {
				// delay exceeded
				startHibernate = true; // hibernate after writing data
				hibernateBeginStart = millis();

				// ToDo: Devise a better communication method than ACK for state changes
				String tempMessage;
				tempMessage = "\n";
				tempMessage += createPacketHeader(millis(), "AK", 2);
				tempMessage += ',';
				tempMessage += "-1";
				tempMessage += ',';
				tempMessage += "MH";
				Serial.println(tempMessage);
				outputMessage += tempMessage;
				packetCount++;
			}
		}
		else {
			// start timer
			hibernateButtonPressed = true;
			hibernateButtonStart = millis();
		}
	}

	// Periodically request a timestamp between data dumps to assess round trip time
	if (millis() - requestTimestampTimerStart > REQUEST_TIMESTAMP_DELAY) {
			requestTimestampTimerStart = millis();
			performTimestampSyncing();
	}
		
	// Send waiting data
	bool newData = false;
	if (millis() - sendTimerStart > NORMAL_MIN_SEND_DELAY) { // add a delay to batch data sending
		sendTimerStart = millis();
		// Create data packets
		for (int16_t i = 0; i < (uint8_t)EmotiBit::DataType::length; i++) {
			newData = addPacket((EmotiBit::DataType) i);
			//if (outputMessage.length() > MAX_SEND_LEN - 100 || i == (uint8_t)EmotiBit::DataType::length - 1) {
			//	// Send batches to avoid using too much memory
			//	sendMessage(outputMessage);
			//	outputMessage = "";
			//}
		}

		sendMessage(outputMessage);
		outputMessage = "";

		// Blink the LED slowly if we're writing to the SD card
		if (sdWrite && !ledPinBusy) {	// only change the LED if it's not busy to avoid crashes
			ledPinBusy = true;
			if (ledOn) {
				pinMode(LED_BUILTIN, INPUT);
				ledOn = false;
			}
			else {
				pinMode(LED_BUILTIN, OUTPUT);
				digitalWrite(LED_BUILTIN, HIGH);
				ledOn = true;
			}
			ledPinBusy = false;
		}
	}

	if (!newData) {
		loopCount++;
	}

	if (startHibernate && millis() - hibernateBeginStart > hibernateBeginDelay) {
		hibernate();
	}

	if (stopSDWrite) {
		sdWrite = false;
		stopSDWrite = false;
	}

	//if (true && sdWrite) {
	//	config.ssid = "garbage";
	//	rebootWiFi(); // debugging wifi issue
	//}
}

void readSensors() {
#ifdef DEBUG_GET_DATA
	Serial.println("readSensors()");
#endif // DEBUG

	// ToDo: Move readSensors and timer into EmotiBit

	// EDA
	if (acquireData.eda) {
		static uint16_t edaCounter;
		if (edaCounter == EDA_SAMPLING_DIV) {
			int8_t tempStatus = emotibit.updateEDAData();
			//if (dataStatus.eda == 0) {
			//	dataStatus.eda = tempStatus;
			//}
			edaCounter = 0;
		}
		edaCounter++;
	}

	// Temperature / Humidity Sensor
	if (acquireData.tempHumidity) {
		static uint16_t temperatureCounter;
		if (temperatureCounter == TEMPERATURE_SAMPLING_DIV) {
			// Note: Temperature/humidity and the thermistor are alternately sampled 
			// on every other call of updateTempHumidityData()
			// I.e. you must call updateTempHumidityData() 2x with a sufficient measurement 
			// delay between calls to sample both temperature/humidity and the thermistor
			int8_t tempStatus = emotibit.updateTempHumidityData();
			//if (dataStatus.tempHumidity == 0) {
			//	dataStatus.tempHumidity = tempStatus;
			//}
			temperatureCounter = 0;

			if (!sdWrite) {	// If we're not recording, blink quickly
				if (!ledPinBusy) {	// only change the LED if it's not busy to avoid crashes
					ledPinBusy = true;
					if (ledOn) {
						pinMode(LED_BUILTIN, INPUT);
						ledOn = false;
					}
					else {
						pinMode(LED_BUILTIN, OUTPUT);
						digitalWrite(LED_BUILTIN, HIGH);
						ledOn = true;
					}
					ledPinBusy = false;
				}
			}
		}
		temperatureCounter++;
	}

	// IMU
	if (acquireData.imu) {
		int8_t tempStatus = emotibit.updateIMUData();
		//if (dataStatus.imu == 0) {
		//	dataStatus.imu = tempStatus;
		//}
	}

	// PPG
	if (acquireData.ppg) {
		int8_t tempStatus = emotibit.updatePPGData();
		//if (dataStatus.ppg == 0) {
		//	dataStatus.ppg = tempStatus;
		//}
	}

	// Battery (all analog reads must be in the ISR)
	static uint16_t batteryCounter;
	if (batteryCounter > BATTERY_SAMPLING_DIV) {
		emotibit.updateBatteryPercentData();
		batteryCounter = 0;
	}
	batteryCounter++;
}

void setTimerFrequency(int frequencyHz) {
	int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
	TcCount16* TC = (TcCount16*)TC3;
	// Make sure the count is in a proportional position to where it was
	// to prevent any jitter or disconnect when changing the compare value.
	TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
	TC->CC[0].reg = compareValue;
	//Serial.println(TC->COUNT.reg);
	//Serial.println(TC->CC[0].reg);
	while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
	REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);
	while (GCLK->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TcCount16* TC = (TcCount16*)TC3;

	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Use the 16-bit timer
	TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Use match mode so that the timer counter resets when the count matches the compare register
	TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Set prescaler to 1024
	TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	setTimerFrequency(frequencyHz);

	// Enable the compare interrupt
	TC->INTENSET.reg = 0;
	TC->INTENSET.bit.MC0 = 1;

	NVIC_EnableIRQ(TC3_IRQn);

	TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void stopTimer() {
	// ToDo: Verify implementation
	TcCount16* TC = (TcCount16*)TC3;
	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
}

void TC3_Handler() {
	TcCount16* TC = (TcCount16*)TC3;
	// If this interrupt is due to the compare register matching the timer count
	// we toggle the LED.
	if (TC->INTFLAG.bit.MC0 == 1) {
		TC->INTFLAG.bit.MC0 = 1;
		//emotibit.scopeTimingTest();
		// Write callback here!!!
		readSensors();

	}
}



void printWiFiStatus() {
#ifdef SEND_UDP || SEND_TCP
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print your WiFi shield's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");
#endif
}

//extern "C" char *sbrk(int i);
//
//int FreeRam() {
//	char stack_dummy = 0;
//	return &stack_dummy - sbrk(0);
//}

void hibernate() {
	stopTimer();

	emotibit.ppgSensor.shutDown();
	if (wifiStatus == WL_CONNECTED) {
		WiFi.disconnect();
	}
	WiFi.end();

	while (ledPinBusy)
	pinMode(LED_BUILTIN, OUTPUT);

	// ToDo: 
	//	Shutdown IMU
	//	Consider more low level power management

	while (true) {
		
		digitalWrite(LED_BUILTIN, LOW); // Show we're asleep
		int sleepMS = Watchdog.sleep();
		digitalWrite(LED_BUILTIN, HIGH); // Show we're awake again
		delay(1);

		//	// Log the battery voltage to the SD card
		//	if (SD.begin(emotibit._sdCardChipSelectPin)) {
		//		dataFile = SD.open("HibernateBatteryLog.csv", FILE_WRITE);
		//		if (dataFile) {
		//			static float data;
		//			static EmotiBit::DataType t = EmotiBit::DataType::BATTERY_VOLTAGE;

		//			data = emotibit.readBatteryVoltage();

		//			static String message;
		//			message = "";
		//			message += createPacketHeader(millis(), typeTags[(uint8_t)t], 1);
		//			message += ",";
		//			message += String(data, printLen[(uint8_t)t]);
		//			message += "\n";
		//			packetCount++;
		//			dataFile.print(message);
		//			dataFile.close();
		//		}
		//}

		// Try to reattach USB connection on "native USB" boards (connection is
		// lost on sleep). Host will also need to reattach to the Serial monitor.
		// Seems not entirely reliable, hence the LED indicator fallback.
//#ifdef USBCON
//		USBDevice.attach();
//#endif
	}

	//sleepmgr_sleep(SLEEPMGR_STANDBY);

	//// Set sleep to full power down.  Only external interrupts or
	//// the watchdog timer can wake the CPU!
	//	set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
	//	power_all_disable();	// disable all functions
	//	sleep_bod_disable(); // disable brown out detect to lower power consumption
	//	// Enable sleep and enter sleep mode.
	//	sleep_mode();


}

int8_t switchRead() {
	// ToDo: Consider reading pin mode https://arduino.stackexchange.com/questions/13165/how-to-read-pinmode-for-digital-pin

	if (LED_BUILTIN == emotibit.switchPin) {
		if (ledPinBusy) {
			return -1;
		}
		else {
			ledPinBusy = true;
			pinMode(LED_BUILTIN, INPUT);
			int8_t switchState = (int8_t) digitalRead(emotibit.switchPin);
			if (ledOn) {
				pinMode(LED_BUILTIN, OUTPUT);
				digitalWrite(LED_BUILTIN, HIGH);
			}
			ledPinBusy = false;
			return switchState;
		}
	}
	else {
		pinMode(LED_BUILTIN, INPUT);
		return (int8_t) digitalRead(emotibit.switchPin);
	}
}

// Loads the configuration from a file
bool loadConfigFile(String filename) {
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
	StaticJsonDocument<1024> jsonDoc;

	// Parse the root object
	DeserializationError err = deserializeJson(jsonDoc,file);

	if (err) {
		Serial.print(F("deserializeJson() failed with code "));
		Serial.println(err.c_str());
		return false;
	}

	// Copy values from the JsonDocument to EmotiBitConfig
	// https://arduinojson.org/v6/doc/deserialization/
	// ssid and password default can be set by = jsonDoc[i] | "Default", but string already defaults to ""
	//Explicit casting here is unneccessary, but generally safer
	configSize = jsonDoc["WifiCredentials"].size(); 
	Serial.print("ConfigSize: ");
	Serial.println(configSize);
	for (size_t i = 0; i < configSize; i++) {
		configList[i].ssid = jsonDoc["WifiCredentials"][i]["ssid"].as<String>();
		configList[i].password = jsonDoc["WifiCredentials"][i]["password"].as<String>();
		Serial.println(configList[i].ssid);
		Serial.println(configList[i].password);
	}

	//strlcpy(config.hostname,                   // <- destination
	//	root["hostname"] | "example.com",  // <- source
	//	sizeof(config.hostname));          // <- destination's capacity

	// Close the file (File's destructor doesn't close the file)

	file.close();
	return true;
}

bool sendSdCardMessage(String & s) {
	// Break up the message in to bite-size chunks to avoid over running the UDP or SD card write buffers
	// UDP buffer seems to be about 1400 char. SD card buffer seems to be about 2500 char.
	if (sdWrite && s.length() > 0) {
		dataFile = SD.open(sdCardFilename, FILE_WRITE);
		if (dataFile) {
			static int16_t firstIndex;
			firstIndex = 0;
			while (firstIndex < s.length()) {
				static int16_t lastIndex;
				if (s.length() - firstIndex > MAX_SD_WRITE_LEN) {
					lastIndex = firstIndex + MAX_SD_WRITE_LEN;
				}
				else {
					lastIndex = s.length();
				}

				////Serial.println(firstIndex);
				//static int16_t lastIndex;
				//lastIndex = s.length();
				//while (lastIndex - firstIndex > MAX_SD_WRITE_LEN) {
				//	lastIndex = s.lastIndexOf('\n', lastIndex - 1);
				//	//Serial.println(lastIndex);
				//}
				////Serial.println(outputMessage.substring(firstIndex, lastIndex));

#ifdef DEBUG_GET_DATA
				Serial.println("writing to SD card");
#endif // DEBUG
				//sdCardFilename = "DATALOGLOGLOG.CSV";

				dataFile.print(s.substring(firstIndex, lastIndex));
				firstIndex = lastIndex;

				//dataFile.flush();
				//firstIndex = lastIndex + 1;	// increment substring indexes for breaking up sends
			}
			dataFile.close();
		}
		else {
			Serial.print("Data file didn't open properly: ");
			Serial.println(sdCardFilename);
			sdWrite = false;
		}
	}
}

bool sendUdpMessage(String & s) {
	if (wifiReady && socketReady) {

		// Break up the message in to bite-size chunks to avoid over running the UDP or SD card write buffers
		// UDP buffer seems to be about 1400 char. SD card buffer seems to be about 2500 char.
		static int16_t firstIndex;
		firstIndex = 0;
		while (firstIndex < s.length()) {
			//Serial.println(firstIndex);
			static int16_t lastIndex;
			lastIndex = s.length();
			while (lastIndex - firstIndex > MAX_SEND_LEN) {
				lastIndex = s.lastIndexOf('\n', lastIndex - 1);
				//Serial.println(lastIndex);
			}
			//Serial.println(outputMessage.substring(firstIndex, lastIndex));

		// UDP sending
			if (socketReady && gotIP) {
#ifdef DEBUG_GET_DATA
				Serial.println("sending UDP");
#endif // DEBUG
				for (uint8_t n = 0; n < nUDPSends; n++) {
					//Udp.beginPacket(remoteIp, localPort);
					Udp.beginPacket(remoteIp, Udp.remotePort());
					Udp.print(s.substring(firstIndex, lastIndex));
					Udp.endPacket();
					wifiRebootCounter++;
				}
			}
			firstIndex = lastIndex + 1;	// increment substring indexes for breaking up sends
		}
	}
}

bool sendMessage(String & s) {

		// Serial sending
		if (sendSerial) {
#ifdef DEBUG_GET_DATA
			Serial.println("sending serial");
#endif // DEBUG
			Serial.print(s);
		}

#ifdef SEND_TCP
		if (socketReady) {
#ifdef DEBUG_GET_DATA
			// ToDo: Determine if TCP packets need to be broken up similarly to UDP
			Serial.println("sending TCP");
#endif // DEBUG
			client.print(s.substring(firstIndex, lastIndex));
		}
#endif // SEND_UDP
#ifdef SEND_UDP
		sendUdpMessage(s);
#endif // SEND_UDP	
		// Write to SD card after sending network messages
		// This give the receiving computer time to process in advance of a sync exchange
		sendSdCardMessage(s);  
}

void updateWiFi() {
	wifiStatus = WiFi.status();
	if (wifiStatus != WL_CONNECTED) {
		wifiReady = false;
		socketReady = false;
	}
#if 1
	//Serial.println("<<<<<<< updateWiFi >>>>>>>");
	Serial.println("------- WiFi Status -------");
	Serial.println(millis());
	Serial.println(wifiStatus);
	//Serial.println(wifiRebootCounter);    /* Uncommment for WiFi Debugging*/
	Serial.println(millis());
	Serial.println("-------  -------");
#endif

	// Handle Wifi Reboot
	if (wifiReady && wifiRebootCounter > wifiRebootTarget) {
		rebootWiFi();
	}

	// Handle WiFi Reconnects
	if (!wifiReady) {
		if (getMomentLost) {
			momentLost = millis();
			getMomentLost = false;
		}

		if (millis() - networkBeginStart > WIFI_BEGIN_ATTEMPT_DELAY) {
				if ((millis() - momentLost > WIFI_BEGIN_SWITCH_CRED) && (attempts >= 2)) {
					switchCred = true;
					gotIP = false;
					attempts = 0;
				}
				else { switchCred = false; }

				if (switchCred && (configPos != configSize - 1)) { configPos++; }
				else if (switchCred && (configPos == configSize - 1)) { configPos = 0; }
				Serial.println("<<<<<<< Wifi begin >>>>>>>");
				Serial.println(millis());
				Serial.println(WIFI_BEGIN_ATTEMPT_DELAY);
				Serial.println(wifiRebootCounter);
				Serial.println(wifiRebootTarget);
				//Serial.println(momentLost);               //uncomment for debugging
				Serial.println(configList[configPos].ssid);
				wifiStatus = WiFi.begin(configList[configPos].ssid, configList[configPos].password);
				attempts++;
				networkBeginStart = millis();
				Serial.println(networkBeginStart);
				Serial.println("<<<<<<<  >>>>>>>");
		}
		if (wifiStatus == WL_CONNECTED) {
			wifiReady = true;
			getMomentLost = true;
			Serial.println(">>>>>>> Connected to wifi <<<<<<<");
			printWiFiStatus();
			wifiRebootCounter = 0;
			//For case where begin works immediately in this loop, but disconnects again within
			// the attempt delay. Without changing nBS, the code is blocked from attempting to reconnect
			networkBeginStart = millis() - WIFI_BEGIN_ATTEMPT_DELAY - 1;
		}
	}
	else {
		if (!socketReady) {
#ifdef SEND_TCP
			if (millis() - networkBeginStart > WIFI_BEGIN_ATTEMPT_DELAY / 2) {
				Serial.println("/////// Client begin ///////");
				Serial.println(millis());
				Serial.println(networkBeginStart);
				Serial.println(WIFI_BEGIN_ATTEMPT_DELAY);
				Serial.println(wifiRebootCounter);
				Serial.println(wifiRebootTarget);
				Serial.println("///////  ///////");
				socketReady = client.connect(server, 11999);
				networkBeginStart = millis();
			}
			if (socketReady) {
				Serial.print("connected to server: ");
				Serial.println(server);
			}
#endif // SEND_TCP
#ifdef SEND_UDP
			Udp.begin(localPort);
			socketReady = true;
#endif // SEND_UDP
		}
	}

}

void rebootWiFi() {
	Serial.println("******* WiFi reboot *******");
	Serial.println(wifiRebootCounter);
	Serial.println(wifiRebootTarget);
	Serial.println("*******  *******");
	wifiReady = false;
	socketReady = false;
#ifdef SEND_UDP
	Udp.stop();
#endif
#ifdef SEND_TCP
	client.stop();
#endif
	WiFi.disconnect();
	wifiStatus = WL_IDLE_STATUS;
	networkBeginStart = millis() - WIFI_BEGIN_ATTEMPT_DELAY - 1; // set to immediately reconnect
	wifiRebootCounter = 0;
}

int listNetworks() {
	// scan for nearby networks:
	Serial.println("xxxxxxx Scan Networks xxxxxxx");
	Serial.println(millis());
	int numSsid = WiFi.scanNetworks();
	if (numSsid == -1)
	{
		Serial.println("Couldn't get a wifi connection");
		//while (true);
	}
	Serial.println(millis());

	// print the list of networks seen:
	Serial.print("number of available networks:");
	Serial.println(numSsid);

	// print the network number and name for each network found:
	for (int thisNet = 0; thisNet < numSsid; thisNet++) {
		Serial.print(thisNet);
		Serial.print(") ");
		Serial.print(WiFi.SSID(thisNet));
		Serial.print("\tSignal: ");
		Serial.print(WiFi.RSSI(thisNet));
		Serial.print(" dBm");
		Serial.print(" \n");
		//Serial.print("\tEncryption: ");
		//printEncryptionType(WiFi.encryptionType(thisNet));
		//Serial.flush();
	}
	Serial.println(millis());
	Serial.println("xxxxxxx  xxxxxxx");

	return numSsid;
}