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
#include <ArduinoLowPower.h>

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
const uint16_t MAX_SD_WRITE_LEN = 512; // 512 is the size of the sdFat buffer
const uint16_t OUT_MESSAGE_RESERVE_SIZE = 4096;
uint16_t OUT_PACKET_MAX_SIZE = 1024;
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
uint32_t wifiRebootTarget = 2000000;				/*Set to 250 for WiFi debugging, 20000 for Release*/
uint32_t networkBeginStart;
uint32_t WIFI_BEGIN_ATTEMPT_DELAY = 5000;
uint32_t WIFI_BEGIN_SWITCH_CRED = 300000;      //Set to 30000 for debug, 300000 for Release
bool hibernateButtonPressed = false;
uint32_t hibernateButtonStart = millis();
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
uint32_t recordBlinkDuration = millis();
bool recordLedStatus = false;
bool UDPtxLed = false;
bool battLed = false;
uint32_t BattLedstatusChangeTime = millis();
uint8_t battLevel = 100;
uint8_t battIndicationSeq = 0;
uint8_t BattLedDuration = INT_MAX;
uint8_t wifiState = 0; // 0 for normal operation
bool attachWifiControlToShortPress = false;
bool attachHibernateToLongPress = false;


//TODO: find a better way to Debug ::DBTAG1
uint8_t debugWifiRecord[40]; // = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// DBTAG1
bool connected = false;// used in updateWifi
uint32_t start_udpSend = 0;// used in sendMessage
uint32_t duration_udpSend = 0;//used in sendMessage
uint32_t start_sdcardSend = 0;// used in sendMessage
uint32_t duration_sdcardSend = 0;// used in sendMessage
const uint8_t MAX_WIFI_CONNECT_HISTORY = 20; // NO. of wifi connectivity status to remember
const uint8_t MAX_SD_WRITE_TIMES = 10;
uint32_t duration_timeWriteFile[MAX_SD_WRITE_TIMES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t duration_timeFileClose = 0;
uint32_t duration_timeOpenFile = 0;
bool sent_FOPEN = false;
bool sent_FCLOSE = false;
uint32_t duration_timeFileSync = 0;
bool fileOpened = false;
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

#define BASE_SAMPLING_FREQ 300
#define IMU_SAMPLING_DIV 3
#define PPG_SAMPLING_DIV 3
#define EDA_SAMPLING_DIV 1
#define TEMPERATURE_SAMPLING_DIV 10
#define BATTERY_SAMPLING_DIV 50


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
/*
Function called when short press is detected in main loop.
calls the function attached to it by the attachToShortButtonPress()
*/
void(*onShortPress)(void){};
/*
Function called when long press is detected in main loop.
calls the function attached to it by the attachToLongButtonPress()
*/
void(*onLongPress)(void){};

/*
Function to attch callback to short press
*/
void attachToShortButtonPress(void(&shortSwitchPressFunction)(void)) {
	onShortPress = &shortSwitchPressFunction;

}
/*
Function to attch callback to short press
*/
void attachToLongButtonPress(void(&longSwitchPressFunction)(void)) {
	onLongPress = &longSwitchPressFunction;
}

void setup() {
	//if (0) {
	//	SD.cardBegin(19, SD_SCK_MHZ(50));
	//	cardSize = SD.card()->cardSize();
	//	cardSize_f = 0.000512*cardSize;
	//	start_time = millis();
	//	SD.fsBegin();
	//	volFree = SD.vol()->freeClusterCount();
	//	fs = 0.000512*volFree*SD.vol()->blocksPerCluster();
	
	//}
	// DBTAG
	for (uint8_t i = 0; i < 40; i++)
		debugWifiRecord[i] = 0;
	Serial.println("setup()");
	// DBTAG1
	// }

	setupTimerStart = millis();

	Serial.begin(SERIAL_BAUD);
	//while (!Serial);
	Serial.println("Serial started");

	delay(500);

	Serial.println("setup()");

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


	if (!outputMessage.reserve(OUT_MESSAGE_RESERVE_SIZE)) {
		Serial.println("Failed to reserve memory for output");
		while (true) {
			hibernate();
		}
	}

	delay(500);

	emotibit.setSensorTimer(EmotiBit::SensorTimer::MANUAL);
	emotibit.setup(EmotiBit::Version::V01C);
	EmotiBit::SamplingRates samplingRates;
	samplingRates.accelerometer = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
	samplingRates.gyroscope = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
	samplingRates.magnetometer = BASE_SAMPLING_FREQ / IMU_SAMPLING_DIV;
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
	samplesAveraged.battery = BASE_SAMPLING_FREQ / BATTERY_SAMPLING_DIV / 1;
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


	Serial.println("Free Ram :" + String(freeMemory(), DEC) + " bytes");
	Serial.println("Setup complete");
	Serial.println("Starting interrupts");
	startTimer(BASE_SAMPLING_FREQ);

	// ToDo: utilize separate strings for SD card writable messages vs transmit acks

	sendResetPacket = true;
	//assignButtonCallbacks((uint8_t)EmotiBit::FunctionalityShortPress::WIFI, (uint8_t)EmotiBit::FunctionalityLongPress::HIBERNATE);
	attachToShortButtonPress(*wifiModeControl);
	attachToLongButtonPress(*initHibernate);
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
//DBTAG1
void addDebugPacket(uint8_t type, uint32_t timestamp)
{
	switch (type){
		case (uint8_t)EmotiBit::DebugTags::WIFI_CONNHISTORY:
			/*
			if a DO occurs, create a packet to specify previous WiFi states
			*/
			outputMessage += createPacketHeader( timestamp, "DB", MAX_WIFI_CONNECT_HISTORY + 1);
			outputMessage += ",WFCONNHIS";
			for (uint8_t i = 0; i < MAX_WIFI_CONNECT_HISTORY; i++){
				outputMessage += ",";
				outputMessage += String(debugWifiRecord[i]);
			}
			outputMessage += "\n";
			break;
		case (uint8_t)EmotiBit::DebugTags::WIFI_DISCONNECT:
			/*
			If Wifi Disconnects. Called from Wifiupdate()
			timestamp = Moment Wifi Disconnects
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( timestamp, "DB", 1);
			outputMessage += ",WFDCON";
			break;
		case (uint8_t)EmotiBit::DebugTags::WIFI_TIMELOSTCONN:
			/*
			To record the time in miilis of Wifi Signal Lost
			timestamp = momentLost
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 1);
			outputMessage += ",WFLostTime";
			break;
		case (uint8_t)EmotiBit::DebugTags::WIFI_CONNECT:
			/*
			When Wifi connects, record the new connection established
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( timestamp, "DB", 1);
			outputMessage += ",WFCON";
			break;
		case (uint8_t)EmotiBit::DebugTags::WIFI_UPDATECONNRECORDTIME:
			/*
			UPDATE: no longer in use. function call commented in code.
			Print the time taken to adjust the buffer of wifi connectivity record.
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 1);
			outputMessage += ",Time taken for Wifi record update: ";
			outputMessage += String(timestamp);
			break;
		case (uint8_t)EmotiBit::DebugTags::TIME_PARSEINCOMINGMSG:
			/*
			To print the time taken for parseIncoingMessage from main loop
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 1);
			outputMessage += ",Time taken for parseIncomingMessages: ";
			outputMessage += String(timestamp);
			break;
		case (uint8_t)EmotiBit::DebugTags::TIME_TIMESTAMPSYNC:
			/*
			UPDATE: no longer in use. function call commented in code.
			To print the time taken for time syncing if it occurs
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 1);
			outputMessage += ",Time taken for time stamp syncing(involves call to parseIncomingMessages): ";
			outputMessage += String(timestamp);
			break;
		case (uint8_t)EmotiBit::DebugTags::TIME_MSGGENERATION:
			/*
			To print the time taken for time syncing if it occurs
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 2);
			outputMessage += ",MGEN,";// Message generation
			outputMessage += String(timestamp);
			break;

		case (uint8_t)EmotiBit::DebugTags::TIME_MSGTX:
			/*
			UPDATE: no longer in use. Commented function call
			To print the time taken for transmitting last message
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 2);
			outputMessage += ",MTX:";// Message Transmission
			outputMessage += String(timestamp);
			break;
		case (uint8_t)EmotiBit::DebugTags::TIME_UDPTX:
			/*
			To print time taken for udp tx of last message
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 2);
			outputMessage += ",udpTX,";// Message Transmission
			outputMessage += String(timestamp);
			break;
		case (uint8_t)EmotiBit::DebugTags::TIME_SDCARDTX:
			/*
			To print time taken for sdcardd tx of last message
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 2);
			outputMessage += ",sdTX,";
			outputMessage += String(timestamp);
			break;

		case (uint8_t)EmotiBit::DebugTags::TIME_FILEOPEN:
			/*
			To print time taken for file open
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 2);
			outputMessage += ",FOPEN,";
			outputMessage += String(timestamp);
			break;

		case (uint8_t)EmotiBit::DebugTags::TIME_FILEWRITES:
			/*
			To print time taken for file write
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 11);
			outputMessage += ",FWRITES";
			for (uint8_t i =0; i < MAX_SD_WRITE_TIMES; i++){
				outputMessage += ",";
				outputMessage += String(duration_timeWriteFile[i]);
			}
			break;

		case (uint8_t)EmotiBit::DebugTags::TIME_FILECLOSE:
			/*
			To print time taken for file close
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 2);
			outputMessage += ",FCLOSE,";
			outputMessage += String(timestamp);
			break;

		case (uint8_t)EmotiBit::DebugTags::TIME_FILESYNC:
			/*
			To print time taken for file sync
			*/
			outputMessage += "\n";
			outputMessage += createPacketHeader( millis(), "DB", 2);
			outputMessage += ",FSYNC,";
			outputMessage += String(timestamp);
			break;

	}
}

bool addPacket(uint32_t timestamp, EmotiBit::DataType t, float * data, size_t dataLen, uint8_t precision = 4) {
#ifdef DEBUG_GET_DATA
	Serial.print("addPacket: ");
	Serial.println(typeTag);
#endif // DEBUGs

	if (dataLen > 0) {
		// ToDo: Consider faster ways to populate the outputMessage
		outputMessage += "\n";
		//DBTAG

		if (t == EmotiBit::DataType::DATA_OVERFLOW){
			addDebugPacket((uint8_t)EmotiBit::DebugTags::WIFI_CONNHISTORY, timestamp);  // addDebugPacket(case, timestamp) 
		}
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
					Serial.println("Creating new file to write data");
					// Try to open the data file to be sure we can write
					sdCardFilename = datetimeString + ".csv";
					uint32_t start_timeOpenFile = millis();
					dataFile = SD.open(sdCardFilename, O_CREAT | O_WRITE | O_AT_END);
					duration_timeOpenFile = millis() - start_timeOpenFile;
					if (dataFile) {
						sdWrite = true;
						Serial.print("** Recording Begin: ");
						Serial.print(sdCardFilename);
						Serial.println(" **");
						// dataFile.close();
						isAckableMessage = true; // only ACK if we were able to open a file
						//DBTAG1
						fileOpened = true;
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

					if(dataFile){
						uint32_t start_timeFileClose = millis();
						dataFile.close();
						duration_timeFileClose = millis() - start_timeFileClose;
						//DBTAG1
						fileOpened = false;
					}
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
#ifdef SEND_UDP
	sendUdpMessage(tempMessage);
#endif
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
	//DBTAG1
	uint32_t start_timeSendMessage = 0;
	uint32_t start_getdata;
	updateWiFi();
	// DBTAG1
	// uint32_t start_WifiRecordAdjust = millis();
	for (uint8_t i=0;i<MAX_WIFI_CONNECT_HISTORY - 1;i++){
		debugWifiRecord[i] = debugWifiRecord[i+1];
	}
	debugWifiRecord[MAX_WIFI_CONNECT_HISTORY - 1] = (uint8_t)WiFi.status();
	// uint32_t duration_WifiRecordAdjust = millis() - start_WifiRecordAdjust;
	// if ( wifiRebootCounter % 500== 0)
	// 	addDebugPacket((uint8_t)EmotiBit::DebugTags::WIFI_UPDATECONNRECORDTIME, duration_WifiRecordAdjust);  // To record the time taken for adjusting the array of wifi status 
	//DBTAG1
	uint32_t start_timeparseIncomingMessage = millis();
	parseIncomingMessages();
	//DBTAG1
	if (millis() - start_timeparseIncomingMessage > 100){
		addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_PARSEINCOMINGMSG,millis() - start_timeparseIncomingMessage);
	}
	
	// Check for hibernate button press
	if (switchRead() == 0) {
		hibernateButtonPressed = false;
	}
	// TODO: When a switch debouncer is added, remove the 500ms delay in polling
	else if (switchRead() == 1 && millis() - hibernateButtonStart > 500) { // poll only after 500mSec, until aswitch  debouncer is added
		if (hibernateButtonPressed) {
			// Long Press
			// hibernate button was already pressed -- check how long
			onLongPress();
		}
		else {
			// start timer for hibernate
			hibernateButtonPressed = true;
			hibernateButtonStart = millis();
			onShortPress();
		}
	}
	//DBTAG1
	uint32_t start_timestampSync = millis();
#if defined(SEND_UDP) || defined(SEND_TCP)
	// Periodically request a timestamp between data dumps to assess round trip time
	if (millis() - requestTimestampTimerStart > REQUEST_TIMESTAMP_DELAY) {
			requestTimestampTimerStart = millis();
			performTimestampSyncing();
			////DBTAG1
			//if (millis() - start_timestampSync > 100){
			//addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_TIMESTAMPSYNC, millis() - start_timestampSync); // To record time taken for time stamp syncing
			//}
	}
#endif	
	// Send waiting data
	bool newData = false;
	if (millis() - sendTimerStart > NORMAL_MIN_SEND_DELAY) { // add a delay to batch data sending
		sendTimerStart = millis();
		// Create data packets
		for (int16_t i = 0; i < (uint8_t)EmotiBit::DataType::length; i++) {
			//DBTAG1
			if (i == 1){
				start_getdata = millis();
			}
			
			newData = addPacket((EmotiBit::DataType) i);
			// DBTAG1
			if(i == (uint16_t)EmotiBit::DataType::length - 1)
				addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_MSGGENERATION, millis() - start_getdata);
			//DBTAG1
			if (outputMessage.length() > OUT_MESSAGE_RESERVE_SIZE - OUT_PACKET_MAX_SIZE) {
				// Send batches to avoid using too much memory
				addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_MSGGENERATION, millis() - start_getdata);
				// start_timeSendMessage = millis(); // commented to stop printing the total TX time
				sendMessage(outputMessage);
				outputMessage = "";

				// addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILEOPEN, duration_timeOpenFile);
				if (fileOpened){
					addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILEWRITES, 0); // to add the file write times
					addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILESYNC, duration_timeFileSync); // to add the file sync time
				}
				// addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILEWRITES,0); // to add the file write times
				// addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILECLOSE, duration_timeFileClose);
				addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_UDPTX, duration_udpSend);
				addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_SDCARDTX, duration_sdcardSend);
				// addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_MSGTX,TIME_MSGTX, millis() - start_timeSendMessage); // commented to stop printing the total TX time
				start_getdata = millis();
			}
		}
		//DBTAG1
		// start_timeSendMessage = millis(); // commented to stop printing the total TX time
		sendMessage(outputMessage);
		outputMessage = "";

		// Test code to simulate a write delay
		//static int delayCounter = 0;
		//delayCounter++;
		//if (delayCounter == 50) {
		//	delay(3000);
		//	delayCounter = 0;
		//}

		if (duration_timeOpenFile && !sent_FOPEN){
			addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILEOPEN, duration_timeOpenFile);
			sent_FOPEN = true;
		}
		if (fileOpened){
			addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILEWRITES,0); // to add the file write times
			addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILESYNC, duration_timeFileSync); // to add the file sync time
		}
		if (duration_timeFileClose && !sent_FCLOSE){
			addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_FILECLOSE, duration_timeFileClose);
			sent_FCLOSE = true;
		}
		addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_UDPTX, duration_udpSend);
		addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_SDCARDTX, duration_sdcardSend);
		// addDebugPacket((uint8_t)EmotiBit::DebugTags::TIME_MSGTX, millis() - start_timeSendMessage); // commented to stop printing the total TX time
		//DBTAG1
	}
	
	// To update Battery level variable for LED indication
	if (battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_HIGH))
		battIndicationSeq = 0;
	else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_HIGH) && battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_MED)) {
		battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_HIGH);
		BattLedDuration = 1000;
	}
	else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_MED) && battLevel > uint8_t(EmotiBit::BattLevel::THRESHOLD_LOW)) {
		battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_MED);
		BattLedDuration = 500;
	}
	else if (battLevel < uint8_t(EmotiBit::BattLevel::THRESHOLD_LOW)) {
		battIndicationSeq = uint8_t(EmotiBit::BattLevel::INDICATION_SEQ_LOW);
		BattLedDuration = 100;
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

/*
StartHibernate
*/
void initHibernate() {
	if (!startHibernate && millis() - hibernateButtonStart > hibernateButtonDelay) {
		// delay exceeded
		//call Long Press Function
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

/*
Function to update the Wifi status on the EmotiBit
*/
void wifiModeControl() {
	if (1) {
		switch (wifiState) {
		case (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL:
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_LOWPOWER;
			Serial.print("WifiMode:In Low Power Mode");
			break;
		case (uint8_t)EmotiBit::WiFiPowerMode::WIFI_LOWPOWER:
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_MAX_LOWPOWER;
			Serial.print("WifiMode:In Max Low Power mode");
			break;
		case (uint8_t)EmotiBit::WiFiPowerMode::WIFI_MAX_LOWPOWER:
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END;
			Serial.print("WifiMode: End Wifi");
			break;
		case (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END:
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL;
			Serial.print("WifiMode: In Normal Mode");
			break;
		}
	}
	//  TODO: declare a variable to track changing between 2 states.
	else {// for toggling between states
		if (wifiState != (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL || wifiState != (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END) {
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL;
		}
		switch (wifiState) {
		case (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL:
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END;
			Serial.print("WifiMode:In Low Power Mode");
			break;
		case (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END:
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL;
			Serial.print("WifiMode: In Normal Mode");
			break;
		}
	}
}
void readSensors() {
#ifdef DEBUG_GET_DATA
	Serial.println("readSensors()");
#endif // DEBUG

	// ToDo: Move readSensors and timer into EmotiBit
	
	// LED STATUS CHANGE SEGMENT
	if (UDPtxLed) 
		emotibit.led.setLED(uint8_t(EmotiBit::Led::LED_BLUE), true);
	else
		emotibit.led.setLED(uint8_t(EmotiBit::Led::LED_BLUE), false);

	if (battIndicationSeq) {
		if (battLed && millis() - BattLedstatusChangeTime > BattLedDuration) {
			emotibit.led.setLED(uint8_t(EmotiBit::Led::LED_YELLOW), false);
			battLed = false;
			BattLedstatusChangeTime = millis();
		}
		else if (!battLed && millis() - BattLedstatusChangeTime > BattLedDuration) {
			emotibit.led.setLED(uint8_t(EmotiBit::Led::LED_YELLOW), true);
			battLed = true;
			BattLedstatusChangeTime = millis();
		}
	}
	else {
		emotibit.led.setLED(uint8_t(EmotiBit::Led::LED_YELLOW), false);
		battLed = false;
	}


	if (sdWrite) {
		if (millis() - recordBlinkDuration >= 500) {
			if (recordLedStatus == true) {
				emotibit.led.setLED(uint8_t(EmotiBit::Led::LED_RED), false);
				recordLedStatus = false;
			}
			else {
				emotibit.led.setLED(uint8_t(EmotiBit::Led::LED_RED), true);
				recordLedStatus = true;
			}
			recordBlinkDuration = millis();
		}
	}
	else if (!sdWrite && recordLedStatus == true) {
		emotibit.led.setLED(uint8_t(EmotiBit::Led::LED_RED), false);
		recordLedStatus = false;
	}

	// EDA
	if (acquireData.eda) {
		static uint16_t edaCounter = 0;
		edaCounter++;
		if (edaCounter == EDA_SAMPLING_DIV) {
			int8_t tempStatus = emotibit.updateEDAData();
			edaCounter = 0;
		}
	}

	// Temperature / Humidity Sensor
	if (acquireData.tempHumidity) {
		static uint16_t temperatureCounter = 0;
		temperatureCounter++;
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
		}
	}

	

	// IMU
	if (acquireData.imu) {
		static uint16_t imuCounter = 0;
		imuCounter++;
		if (imuCounter == IMU_SAMPLING_DIV) {
  			int8_t tempStatus = emotibit.updateIMUData();
			imuCounter = 0;
		}
	}

	// PPG
	if (acquireData.ppg) {
		static uint16_t ppgCounter = 0;
		ppgCounter++;
		if (ppgCounter == PPG_SAMPLING_DIV) {
			int8_t tempStatus = emotibit.updatePPGData();
			ppgCounter = 0;
		}
	}

	// Battery (all analog reads must be in the ISR)
	// TODO: use the stored BAtt value instead of calling readBatteryPercent again
	battLevel = emotibit.readBatteryPercent();
	static uint16_t batteryCounter = 0;
	batteryCounter++;
	if (batteryCounter == BATTERY_SAMPLING_DIV) {
		emotibit.updateBatteryPercentData();
		batteryCounter = 0;
	}
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

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

// NEW Hibernate
void hibernate() {
	Serial.println("hibernate()");
	Serial.println("Stopping timer...");
	stopTimer();

	//PPGToggle
	// For an unknown reason, this need to be before WiFi diconnect/end
	Serial.println("Shutting down ppg...");
	emotibit.ppgSensor.shutDown();

#ifdef SEND_UDP || SEND_TCP
	if (wifiStatus == WL_CONNECTED) {
		Serial.println("Disconnecting WiFi...");
		WiFi.disconnect();
	}
	Serial.println("Ending WiFi...");
	WiFi.end();
#endif

	//IMU Suspend Mode
	Serial.println("Suspending IMU...");
	BMI160.suspendIMU();

	SPI.end(); // shutdown the SPI interface

	Wire.end();


	//pinMode(emotibit._sdCardChipSelectPin, OUTPUT);//cs
	//digitalWrite(emotibit._sdCardChipSelectPin, LOW);
	//pinMode(PIN_SPI_MISO, OUTPUT);
	//digitalWrite(PIN_SPI_MISO, LOW);
	//pinMode(PIN_SPI_MOSI, OUTPUT);
	//digitalWrite(PIN_SPI_MOSI, LOW);
	//pinMode(PIN_SPI_SCK, OUTPUT);
	//digitalWrite(PIN_SPI_SCK, LOW);

	//pinMode(PIN_WIRE_SCL, OUTPUT);
	//digitalWrite(PIN_WIRE_SCL, LOW);
	//pinMode(PIN_WIRE_SDA, OUTPUT);
	//digitalWrite(PIN_WIRE_SDA, LOW);

	//pinMode(PIN_UART, OUTPUT);
	//digitalWrite(PIN_WIRE_SCL, LOW);
	//pinMode(PIN_WIRE_SDA, OUTPUT);
	//digitalWrite(PIN_WIRE_SDA, LOW);

	/*while (ledPinBusy)*/

		// Setup all pins (digital and analog) in INPUT mode (default is nothing)  
		//for (uint32_t ul = 0; ul < NUM_DIGITAL_PINS; ul++)
	for (uint32_t ul = 0; ul < PINS_COUNT; ul++)
	{
		if (ul != emotibit._analogEnablePin) {
			pinMode(ul, OUTPUT);
			digitalWrite(ul, LOW);
			pinMode(ul, INPUT);
		}
	}

	//GSRToggle, write 1 to the PMO
	Serial.println("Disabling MOSFET ");
	pinMode(emotibit._analogEnablePin, OUTPUT);
	digitalWrite(emotibit._analogEnablePin, HIGH);


	//LowPower.deepSleep();
	//deepSleep();
	Serial.println("Entering deep sleep...");
	LowPower.deepSleep();

	Serial.println("Entering sleep loop...");
	while (true) {



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

	return (int8_t) digitalRead(emotibit.switchPin);
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
	//StaticJsonBuffer<1024> jsonBuffer;
	StaticJsonBuffer<1024> jsonBuffer;

	// Parse the root object
	JsonObject &root = jsonBuffer.parseObject(file);

	if (!root.success()) {
		Serial.println(F("Failed to parse config file"));
		return false;
	}

	// Copy values from the JsonObject to the Config
	configSize = root.get<JsonVariant>("WifiCredentials").as<JsonArray>().size();
	Serial.print("ConfigSize: ");
	Serial.println(configSize);
	for (size_t i = 0; i < configSize; i++) {
		configList[i].ssid = root["WifiCredentials"][i]["ssid"] | "";
		configList[i].password = root["WifiCredentials"][i]["password"] | "";
		Serial.println(configList[i].ssid);
		Serial.println(configList[i].password);
	}

	//strlcpy(config.hostname,                   // <- destination
	//	root["hostname"] | "example.com",  // <- source
	//	sizeof(config.hostname));          // <- destination's capacity

	// Close the file (File's destructor doesn't close the file)
	// ToDo: Handle multiple credentials

	file.close();
	return true;
}

bool sendSdCardMessage(String & s) {
	// Break up the message in to bite-size chunks to avoid over running the UDP or SD card write buffers
	// UDP buffer seems to be about 1400 char. SD card buffer seems to be about 2500 char.
	if (sdWrite && s.length() > 0) {
		//DBTAG1
		// dataFile = SD.open(sdCardFilename, FILE_WRITE);//O_CREAT | O_WRITE
		
		uint8_t writeCounter = 0;
		uint32_t start_timeWriteFile;
		for(uint8_t k = 0; k < MAX_SD_WRITE_TIMES;k ++){
			duration_timeWriteFile[k] = 0;  //{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		}
		// uint32_t start_timeOpenFile = millis();
		// dataFile = SD.open(sdCardFilename, O_CREAT | O_WRITE | O_AT_END);
		// duration_timeOpenFile =  millis() - start_timeOpenFile;
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
				// DBTAG1
				start_timeWriteFile = millis();
				dataFile.print(s.substring(firstIndex, lastIndex));
				if (writeCounter < MAX_SD_WRITE_TIMES) {
					duration_timeWriteFile[writeCounter] = millis() - start_timeWriteFile;
					writeCounter++;
				}
				else {
					duration_timeWriteFile[MAX_SD_WRITE_TIMES - 1] = 255;
				}
				firstIndex = lastIndex;

				//dataFile.flush();
				//firstIndex = lastIndex + 1;	// increment substring indexes for breaking up sends
			}
			
			// uint32_t start_timeFileClose = millis();
			// dataFile.close();
			// duration_timeFileClose = millis() - start_timeFileClose;// to add the file close times
			uint32_t start_timeFileSync = millis();
			dataFile.sync();
			duration_timeFileSync = millis() - start_timeFileSync;// to add the file close times
		}
		else {
			Serial.print("Data file didn't open properly: ");
			Serial.println(sdCardFilename);
			sdWrite = false;
		}
	}
}

#ifdef SEND_UDP
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
#endif

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
		//DBTAG1
		if (!wifiState) { // only transmit if the WifiState is ON
			start_udpSend = millis();
			sendUdpMessage(s);
			duration_udpSend = millis() - start_udpSend;
			UDPtxLed = !UDPtxLed;
		}
		else {
			UDPtxLed = false;
		}
		if (wifiState == (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END) {
			WiFi.disconnect();
			WiFi.end();
		}
		
#endif // SEND_UDP	
		// Write to SD card after sending network messages
		// This give the receiving computer time to process in advance of a sync exchange
		//DBTAG1
		start_sdcardSend = millis();
		sendSdCardMessage(s);  
		duration_sdcardSend = millis() - start_sdcardSend;
}

#ifdef SEND_UDP || SEND_TCP
void updateWiFi() {
	wifiStatus = WiFi.status();
	if (wifiStatus != WL_CONNECTED) {
		wifiReady = false;
		socketReady = false;
		//DBTAG1
		// If Wifi gets disconnected
		if (!connected){
			addDebugPacket((uint8_t)EmotiBit::DebugTags::WIFI_DISCONNECT, millis());
			connected = true;
		}

	}
#if 0
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
			//DBTAG1
			addDebugPacket((uint8_t)EmotiBit::DebugTags::WIFI_TIMELOSTCONN, momentLost);// for reporting Wifi Lost moment
		}

		if (wifiState != (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END && millis() - networkBeginStart > WIFI_BEGIN_ATTEMPT_DELAY) {
				if ((millis() - momentLost > WIFI_BEGIN_SWITCH_CRED) && (attempts >= 2)) {
					switchCred = true;
					gotIP = false;
					sendResetPacket = true;
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
		// DBTAG1
		if (connected){
			connected = false;
			addDebugPacket((uint8_t)EmotiBit::DebugTags::WIFI_CONNECT, millis());
		}
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
#endif
