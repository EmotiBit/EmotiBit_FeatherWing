#include "EmotiBit.h"

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE

const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;


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
Function to attach callback to short press
*/
void attachToShortButtonPress(void(&shortSwitchPressFunction)(void)) {
	onShortPress = &shortSwitchPressFunction;
}

/*
Function to attach callback to short press
*/
void attachToLongButtonPress(void(&longSwitchPressFunction)(void)) {
	onLongPress = &longSwitchPressFunction;
}

void setup() {
	Serial.begin(SERIAL_BAUD);
	Serial.println("Serial started");
	delay(2000);	// short delay to allow user to connect to serial, if desired
	//while (!Serial);

	emotibit.setup(EmotiBit::Version::V02H);

	//assignButtonCallbacks((uint8_t)EmotiBit::FunctionalityShortPress::WIFI, (uint8_t)EmotiBit::FunctionalityLongPress::HIBERNATE);
	attachToShortButtonPress(*wifiModeControl);
	attachToLongButtonPress(*initHibernate);
}

void loop() {
#ifdef DEBUG_GET_DATA
	Serial.println("loop()");
#endif // DEBUG

	// Check for hibernate button press
	if (switchRead() == 0) {
		hibernateButtonPressed = false;
		//Serial.println("SwitchNotPressed");
	}
	// TODO: When a switch debouncer is added, remove the 500ms delay in polling
	else if (switchRead() == 1 && millis() - hibernateButtonStart > 500) { // poll only after 500mSec, until aswitch  debouncer is added
		if (hibernateButtonPressed) {
			//Serial.println("Switch long Pressed");
			// Long Press
			// hibernate button was already pressed -- check how long
			onLongPress();
			Serial.println("onLongPress");
		}
		else {
			//Serial.println("Switch short Pressed");
			// start timer for hibernate
			hibernateButtonPressed = true;
			hibernateButtonStart = millis();
			onShortPress();
			//Serial.println("onShortPress");
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
	if (0) { // to toggle between 4 states
		if (wifiState == (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL) {
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_LOWPOWER;
			Serial.println("WifiMode:In Low Power Mode");
		}
		else if (wifiState == (uint8_t)EmotiBit::WiFiPowerMode::WIFI_LOWPOWER) {
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_MAX_LOWPOWER;
			Serial.println("WifiMode:In Max Low Power mode");
		}
		else if (wifiState == (uint8_t)EmotiBit::WiFiPowerMode::WIFI_MAX_LOWPOWER) {
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END;
			Serial.println("WifiMode: End Wifi");
		}
		else if (wifiState == (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END) {
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL;
			Serial.println("WifiMode: In Normal Mode");
		}
	}
	//  TODO: declare a variable to track changing between 2 states.
	else {// for toggling between 2 states
		if (wifiState != (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL && wifiState != (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END) {
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL;
		}
		if (wifiState == (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL) {
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END;
			Serial.print("WifiMode:End Wifi");
		}
		else if(wifiState == (uint8_t)EmotiBit::WiFiPowerMode::WIFI_END){
			wifiState = (uint8_t)EmotiBit::WiFiPowerMode::WIFI_NORMAL;
			Serial.println("WifiMode:End Normal Mode");
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

	// Thermopile

	if (acquireData.tempHumidity) {
		static uint16_t thermopileCounter = 0;
		thermopileCounter++;
		if (thermopileCounter == THERMOPILE_SAMPLING_DIV) {
			int8_t tempStatus = emotibit.updateThermopileData();
			thermopileCounter = 0;
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
				if (firstIndex == 0) {
					UDPtxLed = !UDPtxLed;
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
			//UDPtxLed = !UDPtxLed;
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
