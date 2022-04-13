#include "EmotiBit.h"

const int syncPulsePin = A2;	// Pin on which to receive sync pulses
struct SyncPulse
{
	// ToDo: This should be put in a templated ring buffer
	unsigned long long timestamp;
	float data;
	unsigned int eventCounter;	// hack to capture number of events without a buffer
};
volatile struct SyncPulse syncPulse;

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];

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
	emotibit.sleep();
}

void syncPulseChange()
{
	// ToDo: consider debouncing
	syncPulse.timestamp = millis();
	syncPulse.data = digitalRead(syncPulsePin);
	syncPulse.eventCounter++;
}

void setup() 
{
	Serial.begin(SERIAL_BAUD);
	Serial.println("Serial started");
	delay(2000);	// short delay to allow user to connect to serial, if desired

	emotibit.setup();

	// Attach callback functions
	emotibit.attachShortButtonPress(&onShortButtonPress);
	emotibit.attachLongButtonPress(&onLongButtonPress);

	// Attach interrupts to the rising and falling edges of a sync pulse on a free pin
	// IMPORTANT NOTE: digital input pins must never exceed the range GND-0.6v to VDD+0.6V (i.e. best to keep it within 0V-3.3V)
	// See Feather M0 pinouts: https://learn.adafruit.com/adafruit-feather-m0-wifi-atwinc1500/pinouts
	// See https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
	// ToDo: consider the best way to toggle attaching sync pulse interrupts
	// Options:
	//	- dedicate specified pin in main-line code
	//	- comment in/out
	//	- boolean / ifdef at top of ino
	//	- separate ino/bin (present method)
	//	- reading field from SD card
	//	- control message from EmotiBit Oscilloscope
	Serial.print("Attaching interrupt to sync pulses on pin ");
	Serial.println(syncPulsePin);
	pinMode(syncPulsePin, INPUT_PULLDOWN);
	attachInterrupt(digitalPinToInterrupt(syncPulsePin), syncPulseChange, CHANGE);
}

void loop()
{
	//Serial.println("emotibit.update()");
	emotibit.update();

	size_t dataAvailable = emotibit.readData(EmotiBit::DataType::PPG_GREEN, &data[0], dataSize);
	if (dataAvailable > 0)
	{
		// Hey cool, I got some data! Maybe I can light up my shoes whenever I get excited!

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

	while(syncPulse.eventCounter > 0)
	{
		float data = syncPulse.data;
		emotibit.addPacket(syncPulse.timestamp, "SP", &data, 1);	// ToDo: consider adding EmotiBitPacket::TypeTag::SYNC_PULSE
		syncPulse.eventCounter--;

		Serial.print("SP");
		Serial.println((int) data);
	}

}