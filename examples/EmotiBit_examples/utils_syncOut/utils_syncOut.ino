#include <Arduino.h>
#include "EmotiBit.h"

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

/// @brief Generates a sync pulse in a digital out pin.
/// @details	Toggles a SYNC_PIN digital out HIGH every Nth call of the function. 
///         	The following call, pulls the SYNC_PIN LOW again. A "SP" packet  
///						with a timestamp is logged every time the SYNC_PIN is toggled for
///						easy synchronization across devices into local time.
/// @return The state of the SYNC_PIN is returned.
bool syncOut()
{
	// sync out setup variables //
#if defined (ARDUINO_FEATHER_ESP32)
	//const uint8_t SYNC_PIN = 26;
	const uint8_t SYNC_PIN = A0;
#elif defined (ADAFRUIT_FEATHER_M0)
	const uint8_t SYNC_PIN = 14;
#endif

	const unsigned long offBaseTime = 900; 
	const unsigned long offRandMax = 200;
	const unsigned long onTime = 100;
	// end sync out setup variables //
	
	// sync out code logic
	static unsigned long syncTimer = millis();
	static unsigned long offRand = random(offRandMax);
	static int toggle = LOW;
	static bool sendSync = false;
	static bool isSetup = false;
	
	if (!isSetup) 
	{
		pinMode(SYNC_PIN , OUTPUT);
		isSetup = true;
	}
	if (toggle == LOW && ((millis() - syncTimer) > (offBaseTime + offRand)))
	{
		sendSync = true;
		offRand = random(offRandMax); // get a new random offRand
	}
	if (toggle == HIGH && ((millis() - syncTimer) > onTime))
	{
		sendSync = true;
	}
	if (sendSync)
	{
		syncTimer = millis();
		toggle = !toggle;
		float toggleFloat = (float) toggle;
		digitalWrite(SYNC_PIN , toggle );
		emotibit.addPacket(millis(), "SP", &toggleFloat, 1); // Log the SYNC_PIN toggle
		// ToDo: consider adding EmotiBitPacket::TypeTag::SYNC_PULSE
		sendSync = false;
	}
	return toggle;
	
	// /// Deprecated loop-based version
	// const int NTH_LOOP = 2000; // toggles the sync pulse high every Nth loop

	// // sync out code logic
	// static bool isSetup = false;
	// static int loopCounter = 0;
	// static int toggle = LOW;

	// if (!isSetup) 
	// {
		// pinMode(SYNC_PIN , OUTPUT);
		// isSetup = true;
	// }

	// loopCounter++;
	// if (loopCounter >= NTH_LOOP || toggle == HIGH) 
	// {
		// toggle = !toggle;
		// float toggleFloat = (float) toggle;
		// digitalWrite(SYNC_PIN , toggle );
		// emotibit.addPacket(millis(), "SP", &toggleFloat, 1); // Log the SYNC_PIN toggle
		// // ToDo: consider adding EmotiBitPacket::TypeTag::SYNC_PULSE
	// }
	// if (loopCounter >= NTH_LOOP) 
	// {
		// loopCounter = 0;
	// }
	// return toggle;
}

void setup() 
{
	Serial.begin(SERIAL_BAUD);
	Serial.println("Serial started");
	delay(2000);	// short delay to allow user to connect to serial, if desired

	// Capture the calling ino into firmware_variant information
	String inoFilename = __FILE__;  // returns absolute path of ino file
	inoFilename.replace("/","\\");  // conform to standard directory separator
	// extract filename only if directory separator found in absolute path
	if(inoFilename.lastIndexOf("\\") != -1)
	{
		inoFilename = inoFilename.substring((inoFilename.lastIndexOf("\\")) + 1,(inoFilename.indexOf(".")));
	}
	emotibit.setup(inoFilename);

	// Attach callback functions
	emotibit.attachShortButtonPress(&onShortButtonPress);
	emotibit.attachLongButtonPress(&onLongButtonPress);
}

void loop()
{
	//Serial.println("emotibit.update()");
	emotibit.update();
	Serial.println(syncOut());

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
}