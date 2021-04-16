/*
This example demonstrates use of PPG data from EmotiBit to light up heart emoji on the 
attached charliplex wing.
Working principle:
1. The wavelength signal is filter using a low pass filter of chosen frequency and this 
filtered signal is used as a hreshold to light up the attached charlieplex.
*/
#include "EmotiBit.h"
#include "EmojiBit.h"
#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];
DigitalFilter filter_lp2(DigitalFilter::FilterType::IIR_LOWPASS, 25, 0.075);

// Initializing Charlieplex wing
EmojiBit matrix = EmojiBit();

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
	delay(2000);	// short delay to allow user to connect to serial, if desired

	emotibit.setup();
	// Attach callback functions
	emotibit.attachShortButtonPress(&onShortButtonPress);
	emotibit.attachLongButtonPress(&onLongButtonPress);
	Serial.println("##############################");
	Serial.println("# Hearbeat on sleeve Example #");
	Serial.println("##############################");
	if (!matrix.begin()) {
		Serial.println("IS31 not found");
		while (1);
	}
	Serial.println("IS31 Found!");
}

void loop()
{
	//Serial.println("emotibit.update()");
	emotibit.update();
	float threshold = 0;
	size_t dataAvailable = emotibit.readData(EmotiBit::DataType::PPG_INFRARED, &data[0], dataSize);
	if (dataAvailable > 0)
	{
		// Hey cool, I got some data! Maybe I can light up my shoes whenever I get excited!

		// print the data to view in the serial plotter
		bool printData = true;
		if (printData)
		{
			for (size_t i = 0; i < dataAvailable && i < dataSize; i++)
			{
				// Note that dataAvailable can be larger than dataSize
				Serial.print(data[i]); Serial.print("\t");
				threshold = filter_lp2.filter(data[i]);
				Serial.println(threshold); //Serial.print("\t");
				if (data[i] > threshold)
				{
					// light up heart emoji
					matrix.drawEmoji(EMOJI::HEART, 1);
				}
				else
				{
					// clear heart emoji
					matrix.clear();
				}
			}
		}
	}
}