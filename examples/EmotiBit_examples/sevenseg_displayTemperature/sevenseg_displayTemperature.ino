#include "EmotiBit.h"
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_7segment matrix = Adafruit_7segment();

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
	emotibit.hibernate();
}

void setup() 
{
	Serial.begin(SERIAL_BAUD);
	Serial.println("Serial started");
	Serial.println("EmotiBit Temperature 7 Segment Backpack");
	matrix.begin(0x70);
	delay(2000);	// short delay to allow user to connect to serial, if desired

	emotibit.setup(EmotiBit::Version::V02H);

	// Attach callback functions
	emotibit.attachShortButtonPress(&onShortButtonPress);
	emotibit.attachLongButtonPress(&onLongButtonPress);
}

void loop()
{
	//Serial.println("emotibit.update()");
	emotibit.update();

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
		matrix.print(smoothData);
		matrix.writeDisplay();

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