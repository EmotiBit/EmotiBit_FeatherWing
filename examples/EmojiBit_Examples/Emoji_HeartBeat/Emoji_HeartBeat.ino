#include "EmotiBit.h"

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];
DigitalFilter filter_lp(DigitalFilter::FilterType::IIR_LOWPASS, 25, 23);
DigitalFilter filter_hp(DigitalFilter::FilterType::IIR_HIGHPASS, 25, 20);
DigitalFilter filter_lp2(DigitalFilter::FilterType::IIR_LOWPASS, 25, 0.09);
int redLedPin = 14;

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
	Serial.println("rawdata,LowPass, BandPass");
	pinMode(redLedPin, OUTPUT);
}

void loop()
{
	//Serial.println("emotibit.update()");
	emotibit.update();
	float lp_filtered_value = 0;
	float hp_filtered_value = 0;
	float signalLine = 0;
	bool displayHeart = false;
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
				//Serial.print(data[i]); Serial.print("\t");
				lp_filtered_value = filter_lp.filter(data[i]);
				hp_filtered_value = filter_hp.filter(lp_filtered_value);
				//hp_filtered_value = filter_hp.filter(data[i]);
				//Serial.print(lp_filtered_value); Serial.print("\t");
				Serial.print(hp_filtered_value); Serial.print("\t");
				signalLine = filter_lp2.filter(hp_filtered_value);
				Serial.println(signalLine);
				if (hp_filtered_value > signalLine)
				{
					// use with I2C
					//displayHeart = true;
					digitalWrite(redLedPin, HIGH);
				}
				else
				{
					// use bool with I2c
					//displayHeart = false;
					digitalWrite(redLedPin, LOW);
				}
			}
		}
	}
}