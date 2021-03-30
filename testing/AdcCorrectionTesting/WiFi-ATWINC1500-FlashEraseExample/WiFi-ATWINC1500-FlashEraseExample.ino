/*
 * This code has been taken from the example code provided in the spi_flash.h file in WiFi101 arduino Library.
 * Code has been modified to run on the Feather Wing M0
 */
#include <SPI.h>
#include <WiFi101.h>
#include <spi_flash/include/spi_flash.h>
#include <spi_flash/include/spi_flash_map.h>
#include "AdcCorrection.h"
/*
#define ATWINC_MEM_LOC_PRIMARY_DATA (448*1024UL)
#define ATWINC_MEM_LOC_DUPLICATE_DATA (480*1024UL)
#define ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE (508 * 1024)
#define ATWINC_MEM_LOC_METADATA (512*1024 - 3)
#define FLASH_SIZE (512*1024UL)
#define FLASH_DATA_READ_LENGTH 12
#define FLASH_ERASE_LENGTH 1
#define FLASH_READ_ENABLE 1
#define FLASH_ERASE_ENABLE 1
*/
void setup() {
  // put your setup code here, to run once:
	Serial.begin(115200);
	const uint8_t readSize = 40;
	uint8_t data[readSize];
	while (!Serial.available())
	{
		Serial.println("Enter a character to continue");
		delay(1000);
	}
	Serial.read();
	Serial.println("This code will erase the primary location, secondary location and the last sector(metadata on the AT-WINC)");
	while (!Serial.available())
	{
		Serial.println("Enter a character to continue");
		delay(1000);
	}
	AdcCorrection adcCorrection;
	// Initialize the WiFI module and put it in download mode. Flash access is enabled only when the module is in download mode
	WiFi.setPins(8,7,4,2); // Need this for working with WiFi module on Adafruit feather
	nm_bsp_init();
	uint8_t ret;
	uint8_t atwincFlashSize;
	ret = m2m_wifi_download_mode();

	if(M2M_SUCCESS != ret)
	{
	  Serial.println("Unable to enter download mode\r\n");
	}
	else
	{
		atwincFlashSize = spi_flash_get_size();
	}
	Serial.println("Entered download mode successfully");
	ret = spi_flash_erase(adcCorrection.ATWINC_MEM_LOC_PRIMARY_DATA, 12);
	Serial.println("Done erasing primary sector");
	ret = spi_flash_erase(adcCorrection.ATWINC_MEM_LOC_DUPLICATE_DATA, 12);
	Serial.println("Done erasing secondary sector");
	ret = spi_flash_erase(adcCorrection.ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE, 12);
	Serial.println("Done erasing last sector");
	Serial.print("The flash size is:"); Serial.println(atwincFlashSize);
	Serial.println("Reading location to verify data was cleared(expected value is 255for a erased memory location");
	Serial.println("\nReading primary loc");
	ret = adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_PRIMARY_DATA, readSize, data);
	Serial.println("\nReading secondary loc");
	ret = adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_DUPLICATE_DATA, readSize, data);
	Serial.println("\nReading metadata loc");
	ret = adcCorrection.readAtwincFlash(adcCorrection.ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE, readSize, data);
	Serial.println("\nend of code");

	while(1);

}

void loop() {
  // put your main code here, to run repeatedly:

}
