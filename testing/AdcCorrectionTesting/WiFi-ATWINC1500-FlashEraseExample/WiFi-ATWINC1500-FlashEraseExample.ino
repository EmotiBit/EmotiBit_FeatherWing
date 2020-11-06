/*
 * This code has been taken from the example code provided in the spi_flash.h file in WiFi101 arduino Library.
 * Code has been modified to run on the Feather Wing M0
 */
#include <SPI.h>
#include <WiFi101.h>
#include <spi_flash/include/spi_flash.h>
#include <spi_flash/include/spi_flash_map.h>

#define ATWINC_MEM_LOC_PRIMARY_DATA (448*1024UL)
#define ATWINC_MEM_LOC_DUPLICATE_DATA (480*1024UL)
#define ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE (508 * 1024)
#define ATWINC_MEM_LOC_METADATA (512*1024 - 3)
#define FLASH_SIZE (512*1024UL)
#define FLASH_DATA_READ_LENGTH 12
#define FLASH_ERASE_LENGTH 1
#define FLASH_READ_ENABLE 1
#define FLASH_ERASE_ENABLE 1

void setup() {
  // put your setup code here, to run once:
	Serial.begin(115200);
	while(!Serial.available());
	Serial.println("This code will erase the primary location, secondary location and the last sector(metadata on the AT-WINC)");

	uint8 au8FlashContent[FLASH_DATA_READ_LENGTH] = {0};
	uint32  u32FlashTotalSize = 0;
	uint32  u32FlashOffset = ATWINC_MEM_LOC_PRIMARY_DATA;
	uint8_t ret;
	// Initialize the WiFI module and put it in download mode. Flash access is enabled only when the module is in download mode
	WiFi.setPins(8,7,4,2); // Need this for working with WiFi module on Adafruit feather
	nm_bsp_init();
	ret = m2m_wifi_download_mode();

	if(M2M_SUCCESS != ret)
	{
	  printf("Unable to enter download mode\r\n");
	}
	else
	{
	  u32FlashTotalSize = spi_flash_get_size();
	}
	Serial.println("Entered download mode successfully");
	Serial.print("The flash size is:"); Serial.println(u32FlashTotalSize);

	int eraseIter = 0;
	while (eraseIter < 3)
	{
		if (eraseIter == 0)
			u32FlashOffset = ATWINC_MEM_LOC_PRIMARY_DATA;
		else if (eraseIter == 1)
		{
			u32FlashOffset = ATWINC_MEM_LOC_DUPLICATE_DATA;
		}
		else if (eraseIter == 2)
		{
			u32FlashOffset = ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE;
		}

		if (eraseIter != 2)
		{
			ret = spi_flash_read(au8FlashContent, u32FlashOffset, FLASH_DATA_READ_LENGTH);
			if (M2M_SUCCESS != ret)
			{
				printf("Unable to read SPI sector\r\n");
				break;
			}
			else
			{
				Serial.print("the data stored in the flash "); Serial.print(" @memLoc "); Serial.println(u32FlashOffset);
				for (int i = 0; i < FLASH_DATA_READ_LENGTH; i++)
				{
					Serial.print(i); Serial.print(":"); Serial.print(au8FlashContent[i]); Serial.print("\t");
				}
			}

			Serial.println("\nErasing flash");
			ret = spi_flash_erase(u32FlashOffset, FLASH_ERASE_LENGTH);
			if (M2M_SUCCESS != ret)
			{
				Serial.print("Unable to erase SPI sector\r\n");
				break;
			}
			Serial.print("the UPDATED data stored in the flash "); Serial.print(" @memLoc "); Serial.println(u32FlashOffset);
			ret = spi_flash_read(au8FlashContent, u32FlashOffset, FLASH_DATA_READ_LENGTH);
			if (M2M_SUCCESS != ret)
			{
				printf("Unable to read SPI sector\r\n");
				break;
			}
			else
			{
				for (int i = 0; i < FLASH_DATA_READ_LENGTH; i++)
				{
					Serial.print(i); Serial.print(":"); Serial.print(au8FlashContent[i]); Serial.print("\t");
				}
			}
			Serial.println("\n");
		}
		else // metadata
		{
			u32FlashOffset = ATWINC_MEM_LOC_LAST_SECTOR_FIRST_BYTE;
			ret = spi_flash_read(au8FlashContent, ATWINC_MEM_LOC_METADATA, 3);
			if (M2M_SUCCESS != ret)
			{
				printf("Unable to read SPI sector\r\n");
				break;
			}
			else
			{
				Serial.print("the data stored in the flash "); Serial.print(" @memLoc "); Serial.println(ATWINC_MEM_LOC_METADATA);
				for (int i = 0; i < 3; i++)
				{
					Serial.print(i); Serial.print(":"); Serial.print(au8FlashContent[i]); Serial.print("\t");
				}
			}

			Serial.println("\nErasing flash");
			ret = spi_flash_erase(u32FlashOffset, FLASH_ERASE_LENGTH);
			if (M2M_SUCCESS != ret)
			{
				Serial.print("Unable to erase SPI sector\r\n");
				break;
			}
			Serial.print("the UPDATED data stored in the flash "); Serial.print(" @memLoc "); Serial.println(ATWINC_MEM_LOC_METADATA);
			ret = spi_flash_read(au8FlashContent, ATWINC_MEM_LOC_METADATA, 3);
			if (M2M_SUCCESS != ret)
			{
				printf("Unable to read SPI sector\r\n");
				break;
			}
			else
			{
				for (int i = 0; i < 3; i++)
				{
					Serial.print(i); Serial.print(":"); Serial.print(au8FlashContent[i]); Serial.print("\t");
				}
			}
			Serial.println("\n");
		}
		eraseIter++;
	}

	Serial.println("Verify that all the memory locations printed above hold 255 after erasing.");
	Serial.println("\nend of setup");

	while(1);

}

void loop() {
  // put your main code here, to run repeatedly:

}
