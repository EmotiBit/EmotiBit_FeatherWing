#include"EmojiBit.h"


EmojiBit::EmojiBit(void)
{
	Adafruit_IS31FL3731_Wing();
}

void EmojiBit::drawEmoji(uint8_t emojiTemplate, uint8_t intensity, uint8_t xPos, uint8_t yPos, uint8_t width, uint8_t height)
{
	drawBitmap(xPos, yPos, emoji_bitmaps[emojiTemplate], width, height, intensity);
}
