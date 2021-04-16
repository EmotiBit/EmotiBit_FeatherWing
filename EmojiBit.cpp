#include"EmojiBit.h"


EmojiBit::EmojiBit(void)
{
	Adafruit_IS31FL3731_Wing();
}

/*
 Description: Function to draw emoji on CharliePlex wings
 Inputs:
		emojiTemplate: refer enum EMOJI in emojiTemplates.h for options
		orientation:   accepts 0 or 1
		intensity:     Change the intensity of CharliePlex led
		xPos:          First led position along X-Axis. default 3
		yPos:          First led position along Y-Axis. default 0
		width:         Width of the emoji. default 8
		height:        Width of the emoji. default 8
 
 */
void EmojiBit::drawEmoji(uint8_t emojiTemplate, uint8_t orientation, uint8_t intensity, uint8_t xPos, uint8_t yPos, uint8_t width, uint8_t height)
{
	setRotation(orientation);
	
	if (rotation)
	{
		xPos = 0;
		yPos = 4; // Hardcoded shift to bring the 8x8 emoji to the center to the 15x7 charliePlex Wing
	}
	drawBitmap(xPos, yPos, emojiBitmaps[emojiTemplate], width, height, intensity);
}
