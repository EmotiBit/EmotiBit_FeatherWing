#include <Adafruit_GFX.h>
#include <Adafruit_IS31FL3731.h>
#include "EmojiTemplates.h"

class EmojiBit : public Adafruit_IS31FL3731_Wing
{
public:
	EmojiBit(void);
	void drawEmoji(uint8_t emojiTemplate, uint8_t orientation=0, uint8_t intensity=128, uint8_t xPos=3, uint8_t yPos=0, uint8_t width=8, uint8_t height=8);
};
