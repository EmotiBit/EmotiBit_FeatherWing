// bitmaps for emoji stored in the program memory
static const uint8_t PROGMEM
emojiBitmaps[][8] =
{   // Bitmaps taken from Adafruit Adafruit_IS31FL3731_Library gfx Demo example
	// Smiley face
  { B00111100,
	B01000010,
	B10100101,
	B10000001,
	B10100101,
	B10011001,
	B01000010,
	B00111100 },
  // Neutral face
  { B00111100,
	B01000010,
	B10100101,
	B10000001,
	B10111101,
	B10000001,
	B01000010,
	B00111100 },
  // Frown face
  { B00111100,
	B01000010,
	B10100101,
	B10000001,
	B10011001,
	B10100101,
	B01000010,
	B00111100 },
  // heart
   {
	B01110000,
	B11111000,
	B11111100,
	B01111110,
	B11111100,
	B11111000,
	B01110000
	}
};

// Update this enum when new templates are added to the emojiBitmaps
enum EMOJI
{
	SMILE,
	NEUTRAL,
	FROWN,
	HEART,
	LENGTH
};

