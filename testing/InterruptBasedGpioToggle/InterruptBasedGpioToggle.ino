/**
 * @file InterruptBasedGpioToggle
 *
 * @section description Description
 * This example toggles a GPIO using hw timer.
 * Creates a sq waveform to test timing accuracy.
 * This example was created to test the GpioInterruptLogger example.
 *
 * @section circuit Circuit
 * - This is written for ESP32. Tested on Adafruit Feather ESP32 Huzzah.
 * - Pin 5 generates the timer output. Refer Adafruit Pinout for pin location and details: https://learn.adafruit.com/assets/111179
 * 
 * @section author Author
 * - Created by Nitin Nair for EmotiBit.
 * 
 */
 
#define OUT_PIN 5
hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer(){
digitalWrite(OUT_PIN, !digitalRead(OUT_PIN));
}
void setup() {
pinMode(OUT_PIN, OUTPUT);
My_timer = timerBegin(0, 80, true);
timerAttachInterrupt(My_timer, &onTimer, true);
// We want to create a wave with a time period of 6666uS (150Hz, default data acuisition task frequency)
// Choose timer to be 1/2 of required time period
timerAlarmWrite(My_timer, 3333, true); 
timerAlarmEnable(My_timer); //Just Enable
}
void loop() {
}