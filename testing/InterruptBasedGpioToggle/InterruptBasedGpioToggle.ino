// Code to create a sq waveform to test timing accuracy
#define OUT_PIN 5
hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer(){
digitalWrite(OUT_PIN, !digitalRead(OUT_PIN));
}
void setup() {
pinMode(OUT_PIN, OUTPUT);
My_timer = timerBegin(0, 80, true);
timerAttachInterrupt(My_timer, &onTimer, true);
// We want to create a wave with a time period of 6666uS (150Hz)
// Choose timer to be 1/2 of required time period
timerAlarmWrite(My_timer, 3333, true); 
timerAlarmEnable(My_timer); //Just Enable
}
void loop() {
}