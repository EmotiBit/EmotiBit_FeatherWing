#include "EmotiBitTimers.h"
#include "Arduino.h"

EmotiBitTimers* myEmotiBitTimers = nullptr;
void(*onInterruptCallback)(void);

void attachEmotiBitToInterruptTC3(void(*readFunction)(void), EmotiBitTimers *e)
{
	onInterruptCallback = readFunction;
	attachEmotiBitTimer(e);
}

void attachEmotiBitTimer(EmotiBitTimers *e)
{
	myEmotiBitTimers = e;
}

void ReadSensors()
{
	if (myEmotiBitTimers != nullptr)
	{
		myEmotiBitTimers->readSensors();
	}
}

//EmotiBitTimers::EmotiBitTimers()
//{
//	myEmotiBit = nullptr;
//}

//#define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(TC3_Callback))

void EmotiBitTimers::readSensors()
{
	if (myEmotiBit != nullptr)
	{
		myEmotiBit->TC3_Callback();
		//TC3_Callback(myEmotiBit);
	}
}

void EmotiBitTimers::attachToInterruptTC3(EmotiBit *e)
{
	myEmotiBit = e;
	attachEmotiBitToInterruptTC3(&ReadSensors, this);
}

void EmotiBitTimers::setTimerFrequency(int frequencyHz) {
	uint32_t CPU_HZ = 40000;
	int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
	TcCount16* TC = (TcCount16*)TC3;
	// Make sure the count is in a proportional position to where it was
	// to prevent any jitter or disconnect when changing the compare value.
	TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
	TC->CC[0].reg = compareValue;
	//Serial.println(TC->COUNT.reg);
	//Serial.println(TC->CC[0].reg);
	while (TC->STATUS.bit.SYNCBUSY == 1);
}

void EmotiBitTimers::startTimer(int frequencyHz) {
	REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);
	while (GCLK->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TcCount16* TC = (TcCount16*)TC3;

	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Use the 16-bit timer
	TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Use match mode so that the timer counter resets when the count matches the compare register
	TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

																				// Set prescaler to 1024
	TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	setTimerFrequency(frequencyHz);

	// Enable the compare interrupt
	TC->INTENSET.reg = 0;
	TC->INTENSET.bit.MC0 = 1;

	NVIC_EnableIRQ(TC3_IRQn);

	TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void EmotiBitTimers::stopTimer() {
	// ToDo: Verify implementation
	TcCount16* TC = (TcCount16*)TC3;
	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
}

void TC3_Handler() {

	TcCount16* TC = (TcCount16*)TC3;
	// If this interrupt is due to the compare register matching the timer count
	// we toggle the LED.
	if (TC->INTFLAG.bit.MC0 == 1) {
		TC->INTFLAG.bit.MC0 = 1;
		//scopeTimingTest();
		// Write callback here!!!
#ifdef TIMER_TEST
		toggleLED();
		Serial.println("LED Testing Routine");
#else
		onInterruptCallback();
#endif
	}
}

