## About the Example
This example can be used to display your heart beat on a charlieplex feather wing.

## Requirements
- [EmotiBit](https://www.emotibit.com/)
- [Adafruit Feather M0 [with stacking header]](https://www.adafruit.com/product/3010)
- [Charlieplex Feather Wing](https://www.adafruit.com/product/3134)

## Steps
- You will require a Feather M0 with stacking headers soldered onto it. This Enables the feather to be connected to the EmotiBit and also the CharliePlex feather Wing!
- Program the fewather M0 with this example.
- Stack the EmotiBit, Feather M0 and Charlieplex feather wing together.
- That's it! Its that easy!

## How this code works
- This example uses the concept of a simple crossing detector.
- We filter the input(PPG data) using a LPF. This filtered signal now acts as a threshold.
- When the acquired signal measures more than the filtered threshold(during a pulse), we light the charlieplex using the emojibit class.
- when the acquired signal measures below the filtered signal, we clear the display.


`Note: we have noticed that we get reletively good PPG signals when measuring data from the finger tips.`

