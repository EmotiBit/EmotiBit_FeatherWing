## Instructions:
1. Follow the instructions on [https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide?view=all](https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide?view=all).

  Just perform the steps required for **Arduino BSP Installation**.

2. After completing _step 1_ (mentioned above), All the necessary/required files will be added to the **Arduino IDE**.
3. In the Arduino IDE app, goto `Files>Examples`. Here, you will see, a new section containing all the examples relating to the nRF52.
4. The **Blink test** can be performed without requiring any additional libraries. A simple code just toggling _Pin 17_ can be uploaded to perform the blink test.
5. The `Test_En_low_effect` contains the code which was used to perform a test to check the effect of pulling down the **EN** pin of the arduino. It was found that If the EN is pulled LOW, The MCU Shuts Down.
6. The `nRF52_FunctionalBluetoothTest` contains the code to pair the feather with another bluetooth device. It is fully functional and can be used as a base to create the Bluetooth communication System.
