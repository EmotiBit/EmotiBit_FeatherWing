# syncing_digitalInput.ino

## Description
Adds a data packet with the `SP` TypeTag for every change on specified pin (`A2`, a.k.a. `16` at time of writing, see See Feather M0 pinouts: https://learn.adafruit.com/adafruit-feather-m0-wifi-atwinc1500/pinouts). Each packet has a standard packet header (including a timestamp) and one data value indicating whether digitalRead() is high or low after the change.

## IMPORTANT NOTES
- You should NEVER attach EmotiBit to a wall-power source while someone is wearing EmotiBit to avoid shock risk. Optical or other electrical isolation with medical safety protections are critical. 
- Sharing a ground with other devices (e.g. EEG) may introduce substantial noise. Optical or other electrical isolation is advised if syncing is desired during physiological data recording.
- Digital input pins must never exceed the range GND-0.6v to VDD+0.6V (i.e. best to keep it within 0V-3.3V). Isolation circuitry or a resistor divider is critical for inputs greater than 3.3V to avoid damaging the Feather.
- It's recommended to use sync pulses that are longer than 200 milliseconds to minimize missed pulses (one second or more for each high and low states is even better). Usually SD card writes are fast on EmotiBit (<100msec) but occasionally can take a long time and sync pulses are not being buffered like other EmotiBit data streams. Thus, if two or more sync pulse changes occur while the SD card is in a long write cycle, the number of events will be captured, but the timestamps and high/low state will be overwritten. In that case, you'll get repeated `SP` data packets with the same timestamp and data value, indicating that additional sync pulse events were detected but not faithfully recorded in the data stream.
