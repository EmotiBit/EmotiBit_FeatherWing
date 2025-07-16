# Description
- This tests verifies that the Packet Fixed Length Test data in the EmotiBit SD card does not contain any Debug Overloads under conditions where the test runs up to 512 iterations.
- The data is verified with the bash script, checking the SD card output
- Instructions to run this test can be found in the EmotiBit Test Protocols Document under Max Length Data Test

The following table shows commands for choosing the test type. A typical workflow will consist of:
1. Going into debug mode
2. Setting sendTestData to true
3. Choosing the Splitter test '@'
4. Pressing record in the oscilliscope and waiting for the test to finish
5. Comparing the SD card result with the bash script, specifying the extension

| Command | Details |
|--------|--------|
| Z | Turns on "splitter" indicator, which is a "S" that is printed after each split|
| z | Turns off "splitter" indicator|
| < | Sets "sendTestData" to true|
| > | Sets "sendTestData" to false|
| @ | Sets test data to Splitter| 