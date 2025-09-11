# Description
- This tests verifies that the Packet Fixed Length Test data in the EmotiBit SD card does not go over the 512 max data length. If there are packets that go over that length, then it will be signified by Debug Overloads.
- The data is verified with the bash script, checking the SD card output

The following table shows commands for choosing the test type. A typical workflow will consist of:
1. Going into debug mode
2. Setting sendTestData to true
3. Choosing the Packet Fixed Length Test '@'
4. Pressing record in the oscilliscope and waiting for the test to finish
5. Comparing the SD card result with the bash script, specifying the extension

| Command | Details |
|--------|--------|
| > | Sets "sendTestData" to true|
| < | Sets "sendTestData" to false|
| @ | Sets test data to Packet Fixed Length Test| 