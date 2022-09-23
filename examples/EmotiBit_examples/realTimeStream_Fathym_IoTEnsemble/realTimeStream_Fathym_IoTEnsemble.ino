#include "Esp32MQTTClient.h"
#include "EmotiBit.h"

#define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
#define MESSAGE_MAX_LEN 512
const uint32_t SERIAL_BAUD = 2000000; //115200

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];
static bool hasIoTHub = false;
//uint32_t dataTime;
//unsigned long Epoch_Time;
//const char* ntpServer = "pool.ntp.org";
static const char* fathymConnectionString = "";
String fathymDeviceID = "emotibit";
char fathymReadings[][3] = {"PI", "PR", "PG"};
int readingsInterval = 5000;
char metadataTypeTags[3];

void onShortButtonPress()
{
  // toggle wifi on/off
  if (emotibit.getPowerMode() == EmotiBit::PowerMode::NORMAL_POWER)
  {
    emotibit.setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
    Serial.println("PowerMode::WIRELESS_OFF");
  }
  else
  {
    emotibit.setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
    Serial.println("PowerMode::NORMAL_POWER");
  }
}

void onLongButtonPress()
{
  emotibit.sleep();
}

EmotiBit::DataType loadDataTypeFromTypeTag(String typeTag) {
  if (typeTag == "AX"){
    EmotiBit::DataType dataType {EmotiBit::DataType::ACCELEROMETER_X};
    return dataType;
  } 
  else if (typeTag == "AY"){
    EmotiBit::DataType dataType {EmotiBit::DataType::ACCELEROMETER_Y};
    return dataType;
  }
  else if (typeTag == "AZ"){
    EmotiBit::DataType dataType {EmotiBit::DataType::ACCELEROMETER_Z};
    return dataType;
  }
  else if (typeTag == "GX"){
    EmotiBit::DataType dataType {EmotiBit::DataType::GYROSCOPE_X};
    return dataType;
  }
  else if (typeTag == "GY"){
    EmotiBit::DataType dataType {EmotiBit::DataType::GYROSCOPE_Y};
    return dataType;
  }
  else if (typeTag == "GZ"){
    EmotiBit::DataType dataType {EmotiBit::DataType::GYROSCOPE_Z};
    return dataType;
  }
  else if (typeTag == "MX"){
    EmotiBit::DataType dataType {EmotiBit::DataType::MAGNETOMETER_X};
    return dataType;
  }
  else if (typeTag == "MY"){
    EmotiBit::DataType dataType {EmotiBit::DataType::MAGNETOMETER_Y};
    return dataType;
  }
  else if (typeTag == "MZ"){
    EmotiBit::DataType dataType {EmotiBit::DataType::MAGNETOMETER_Z};
    return dataType;
  }
  else if (typeTag == "EA"){
    EmotiBit::DataType dataType {EmotiBit::DataType::EDA};
    return dataType;
  }
  else if (typeTag == "EL"){
    EmotiBit::DataType dataType {EmotiBit::DataType::EDL};
    return dataType;
  }
  else if (typeTag == "ER"){
    EmotiBit::DataType dataType {EmotiBit::DataType::EDR};
    return dataType;
  }
  else if (typeTag == "H0"){
    EmotiBit::DataType dataType {EmotiBit::DataType::HUMIDITY_0};
    return dataType;
  }
  else if (typeTag == "T0"){
    EmotiBit::DataType dataType {EmotiBit::DataType::TEMPERATURE_0};
    return dataType;
  }
  else if (typeTag == "TH"){
    EmotiBit::DataType dataType {EmotiBit::DataType::THERMOPILE};
    return dataType;
  }
  else if (typeTag == "PI"){
    EmotiBit::DataType dataType {EmotiBit::DataType::PPG_INFRARED};
    return dataType;
  }
  else if (typeTag == "PR"){
    EmotiBit::DataType dataType {EmotiBit::DataType::PPG_RED};
    return dataType;
  }
  else if (typeTag == "PG"){
    EmotiBit::DataType dataType {EmotiBit::DataType::PPG_GREEN};
    return dataType;
  }
  else if (typeTag == "BV"){
    EmotiBit::DataType dataType {EmotiBit::DataType::BATTERY_VOLTAGE};
    return dataType;
  }
  else if (typeTag == "BP"){
    EmotiBit::DataType dataType {EmotiBit::DataType::BATTERY_PERCENT};
    return dataType;
  }
  else if (typeTag == "DO"){
    EmotiBit::DataType dataType {EmotiBit::DataType::DATA_OVERFLOW};
    return dataType;
  }
  else if (typeTag == "DC"){
    EmotiBit::DataType dataType {EmotiBit::DataType::DATA_CLIPPING};
    return dataType;
  }
  else if (typeTag == "DB"){
    EmotiBit::DataType dataType {EmotiBit::DataType::DEBUG};
    return dataType;
  }
  else{
    EmotiBit::DataType dataType {EmotiBit::DataType::DEBUG};
    return dataType;
  }  
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Serial started");
  delay(2000);  // short delay to allow user to connect to serial, if desired

  emotibit.setup();

  //if (!loadConfigFile(emotibit._configFilename)) {
  //Serial.println("SD card configuration file parsing failed.");
  //Serial.println("Create a file 'config.txt' with the following JSON:");
  //Serial.println("{\"WifiCredentials\": [{\"ssid\": \"SSSS\", \"password\" : \"PPPP\"}],\"Fathym\":{\"ConnectionString\": \"xxx\", \"DeviceID\": \"yyy\"}}");
  //}
  //configTime(0, 0, ntpServer);
  
  if (!Esp32MQTTClient_Init((const uint8_t*)fathymConnectionString, true))
  {
    hasIoTHub = false;
    Serial.println("Initializing IoT hub failed.");
    return;
  }

  hasIoTHub = true;

  // Attach callback functions
  emotibit.attachShortButtonPress(&onShortButtonPress);
  emotibit.attachLongButtonPress(&onLongButtonPress);
}

void loop()
{
  emotibit.update();
  
  // allocate the memory for the document
  const size_t CAPACITY = JSON_OBJECT_SIZE(1);
  
  StaticJsonBuffer<500> doc;
  
  JsonObject& payload = doc.createObject();

  payload[String("DeviceID")] = fathymDeviceID;

  payload["DeviceType"] = "emotibit";

  payload["Version"] = "1";

  // object["Timestamp"] = timeClient.getFormattedDate();

  JsonObject& payloadDeviceData = payload.createNestedObject("DeviceData");

  //payloadDeviceData["EpochTime"] = timeclient.getEpochTime();

  //payloadDeviceData["Timestamp"] = timeclient.getFormattedDate();

  JsonObject& payloadSensorReadings = payload.createNestedObject("SensorReadings");

  JsonObject& payloadSensorMetadata = payload.createNestedObject("SensorMetadata");

  JsonObject& payloadSensorMetadataRoot = payloadSensorMetadata.createNestedObject("_");
 
  for (auto typeTag : fathymReadings) {
      
    enum EmotiBit::DataType dataType = loadDataTypeFromTypeTag(String(typeTag));
    size_t dataAvailable = emotibit.readData((EmotiBit::DataType)dataType, &data[0], dataSize);
        
    if (dataAvailable > 0)
    {
      payloadSensorReadings[String(typeTag)] = String(data[dataAvailable - 1]);
    }
  }
  
  char messagePayload[MESSAGE_MAX_LEN];

  // serialize the payload for sending
  payload.printTo(messagePayload);

  Serial.println(messagePayload);

  EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);

  Esp32MQTTClient_SendEventInstance(message);

  delay(readingsInterval);
}

// Loads the configuration from a file
//bool loadConfigFile(const char *filename) {
  // Open file for reading
  //File file = SD.open(filename);

  //if (!file) {
    //Serial.print("File ");
    //Serial.print(filename);
    //Serial.println(" not found");
    //return false;
  //}

  //Serial.print("Parsing: ");
  //Serial.println(filename);

  // Allocate the memory pool on the stack.
  // Don't forget to change the capacity to match your JSON document.
  // Use arduinojson.org/assistant to compute the capacity.
  //StaticJsonBuffer<1024> jsonBuffer;

  // Parse the root object
  //JsonObject &root = jsonBuffer.parseObject(json);

  //if (!root.success()) {
    //Serial.println(F("Failed to parse config file"));
    //setupFailed("Failed to parse Config fie contents");
    //return false;
  //}

  //size_t configSize;
  //configSize = root.get<JsonVariant>("Fathym").as<JsonArray>().size();

  //try{
  //for(size_t i =0; i < configSize; i++) {
  //fathymConnectionString = root["Fathym"]["ConnectionString"];
  //Serial.println("CS : ");
  //Serial.println(fathymConnectionString);
  //fathymDeviceID = root["Fathym"]["DeviceID"];
  //Serial.println("DeviceID : ");
  //Serial.println(fathymDeviceID);

  //int fathymReadingsCount = root["Fathym"]["Readings"].as<JsonArray>().copyTo(fathymReadings);
  //}}
  //catch(int ex){
  //Serial.println("Error");
  //}

  //const char* fathymReadings[MAX_READINGS_COUNT];

  //Serial.println("Readings : ");
  //Serial.println(fathymReadings);



  //try {
  //strlcpy(fathymConnectionString,
  //doc["Fathym"]["ConnectionString"],
  //sizeof(fathymConnectionString));

  //int counter = 0;

  //for (char reading[3] : doc["Fathym"]["Readings"] ){
  //strlcpy(fathymReadings[0][counter],
  //reading,
  //sizeof(fathymReadings[0][counter]));

  //counter++;
  //}

  //strlcpy(fathymDeviceID,
  //doc["Fathym"]["DeviceID"],
  //sizeof(fathymDeviceID));

  //fathymConnectionString = doc["Fathym"]["ConnectionString"];

  //fathymReadings = doc["Fathym"]["Readings"];

  //fathymDeviceID = doc["Fathym"]["DeviceID"];

  //if(readingsInterval = doc["Fathym"]["ReadingsInterval"] ==  NULL)
  //readingsInterval = 5000;
  //else
  //readingsInterval = doc["Fathym"]["ReadingsInterval"];
  //}

  //catch (int num) {
  //Serial.println(F("Missing properties from config file"));
  //}

  // Close the file (File's destructor doesn't close the file)
  // ToDo: Handle multiple credentials

  //file.close();
  //return true;
//}
