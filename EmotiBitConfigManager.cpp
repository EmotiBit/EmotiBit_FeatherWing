#include <EmotiBitConfigManager.h>

#ifdef ARDUINO_FEATHER_ESP32
bool EmotiBitConfigManager::init(fs::SDFS* sd)
{
	SD = sd;
	if(SD != nullptr)
	{
		return true;
	}
	else
	{
		return false;
	}
}
#else
bool EmotiBitConfigManager::init(SdFat* sd)
{
	SD = sd;
	if(SD != nullptr)
	{
		return true;
	}
	else
	{
		return false;
	}
}
#endif

bool EmotiBitConfigManager::createNewConfigFile(String configFilename, File& file, JsonDocument& configAsJson)
{
#if defined ARDUINO_FEATHER_ESP32
	file = SD->open(configFilename, FILE_WRITE);
#else 
	file = SD->open(configFilename, O_CREAT | O_WRITE);
#endif
	if(file)
	{
		if( serializeJsonPretty(configAsJson, file) == 0)
		{
			Serial.println("Failed to write to file");
			EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_ADD);
			return false;
		}
		file.close();
		EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::ACK, EmotiBitPacket::TypeTag::WIFI_ADD);
	}
	else
	{
		Serial.println("Failed to create file");
		EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_ADD);
		return false;
	}
	return true;
}

uint8_t EmotiBitConfigManager::loadConfigFile(const String &filename, JsonDocument& jsonDoc)
{
		// Open file for reading
	File file = SD->open(filename);

	if (!file) {
		Serial.print("File ");
		Serial.print(filename);
		Serial.println(" not found");
		return (uint8_t) Status::FILE_NOT_FOUND;
	}

	// Parse the root object
	DeserializationError error = deserializeJson(jsonDoc, file);
	file.close();

	if (error) {
		Serial.println(F("Failed to parse config file"));
		return (uint8_t) Status::FILE_PARSE_FAIL;
	}
	return Status::OK;
}

void EmotiBitConfigManager::updateWiFiCredentials(String emotibitFwVersion, String configFilename, const uint8_t maxCreds)
{
	// Send EmotiBit Firmware version to host
	EmotiBitSerial::sendMessage(EmotiBitFactoryTest::TypeTag::FIRMWARE_VERSION, emotibitFwVersion);
	// instructions
	Serial.println("Options available:");
	Serial.println("1. Add a wifi credential. Send: @WA,{\"ssid\":\"SSSS\",\"password\" : \"PPPP\"}~");
	Serial.println("  - Replace SSSS with network name and PPPP with network password");
	Serial.println("2. List wifi credentials. Send: @LS~");
	Serial.println("3. Delete Wifi Credential. Send: @WD,<credential number>~");
	Serial.println("  - use List option to get a wifi list. Then use the number in the list to delete that credential");
	Serial.println("4. Reset device. Send @RS~");
	Serial.println("  - Restart the device after editing config file");

	while (1) 
	{
		if (Serial.available())
		{
			String msg = Serial.readStringUntil(EmotiBitSerial::MSG_TERM_CHAR); // is ~ is not present, readStringUntil() break after timeout
			msg += EmotiBitSerial::MSG_TERM_CHAR;
			String typetag, payload;
			bool serialParsed = EmotiBitSerial::parseSerialMessage(msg, typetag, payload);
			if(serialParsed)
			{
				File file;
				StaticJsonDocument<1024> configAsJson;
				DeserializationError parseError;
				bool fileExists = false, fileParses = false;
				fileExists = SD->exists(configFilename);
				if(fileExists)
				{
					file = SD->open(configFilename);
					parseError = deserializeJson(configAsJson, file); 
					file.close();
					if(parseError)
					{
						// remove broken config file
						Serial.println("Config file parse failed. Removing file.");
						SD->remove(configFilename);
						fileExists = false;
					}
				}

				if (typetag.compareTo(EmotiBitPacket::TypeTag::WIFI_ADD) == 0)
				{
					String ssid, password, username, userid;
					StaticJsonDocument<256> inputJson;
					DeserializationError error = deserializeJson(inputJson, payload);
					if(error)
					{
						// user input cannot be parsed
						Serial.println("Try again. Cannot parse JSON input.");
						Serial.println("Expected format: WA,{\"ssid\":\"SSSS\",\"password\" : \"PPPP\"}");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_ADD);
						continue;
					}
					else
					{
						// user input parsed correctly	
						ssid = inputJson["ssid"] | "";
						password = inputJson["password"] | "";
						username = inputJson["username"] | "";
						userid = inputJson["userid"] | "";
						//Serial.println("Parsed output: ");
						Serial.print("Adding:: ");
						Serial.print("ssid: " + ssid); Serial.print(" | ");
						Serial.print("password: " + password);
						username.compareTo("")!= 0 ? Serial.println("username: " + username) : Serial.println();
						userid.compareTo("")!= 0 ? Serial.println("userid: " + userid) : Serial.println();
						if(!fileExists)
						{
							// either config file does not exist or parse failed and it was deleted
							Serial.println("Creating new config file");
							// create new file
							configAsJson.clear();
							JsonArray wifiCreds = configAsJson.createNestedArray("WifiCredentials");
							JsonObject cred = wifiCreds.createNestedObject();
							cred["ssid"] = ssid;
							cred["password"] = password;
							if (userid.compareTo("") != 0)
							{
								cred["userid"] = userid;
							}
							if (username.compareTo("") != 0)
							{
								cred["username"] = username;
							}
							bool status = createNewConfigFile(configFilename, file, configAsJson);
							if (!status)
							{
								// error messages are printed in the called function
								continue;
							}
						}
						else
						{
							// config file present!
							Serial.println("config file exists. Appending to file.");
							// file parsed successfully
							if(configAsJson["WifiCredentials"].size() >= maxCreds)
							{
								Serial.println("Config file already has max num allowed creds. Please delete before adding");
								EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_ADD);
								continue;
							}
							else
							{
								JsonObject cred = configAsJson["WifiCredentials"].createNestedObject();
								cred["ssid"] = ssid;
								cred["password"] = password;
								if(userid.compareTo("") != 0)
								{
									cred["userid"] = userid;
								}
								if(username.compareTo("") != 0)
								{
									cred["username"] = username;
								}
								bool status = createNewConfigFile(configFilename, file, configAsJson);
								if (!status)
								{
									// error messages are printed in the called function
									continue;
								}
								else
								{
									// success. Consider sending ACK response, if it does ont affect UX on teh serial monitor
								}
							}				
						}
						// read file contents
						Serial.println("Updated file contents:");
						file = SD->open(configFilename);
						configAsJson.clear();
						error = deserializeJson(configAsJson, file);
						serializeJsonPretty(configAsJson, Serial);
						file.close();
						Serial.println();
					}
				}
				else if (typetag.compareTo(EmotiBitPacket::TypeTag::WIFI_DELETE) == 0)
				{
					if(!fileExists)
					{
						Serial.println("config file does not exist. Aborting delete");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
						continue;
					}
					if(parseError)
					{
						Serial.println("Cannot parse JSON file. Aborting delete");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
						continue;
					}
					else
					{
						// file can be parsed!
						if(payload.compareTo("") != 0)
						{
							// delete requested credential
							int deleteIndex = payload.toInt();
							if(deleteIndex == 0 && payload.compareTo("0") != 0)
							{
								// toInt() tried to convert invalid string and returned 0 by default
								Serial.println("invalid option. Please be careful to avoid blank spaces.");
								EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
								continue;
							}
							
							if(deleteIndex < configAsJson["WifiCredentials"].size())
							{
								Serial.print("Deleting entry: " + String(deleteIndex) + ". "); 
								Serial.println(configAsJson["WifiCredentials"][deleteIndex]["ssid"].as<String>());
								configAsJson["WifiCredentials"].remove(deleteIndex);
								SD->remove(configFilename);
								bool status = createNewConfigFile(configFilename, file, configAsJson);
								if(status)
								{
									EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::ACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
								}
								else
								{
									EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
								}
							}
							else
							{
								Serial.println("Out of bounds delete. Enter a valid choice.");
								EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
							}
						}
						else
						{
							Serial.println("Delete needs a parameter. Call LS to get current credential list.");
							EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::WIFI_DELETE);
						}
					}
				}
				else if (typetag.compareTo(EmotiBitPacket::TypeTag::LIST) == 0)
				{
					if(!fileExists)
					{
						Serial.println("config file does not exist.");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::LIST);
						continue;
					}
					if(parseError)
					{
						Serial.println("Cannot parse config file. Aborting LIST.");
						EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::NACK, EmotiBitPacket::TypeTag::LIST);
						continue;
					}
					if(configAsJson["WifiCredentials"].size())
					{
						Serial.println("##################################");
						Serial.println("config file credentials:");
						for(int i = 0; i < configAsJson["WifiCredentials"].size(); i++ )
						{
							Serial.print(i); Serial.print(". " + configAsJson["WifiCredentials"][i]["ssid"].as<String>());
							Serial.print(" : "); Serial.print(configAsJson["WifiCredentials"][i]["password"].as<String>());
							Serial.println();
						}
						Serial.println("##################################");
					}
					else
					{
						Serial.println("Config file has no credentials. Try WA to add credential first.");
					}
					EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::ACK, EmotiBitPacket::TypeTag::LIST);

				}
				else if (typetag.compareTo(EmotiBitPacket::TypeTag::RESET) == 0)
				{
					EmotiBitSerial::sendMessage(EmotiBitPacket::TypeTag::ACK, EmotiBitPacket::TypeTag::RESET);
					Serial.println("Restarting MCU");
					delay(1000);
					#ifdef ARDUINO_FEATHER_ESP32
						ESP.restart();
					#elif defined ADAFRUIT_FEATHER_M0
						NVIC_SystemReset();
					#endif
				} 
			}
			else
			{
				Serial.println("Not a valid serial message. Expecting: @TYPETAG,PAYLOAD~ or @TYPETAG~");
			}
			
		}
	}
}