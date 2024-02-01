//#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include "EspMQTTClient.h"
#include <FS.h>
#include <ESP8266FtpServer.h>
#include "ArduinoJson.h"
#include "Arduino.h"
#include "string.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define PUB_DELAY (5 * 1000) // 5 second
#define LATCH_PIN 4
#define CLOCK_PIN 5
#define DATA_PIN 0
#define CONFIG_START 32
#define CONFIG_VERSION "00001"
#define DS18B20_BUS 2

const char* ssidAP = "myESP";
const char* passwordAP = "12345678";
char Ip[50];
uint16_t Port;
char ClientName[30];
 
String JSONdata;
byte count = 0;
byte relays1_8 = 255;
byte relays9_16 = 255;
int relays1_16 = relays1_8 | (relays9_16<<8);
long last = 0;

FtpServer ftpSrv;
OneWire ds18b20(DS18B20_BUS);
DallasTemperature temp_sensor(&ds18b20);
EspMQTTClient client;

typedef struct {
  char version[6];      // Version of the configuration in EEPROM, used in case we change the struct
  uint8_t debug;        // Debug on yes/no 1/0
  char nodename[32];    // this node name
  char mqttip[32];      // Mosquitto server IP
  uint16_t mqttport;    // Mosquitto port
  char mqttuser[32];    // Mosquitto Username (if needed, or "")
  char mqttpass[64];    // Moqsuitto Password (if needed, or "")
} configuration_t;

/*
  Declare a default configuration_t to use in case there is no EEPROM data, otherwise this gets
  overwritten by whatever is in EEPROM
 */
  configuration_t CONFIGURATION;


void MakeAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssidAP, passwordAP);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("\nMy IP to connect via Web-Browser or FTP: ");
  Serial.println(myIP);
  Serial.println("\n");
  Serial.print("ssid: ");
  Serial.println(ssidAP);
  Serial.print("password: ");
  Serial.println(passwordAP);
}

int loadConfig() {
  // validate its the correct version (and therefore the same struct...)
  if (EEPROM.read(CONFIG_START + 0) == CONFIGURATION.version[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIGURATION.version[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIGURATION.version[2] &&
      EEPROM.read(CONFIG_START + 3) == CONFIGURATION.version[3] &&
      EEPROM.read(CONFIG_START + 4) == CONFIGURATION.version[4]) {
    // and if so read configuration into struct
    EEPROM.get(CONFIG_START, CONFIGURATION);
    return 1;
  }
  return 0;
}

/*
  Save the current CONFIGURATION definition to EEPROM
 */
void saveConfig() {
  EEPROM.put(CONFIG_START, CONFIGURATION);
  EEPROM.commit();
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(sizeof(configuration_t) + CONFIG_START);
  SPIFFS.begin();
  temp_sensor.begin();    
  File file = SPIFFS.open("/config.json", "r");
  if (!file) {
  Serial.println("file /config.json open fieled ");
  }

  JSONdata = file.readString();
  StaticJsonDocument<1024> net;
  deserializeJson(net, JSONdata);
  const char* conf_version = net["config_version"];
  const char* ssid = net["ssid"];
  const char* password = net["password"];
  const char* mqttServIp = net["mqttServerIp"];
  int mqttServPort = net["mqttServerPort"];
  const char* mqttClntName = net["mqttClientName"];

  strcpy(CONFIGURATION.version, conf_version);
  CONFIGURATION.debug = 1;
  strcpy(CONFIGURATION.nodename, mqttClntName);
  strcpy(CONFIGURATION.mqttip, mqttServIp);
  CONFIGURATION.mqttport = mqttServPort;
  strcpy(CONFIGURATION.mqttuser, "");
  strcpy(CONFIGURATION.mqttpass, "");
   
  if (!loadConfig()) {
    saveConfig();
  }
  // enable debug messages if our configuration tells us to
  if (CONFIGURATION.debug)
    client.enableDebuggingMessages();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (count > 16){
    Serial.println("WiFi connect fieled");
    MakeAP();
    break;
    }
    count += 1;
    Serial.print(".");
  }
  if (WL_CONNECTED){
    Serial.println("");
    Serial.print("IP addres: ");
    Serial.println(WiFi.localIP());
  }
  
  ftpSrv.begin("relay","relay");
  Serial.println("FTPS login/pass: ");
  Serial.println("relay/relay");

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  Serial.println(mqttServIp + String(mqttServPort) + mqttClntName);

  client.setMqttClientName(CONFIGURATION.nodename);
  client.setMqttServer(CONFIGURATION.mqttip, CONFIGURATION.mqttuser, CONFIGURATION.mqttpass, CONFIGURATION.mqttport);
  EEPROM.end();
}
  
 
void printConstChar(const char *text){
  Serial.println(text); 
}
void printUint16_t(const uint16_t port){
  Serial.println(String(port));
}
  
void publishStatus(){
  Serial.println("status");
  
  StaticJsonDocument<1024> statRelay;
  for (byte i=0; i<16; i++) {
    String nameRelay = "r"+String(i+1);
    byte valRelay = bitRead(relays1_16, i);
    statRelay[nameRelay] = valRelay;
  }  
  String payload;
  serializeJson(statRelay, payload);
  client.publish("automatic/relay", payload);

  Serial.println(".....");
  Serial.println(relays1_16, BIN);
  Serial.println(".....");
}

void publishTemperature() {
  long now = millis();
  if (client.isConnected() && (now - last > PUB_DELAY)) {
    temp_sensor.requestTemperatures();
    float tempC = temp_sensor.getTempCByIndex(0);
    Serial.println(tempC);
    client.publish("automatic/temperature", String(tempC));
    printConstChar(Ip);
    printUint16_t(Port);
    printConstChar(ClientName);
    last = now;
  }
}

void onConnectionEstablished() {

  client.subscribe("automatic/set/status", [] (const String &stat)  {
    publishStatus();
  });

  client.subscribe("automatic/set/relay", [] (const String &payload)  {
    bitToRelay(payload);
    publishStatus();
  });

}

void bitToRelay(const String &payload) {  
  StaticJsonDocument<1024> data;
  Serial.println(payload);  
  deserializeJson(data, payload);
  byte numRelay = data["nr"];
  byte valJson = data["val"];
  Serial.println(String(numRelay) +":" + String(valJson));
  byte numBit = numRelay - 1;
  bitWrite(relays1_16, numBit, valJson);
  setRelay();
  Serial.println("________________");
  Serial.println(relays1_16, BIN);
  Serial.println("________________");
}

void setRelay(){
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST,relays1_16);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST,(relays1_16 >> 8));
  digitalWrite(LATCH_PIN, HIGH);
}

void loop() {
  ftpSrv.handleFTP();
  client.loop();
  publishTemperature();
}
