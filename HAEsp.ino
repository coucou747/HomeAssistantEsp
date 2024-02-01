/*
 * Fournit sans garantie.
 * Attention, ce code agit sur des choses physique
 * comme de la domotique, ça peut piloter des radiateurs ou autre
 * Donc attention : en cas de bug, ça peut vraiment être très très chiant.
 * 
 * Auteur : Maxime Audouin <coucou747@gmail.com>
 * Code open source.
*/

///// CONFIGS

#define VERSION "v0.0.1"

#define PIN_MOTION D5 // 0
#define FEATURE_MOTION
#define FEATURE_DHT
#define DHT_PIN D2 //2
#define DHT_TYPE DHT22

#define FEATURE_RELAY
const int relay_enabled[] = { D7 };



/* config pour mes D1 litle capteurs température, humidité, présence. */
/*
#define FEATURE_LIGHTSENSOR
#define PIN_LIGHTSENSOR A0

#define FEATURE_DHT
#define DHT_PIN D2
#define DHT_TYPE DHT22

#define FEATURE_MOTION
#define PIN_MOTION D6
*/

/*
// Config pour sonoff : généric esp8285
#define FEATURE_RELAY
const int relay_enabled[] = { 12 };
// rester appuyé sur le bouton reset,
// fils : RIEN // GND //  RX //   TX //  3V
*/


///// FIN CONFIG

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "wificonfig.h"
#include <ArduinoJson.h>

#ifdef FEATURE_DHT
  #include <DHT.h>
  DHT dht(DHT_PIN, DHT_TYPE);
#endif



String features =
#ifdef FEATURE_DHT
  #ifdef FEATURES
  + String("_DHT")
  #else
    #define FEATURES
    "DHT"
  #endif
#endif

#ifdef FEATURE_RELAY
  #ifdef FEATURES
    + String("_RELAY")
  #else
    #define FEATURES
    "RELAY"
  #endif
#endif

#ifdef FEATURE_MOTION
  #ifdef FEATURES
    + String("_PIR")
  #else
    #define FEATURES
    "PIR"
  #endif
#endif

#ifndef FEATURES
  #define FEATURES "None"
#endif
;


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
String hardwareName;

#ifdef FEATURE_RELAY
void config_relais(String hardwareName, String indice, JsonObject device){
  DynamicJsonDocument rjsb(2048);
  rjsb["name"] = "relais "+indice;
  rjsb["object_id"] = hardwareName;
  rjsb["uniq_id"] = hardwareName+"_"+indice;
   //rjsb["state_topic"] = HPREFIX+"/switch/"+hardwareName+"/"+indice+"/state"; // on a pas la place pour mettre ça... mqttclient est un peu pourrit
  String cmdTopic = HPREFIX+"/switch/"+hardwareName+"/"+indice+"/set";
  rjsb["command_topic"] = cmdTopic;
  rjsb["device_class"] = "switch";
  rjsb["payload_on"] = "ON";
  rjsb["payload_off"] = "OFF";
  rjsb["state_on"] = "ON";
  rjsb["state_off"] = "OFF";
  rjsb["retain"] = "True";
  rjsb["device"] = device;
  char data[2048];
  serializeJson(rjsb, data);
  int len = strlen(data);
  mqttClient.beginPublish((HPREFIX+"/switch/"+hardwareName+"_"+indice+"/config").c_str(), len, false);
  mqttClient.write((const uint8_t *)data, len);
  mqttClient.endPublish();
  mqttClient.subscribe(cmdTopic.c_str());
}
#endif

#ifdef FEATURE_MOTION
void config_pir(String hardwareName, JsonObject device){
  DynamicJsonDocument rjsb(2048);
  rjsb["name"] = "PIR";
  rjsb["object_id"] = hardwareName;
  rjsb["uniq_id"] = hardwareName+"_PIR";
  //rjsb["val_tpl"] = "{{value_json['motion']}}";
  rjsb["state_topic"] = HPREFIX+"/binary_sensor/"+hardwareName+"_PIR";
  rjsb["payload_on"] = "ON";
  rjsb["payload_off"] = "OFF";
  rjsb["device_class"] = "presence";
  rjsb["away_timeout"] = 10;
  rjsb["retain"] = "True";
  rjsb["device"] = device;
  char data[2048];
  serializeJson(rjsb, data);
  int len = strlen(data);
  mqttClient.beginPublish((HPREFIX+"/binary_sensor/"+hardwareName+"_PIR/config").c_str(), len, false);
  mqttClient.write((const uint8_t *)data, len);
  mqttClient.endPublish();
}
#include "Schedule.h"
int last_motion;
void detectsMovement_AUX() {
  const char* msg = last_motion ? "ON" : "OFF";
  
  int len = strlen(msg);
  mqttClient.beginPublish((HPREFIX+"/binary_sensor/"+hardwareName+"_PIR").c_str(), len, false);
  mqttClient.write((const uint8_t *)msg, len);
  mqttClient.endPublish();
  Serial.printf("interruption done\n");
}
void ICACHE_RAM_ATTR detectsMovement() {
  last_motion = digitalRead(PIN_MOTION) == HIGH ? 1 : 0;
  Serial.printf("MOTION DETECTED ? %d\n", last_motion);
  schedule_function(detectsMovement_AUX);
}
#endif

void config_device(String hardwareName, String device_class, String unit_, JsonObject device){
  DynamicJsonDocument rjsb(2048);
  rjsb["name"] = device_class;
  rjsb["object_id"] = hardwareName;
  rjsb["uniq_id"] = hardwareName+"_"+device_class;
  rjsb["expire_after"] = 120;
  rjsb["device_class"] = device_class;
  rjsb["val_tpl"] = "{{value_json['"+device_class+"']}}";
  rjsb["unit_of_meas"] = unit_;
  rjsb["state_topic"] = HPREFIX+"/sensor/"+hardwareName+"/DTH";
  rjsb["retain"] = "True";
  rjsb["device"] = device;
  char data[1024];
  serializeJson(rjsb, data);
  int len = strlen(data);
  mqttClient.beginPublish((HPREFIX+"/sensor/"+hardwareName+"_"+device_class+"/config").c_str(), len, false);
  mqttClient.write((const uint8_t *)data, len);
  mqttClient.endPublish();
}

#ifdef FEATURE_RELAY
void onMqttMessage(char* topic_, byte* payload_, unsigned int length) {

  char* payload = (char*)malloc(length + 1);
  memcpy(payload, payload_, length);
  payload[length] = 0;
  
  String topic = String(topic_);
  Serial.print("Received a message with topic '");
  Serial.println(topic);
  Serial.print("And Payload");
  Serial.println((char*)payload);
  int pos = topic.lastIndexOf("/");
  int pos2 = topic.lastIndexOf("/", pos - 1);
  int id = topic.substring(pos2 + 1, pos).toInt();

  if (strcmp((char *)payload, "ON") == 0){
    digitalWrite(relay_enabled[id], HIGH);
  }else if (strcmp((char *)payload,"OFF") == 0){
    digitalWrite(relay_enabled[id], LOW);
  }
  free(payload);
}
#endif

void setup() {
  Serial.begin(115200);
  int hardwareID = ESP.getChipId();
  hardwareName = "esp_" + String(hardwareID);
  Serial.println("Device name : "+hardwareName);
  const char* hname = ("esp " + String(hardwareID) + " " + VERSION + " " + features).c_str();
  WiFi.hostname(hname);
  ArduinoOTA.setHostname(hname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(host, port);
  if (!mqttClient.connect(hardwareName.c_str())) {
    Serial.println("MQTT connection Failed! Rebooting...");
    ESP.restart();
  }
  
  
  #ifdef FEATURE_RELAY
  mqttClient.setCallback(onMqttMessage);
  #endif
  Serial.println("You're connected to the MQTT broker!");

  DynamicJsonDocument rjsb(2048);
  JsonObject device = rjsb.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(ESP.getChipId());
  device["manufacturer"] = "Esp";
  device["model"] = ESP.getSdkVersion();
  device["name"] = hardwareName;

  #ifdef FEATURE_DHT
    dht.begin();
    config_device(hardwareName, "temperature", "C", device);
    config_device(hardwareName, "humidity", "%", device);
  #endif
  #ifdef FEATURE_RELAY
    int count_relay = sizeof(relay_enabled) / sizeof(relay_enabled[0]);
    Serial.println("Setting " + String(count_relay) + " relays");
    for (int i = 0; i < count_relay; i ++) {
      pinMode(relay_enabled[i], OUTPUT);
      digitalWrite(relay_enabled[i], LOW);
      config_relais(hardwareName, (String)i, device);
    }
  #endif
  #ifdef FEATURE_MOTION
  config_pir(hardwareName, device);
  pinMode(PIN_MOTION, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MOTION), detectsMovement, CHANGE);
  #endif

}

const long interval = 1000*60;
unsigned long previousMillis = -interval;

void loop() {
  ArduinoOTA.handle();
  mqttClient.loop();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
#ifdef FEATURE_DHT
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    Serial.println("température : "+(String)temperature);

    DynamicJsonDocument rjsb(1024);
    rjsb["temperature"] = (String)temperature;
    rjsb["humidity"] = (String)humidity;
    char data[200];
    serializeJson(rjsb, data);
    int len = strlen(data);
    mqttClient.beginPublish( (HPREFIX+"/sensor/"+hardwareName+"/DTH").c_str(), len, false);
    mqttClient.write((const uint8_t *)data, len);
    mqttClient.endPublish();
#endif
  }
}
