/*
 * Fournit sans garantie.
 * Attention, ce code agit sur des choses physique
 * comme de la domotique, ça peut piloter des radiateurs ou autre
 * Donc attention : en cas de bug, ça peut vraiment être très très chiant.
 * 
 * Autheur : Maxime Audouin <coucou747@gmail.com>
 * Code open source.
*/



///// DEBUG CONFIGS

#define VERSION "v0.0.1"

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


// Config pour sonoff : généric esp8285
#define FEATURE_RELAY
const int relay_enabled[] = { 12 };
// rester appuyé sur le bouton reset,
// fils : RIEN // GND //  RX //   TX //  3V



///// FIN CONFIG

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoMqttClient.h>
#include <ArduinoOTA.h>
#include "wificonfig.h"
#include <ArduinoJson.h>

#ifdef FEATURE_DHT
  #include <DHT.h>
  DHT dht(DHT_PIN, DHT_TYPE);
  #ifdef FEATURES
    #define FEATURES FEATURES##"_DHT"
  #else
    #define FEATURES "DHT"
  #endif
#endif

#ifdef FEATURE_RELAY
  #ifdef FEATURES
    #define FEATURES FEATURES##"_RELAY"
  #else
    #define FEATURES "RELAY"
  #endif
#endif

#ifndef FEATURES
  #define FEATURES "None"
#endif


WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
String hardwareName;

#ifdef FEATURE_RELAY
void config_relais(String hardwareName, String indice){
  DynamicJsonDocument rjsb(2048);
  rjsb["name"] = hardwareName;
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
  char data[2048];
  serializeJson(rjsb, data);
  mqttClient.beginMessage(HPREFIX+"/switch/"+hardwareName+"_"+indice+"/config");
  mqttClient.print(data);
  mqttClient.endMessage();
  mqttClient.subscribe(cmdTopic);
}
#endif

void config_device(String hardwareName, String device_class, String unit_){
  DynamicJsonDocument rjsb(1024);
  rjsb["name"] = hardwareName;
  rjsb["uniq_id"] = hardwareName+"_"+device_class;
  rjsb["expire_after"] = 120;
  rjsb["device_class"] = device_class;
  rjsb["val_tpl"] = "{{value_json['"+device_class+"']}}";
  rjsb["unit_of_meas"] = unit_;
  rjsb["state_topic"] = HPREFIX+"/sensor/"+hardwareName+"/DTH";
  rjsb["retain"] = "True";
  char data[1024];
  serializeJson(rjsb, data);
  mqttClient.beginMessage(HPREFIX+"/sensor/"+hardwareName+"_"+device_class+"/config");
  mqttClient.print(data);
  mqttClient.endMessage();
}

#ifdef FEATURE_RELAY
void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();
  Serial.print("Received a message with topic '");
  Serial.println(topic);
  int pos = topic.lastIndexOf("/");
  int pos2 = topic.lastIndexOf("/", pos - 1);
  int id = topic.substring(pos2 + 1, pos).toInt();
  
  char content[1024];
  int i = 0;
  while (mqttClient.available()) {
    content[i++] = (char)mqttClient.read();
  }
  if (strcmp(content, "ON") == 0){
    digitalWrite(relay_enabled[id], HIGH);
  }else if (strcmp(content,"OFF") == 0){
    digitalWrite(relay_enabled[id], LOW);
  }else{
    Serial.println("mauvais message " + (String)content);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  int hardwareID = ESP.getChipId();
  hardwareName = "esp_" + String(hardwareID);
  Serial.println("Device name : "+hardwareName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname(("esp " + String(hardwareID) + " " + VERSION + " " + FEATURES).c_str());
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

  if (!mqttClient.connect(host, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    delay(5000);
    ESP.restart();
  }

#ifdef FEATURE_RELAY
  mqttClient.onMessage(onMqttMessage);
#endif
  Serial.println("You're connected to the MQTT broker!");

  #ifdef FEATURE_DHT
    dht.begin();
    config_device(hardwareName, "temperature", "C");
    config_device(hardwareName, "humidity", "%");
  #endif
  #ifdef FEATURE_RELAY
    int count_relay = sizeof(relay_enabled) / sizeof(relay_enabled[0]);
    Serial.println("Setting " + String(count_relay) + " relays");
    for (int i = 0; i < count_relay; i ++) {
      pinMode(relay_enabled[i], OUTPUT);
      digitalWrite(relay_enabled[i], LOW);
      config_relais(hardwareName, (String)i);
    }
  #endif
}

const long interval = 1000*60;
unsigned long previousMillis = 0;

void loop() {
  ArduinoOTA.handle();
  mqttClient.poll();
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
    mqttClient.beginMessage(HPREFIX+"/sensor/"+hardwareName+"/DTH");
    mqttClient.print(data);
    mqttClient.endMessage();
#endif
  }
}
