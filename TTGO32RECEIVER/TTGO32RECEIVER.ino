#include <SPI.h>
#include <LoRa.h>
#include "ThingsBoard.h"

#include <hwcrypto/aes.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#define LORA_MISO 19
#define LORA_CS 18
#define LORA_MOSI 27
#define LORA_SCK 5
#define LORA_RST 14
#define LORA_IRQ 26

// LoRa Band (change it if you are outside Europe according to your country)
#define LORA_BAND 868E6

#define ssid "SSID"
#define passwd "11111111"

#define thingsboardpw "Gpmy6mHqKsmeHOiu8eXZ"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

double lo, la;
int gid, hrt, spo2, hflg, spoflg;
WiFiClient espClient;
ThingsBoard tb(espClient);
// the Wifi radio's status
char lorabuf[300] = {0};
bool processing = false;
int status = WL_IDLE_STATUS;
void setup() {
  Serial.begin(9600);
  Serial.println("Setup LoRa Receiver....");
  initLoRa();
  WiFi.begin(ssid, passwd);
  InitWiFi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
  }
  delay(1000);

  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
  }

  if (!tb.connected()) {
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(thingsboardpw);
    if (!tb.connect(THINGSBOARD_SERVER, thingsboardpw)) {
      Serial.println("Failed to connect");
      return;
    }
  }

  Serial.println("Sending data...");

  // Uploads new telemetry to ThingsBoard using MQTT.
  // See https://thingsboard.io/docs/reference/mqtt-api/#telemetry-upload-api
  // for more details

  if (processing) {
    tb.sendTelemetryInt("ID", gid);
    if (lo != 0 && la != 0) {

      tb.sendTelemetryFloat("lon", (double)lo / 10000000);
      tb.sendTelemetryFloat("lat", (double)la / 10000000);
    }

    tb.sendTelemetryInt("hb", hrt);
    tb.sendTelemetryInt("hf", hflg);
    tb.sendTelemetryInt("spo2", spo2);
    tb.sendTelemetryInt("spof", spoflg);
    processing = false;
  }

  tb.loop();
}

void initLoRa() {
  Serial.println("Initializing LoRa....");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  // Start LoRa using the frequency
  int result = LoRa.begin(LORA_BAND);
  if (result != 1) {
    Serial.println("Failed to start LoRa network!");
    for (;;);
  }
  Serial.println("LoRa initialized");
  delay(2000);
  LoRa.onReceive(onReceive);

  // put the radio into receive mode
  LoRa.receive();
}

void onReceive(int packetSize) {
  // received a packet
  if (!processing) {
    DynamicJsonDocument json(300);
    Serial.print("Received packet '");
    memset(lorabuf, '\0', sizeof(lorabuf));
    // read packet
    for (int i = 0; i < packetSize; i++) {
      lorabuf[i] = LoRa.read();
    }
    Serial.print(lorabuf);
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    DeserializationError err = deserializeJson(json, lorabuf);
    switch (err.code()) {
      case DeserializationError::Ok:
        Serial.print(F("Deserialization succeeded"));
        la = json["gps"][0].as<long>();
        lo = json["gps"][1].as<long>();
        hflg = json["hf"].as<int>();
        spoflg = json["sf"].as<int>();
        hrt = json["h"].as<int>() ;
        spo2 = json["o"].as<int>();
        gid = json["i"].as<int>();
        processing = true;

        break;
      case DeserializationError::InvalidInput:
        Serial.print(F("Invalid input!"));
        break;
      case DeserializationError::NoMemory:
        Serial.print(F("Not enough memory"));
        break;
      default:
        Serial.print(F("Deserialization failed"));
        break;
    }
  }
}


void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(ssid, passwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(ssid, passwd);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}
