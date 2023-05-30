#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include "time.h"

String MACHINE = "1";

WiFiUDP Udp;

//Provide the informations on the following 4 lines below.
const char* ssid = "SSID";
const char* password = "PASSWORD";

const char* udpAddress = "IP ADDRESS";
unsigned int udpPort = PORT;

#define SENSOR_PIN 23

float WAIT_TIME = 2;  //minutes
const int CONSTANT = 5;

TaskHandle_t Task1;
TaskHandle_t Task2;

int counter = 0;
int temp = 0;

bool isSent = false;

float start_time = 0;
float end_time = 0;

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("ESP32 WIFI connected to Access Point");
}

void Get_IP_Address(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WIFI Connected!");
  Serial.print("IP Address of the device: ");
  Serial.println(WiFi.localIP());
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WIFI");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.print("Reconnecting ...");
  WiFi.begin(ssid, password);
}

void initWiFi() {
  WiFi.disconnect(true);
  delay(1000);

  WiFi.onEvent(Wifi_connected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(Get_IP_Address, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.begin(ssid, password);

  Serial.print("Waiting for WIFI network ...");

  //UDP Settings
  Udp.begin(WiFi.localIP(), udpPort);
}

void SendUDP(int count) {
  Udp.beginPacket(udpAddress, udpPort);
  String data = MACHINE + " " + String(count)+".";

  const char* packet = data.c_str();
  Udp.printf(packet, sizeof(packet) / sizeof(*packet));
  Udp.endPacket();

  Serial.println("Sent UDP!");
}

void Task1code(void* pvParameters) {
  Serial.println("Task1");
  bool waitSent = false;
  uint8_t temp = 0;
  for (;;) {
    uint8_t eeprom_value = EEPROM.read(0);
    end_time = (millis() - start_time) / (60 * 1000.0);

    if (temp != eeprom_value) {
      waitSent = false;
      start_time = millis();
    }

    if (eeprom_value % CONSTANT != 0) {
      isSent = false;
    } else if (eeprom_value % CONSTANT == 0 && isSent == false) {
      SendUDP(eeprom_value);
      isSent = true;
    }

    if (end_time >= WAIT_TIME && waitSent == false) {
      SendUDP(eeprom_value);
      SendUDP(-1);
      waitSent = true;
    }

    temp = eeprom_value;
    delay(20);
  }
}

void Task2code(void* pvParameters) {
  Serial.println("Task2");

  for (;;) {

    int pinCode = digitalRead(SENSOR_PIN);
    if (pinCode == 0 && temp == 1) {
      counter += 1;
    }
    temp = pinCode;

    EEPROM.write(0, counter);
    EEPROM.commit();
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);

  //WIFI Setup
  initWiFi();

  //EEPROM Initialize
  EEPROM.begin(sizeof(MACHINE) * strlen(MACHINE.c_str()));

  //Pin Initialize
  pinMode(SENSOR_PIN, INPUT);

  //Task1 Core
  xTaskCreatePinnedToCore(
    Task1code,
    "Task1",
    10000,
    NULL,
    1,
    &Task1,
    0);
  delay(500);

  //Task2 Core
  xTaskCreatePinnedToCore(
    Task2code,
    "Task2",
    10000,
    NULL,
    1,
    &Task2,
    1);
  delay(500);
}

void loop() {}
