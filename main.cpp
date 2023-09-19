#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"

String MACHINE = "2";

WiFiUDP Udp;
AsyncUDP asyncUdp;

const char* ssid = "";
const char* password = "";

const char* udpAddress = "10.12.0.15";
unsigned int udpPort = 5055;

#define SENSOR_PIN 23

float WAIT_TIME = 2;    //minutes
float RESET_TIME = 60;  //minutes

int CONSTANT = 10;

TaskHandle_t Task1;
TaskHandle_t Task2;

int counter = 0;
int temp = 0;

bool isSent = false;

float start_time = 0;
float end_time = 0;

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info) {
  //Serial.println("ESP32 WIFI connected to Access Point");
}

void Get_IP_Address(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("WIFI Connected!");
/*
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Subnet Mask: " );
    Serial.println(WiFi.subnetMask());
    Serial.print("Gateway IP: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("DNS 1: ");
    Serial.println(WiFi.dnsIP(0));
    Serial.print("DNS 2: ");
    Serial.println(WiFi.dnsIP(1));
 */
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
 // Serial.println("Disconnected from WIFI");
 // Serial.print("WiFi lost connection. Reason: ");
 // Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Reconnecting ...");
  WiFi.begin(ssid, password);
}

void initWiFi() {
  WiFi.disconnect(true);
  delay(1000);

  WiFi.onEvent(Wifi_connected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(Get_IP_Address, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);

  Serial.println("Waiting for WIFI network ...");

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

bool is_number(char* str)
{
    if(str == NULL)
      return 1;

    bool cond = true;

    for (int i = 0; i < strlen(str); i++) {
      if(isDigit(str[i])){
        cond = false;
        break;
      }
    }

    return cond;
}

void Task1code(void* pvParameters) {
  Serial.println("Task1");
  int temp = 0;
  bool waitSent = false;
  while(1) {
    end_time = (millis() - start_time) / (60 * 1000.0);

    if(temp != counter){
      waitSent = false;
      start_time = millis();
    }

    if(counter % CONSTANT != 0){
      isSent = false;
    }
    else if(counter % CONSTANT == 0 && isSent == false){
      SendUDP(counter);
      isSent = true;
    }

    if(end_time >= RESET_TIME){
      counter = 0;
      SendUDP(-2);
      start_time = millis();
    }
    else if (end_time >= WAIT_TIME && waitSent == false) {
      SendUDP(counter);
      SendUDP(-1);
      waitSent = true;
    }

    temp = counter;
    
    delay(100);
  }
}

void Task2code(void* pvParameters) {
  Serial.println("Task2");

  while(1) {
    int pinCode = digitalRead(SENSOR_PIN);
    if (pinCode == 0 && temp == 1) {
      counter += 1;
    }
    temp = pinCode;

    delay(10);

    if (asyncUdp.listen(1234)) {
      asyncUdp.onPacket([](AsyncUDPPacket packet) {
      String myString = (const char*)packet.data();
      char* str = strdup(myString.c_str());
      char* token = strtok(str, "-");
      char* num1 = token;
      token = strtok(NULL, "-");
      char* num2 = token;

      if(is_number(num1) == 0 && is_number(num2) == 0){
        WAIT_TIME = atoi(num1);
        RESET_TIME = atoi(num2);
        packet.printf("Changed values!");
      }
    });
  }
  }
}

void setup() {
  Serial.begin(115200);

  //WIFI Setup
  initWiFi();

  SendUDP(0);

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
