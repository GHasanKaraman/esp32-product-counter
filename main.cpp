#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include "time.h"

String MACHINE = "J1";

WiFiUDP Udp;

const char* ssid = "cibovita";
const char* password = "cibovita2016";

const char * udpAddress = "10.12.11.193";
unsigned int udpPort = 80;

#define SENSOR_PIN 23
#define BUTTON_PIN 22

float WAIT_TIME = 10; //minutes
float STOP_TIME = 60; //minutes
const int CONSTANT = 5;

TaskHandle_t Task1;
TaskHandle_t Task2;

int counter = 0;
int temp = 0;

bool isSent = false;

float start_time = 0;
float end_time = 0;

//Time variables
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;
const int daylightOffset_sec = 0;

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("ESP32 WIFI connected to Access Point");
}

void Get_IP_Address(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WIFI Connected!");
  Serial.print("IP Address of the device: ");
  Serial.println(WiFi.localIP());
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WIFI");
  Serial.print("Connection lost! The reason is: ");

  Serial.print("Reconnecting .");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
}

void checkTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  if(timeinfo.tm_hour == 24){
    counter = 0;
  }
}

void initWiFi() {
  WiFi.disconnect(true);
  delay(1000);

  WiFi.onEvent(Wifi_connected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(Get_IP_Address, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, SYSTEM_EVENT_STA_DISCONNECTED);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Waiting for WIFI network .");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  //UDP Settings
  Udp.begin(WiFi.localIP(), udpPort);
}

void SendUDP(int count){
    Udp.beginPacket(udpAddress, udpPort);
    String data = MACHINE + " " + String(count);

    const char* packet = data.c_str();
    Udp.printf(packet, sizeof(packet)/sizeof(*packet));
    Udp.endPacket();

    Serial.println("Sent UDP!");
}

void Task1code( void * pvParameters ){
  Serial.println("Task1");
  bool waitSent = false;
  uint8_t temp = 0;
  for(;;){
    uint8_t eeprom_value = EEPROM.read(0);  
    end_time = (millis() - start_time) / (60*1000.0); 

    if(temp!=eeprom_value){
      waitSent = false;
      start_time = millis();
    }

    if(eeprom_value % CONSTANT != 0){
      isSent = false;
    }
    else if(eeprom_value % CONSTANT == 0 && isSent == false){
      SendUDP(eeprom_value);
      isSent = true;
    }

    if(end_time >= WAIT_TIME && waitSent == false){
      SendUDP(eeprom_value);
      SendUDP(-1);
      waitSent = true;
    }

    if (end_time >= STOP_TIME){
      checkTime();
      delay(4980);
    }
    temp = eeprom_value;
    delay(20);
  } 
}

void Task2code( void * pvParameters ){
  Serial.println("Task2");

  for(;;){

    if(digitalRead(BUTTON_PIN) == 1){
      counter = 0;
      EEPROM.write(0, 0);
      EEPROM.commit();
      Serial.println("Reset EEPROM and the counter!");
    }

    int pinCode = digitalRead(SENSOR_PIN);
    if(pinCode == 0 && temp == 1){ 
      counter+=1;
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
  EEPROM.begin(sizeof(MACHINE)*strlen(MACHINE.c_str()));

  //Pin Initialize
  pinMode(SENSOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);

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

void loop(){}
