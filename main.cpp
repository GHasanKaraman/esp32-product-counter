#include <WiFi.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"

String MACHINE = "1";

const char* ssid = "cibovita";
const char* password = "cibovita2016";
const char* udpServerIP = "10.12.0.15";
const int udpServerPort = 5055;

const int laserPin = 35; // GPIO pin connected to the laser sensor 
//input 1 pun the most right

int productCount = 0;
int lastSentCount = 0;
int temp = 0;
int sentUDP = false;

WiFiUDP udp;
AsyncUDP asyncUdp;

unsigned long startTime = 0;
unsigned long lastSentTime = 0;
unsigned long now = 0;

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("CiboController WIFI connected to Access Point");
}

void Get_IP_Address(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("WIFI Connected!");
    startTime = millis();
    sendProductCount(productCount, now);
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    /*Serial.print("Subnet Mask: " );
    Serial.println(WiFi.subnetMask());
    Serial.print("Gateway IP: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("DNS 1: ");
    Serial.println(WiFi.dnsIP(0));
    Serial.print("DNS 2: ");
    Serial.println(WiFi.dnsIP(1));*/
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
 // Serial.println("Disconnected from WIFI");
 // Serial.print("WiFi lost connection. Reason: ");
 // Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Reconnecting ...");
  WiFi.begin(ssid, password);
}

void connectToWiFi() {
  WiFi.disconnect(true);
  delay(1000);

  WiFi.onEvent(Wifi_connected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(Get_IP_Address, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.begin(ssid, password);
}

void sendProductCount(int count, unsigned long time) {
  if(WiFi.status() == WL_CONNECTED){
    if (udp.beginPacket(udpServerIP, udpServerPort)) {
      String data = MACHINE + " " + String(count)+".";
      const char* packet = data.c_str();
      udp.printf(packet, sizeof(packet) / sizeof(*packet));
      udp.endPacket();
      Serial.println("Sent product count to server");
      lastSentCount = productCount;
      lastSentTime = time;
    } else {
      Serial.println("Failed to send product count");
    }
  }
  else{
    Serial.println("No Connection No UDP!");
  }
}

//setup
void setup() {
  Serial.begin(115200);
  pinMode(laserPin, INPUT);
  connectToWiFi();
}

//loop
void loop() {
  int pinCode = digitalRead(laserPin);
    // Laser beam detected, increment product count
    if (pinCode == 0 && temp == 1) {
      productCount += 1;
      Serial.println("Product Count: " + String(productCount));
    }
 
    now = millis();

    if(lastSentCount != productCount && now - lastSentTime >= 5000){
      sendProductCount(productCount, now);
    }

    if(now - lastSentTime >= 1000*60*5){
      sendProductCount(-1, now);
    }

    startTime = now;

    if (asyncUdp.listen(1234)) {
      asyncUdp.onPacket([](AsyncUDPPacket packet) {
      String myString = (const char*)packet.data();
      if(myString == "rst"){
          productCount = 0;
          packet.printf("done");
      }
    });
  }
    temp = pinCode;
    delay(50);
}
