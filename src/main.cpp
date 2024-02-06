// WiFi enabled GPS NTP server - Cristiano Monteiro <cristianomonteiro@gmail.com> - 06.May.2021
// Based on the work of:
// Bruce E. Hall, W8BH <bhall66@gmail.com> - http://w8bh.net
// and
// https://forum.arduino.cc/u/ziggy2012/summary

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <Ticker.h>

// WiFi credentials
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// Network configuration
IPAddress staticIP(192, 168, 1, 101);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

#define NTP_PORT 123
static const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
WiFiUDP Udp;
Ticker wifiReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

RtcDS3231<TwoWire> Rtc(Wire);

ESP8266WebServer server(80);

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

void handleRoot() {
  server.send(200, "text/html", "<!DOCTYPE html><head><title>ESP8266 NTP Server</title></head><body><h3>You are connected</h3></body></html>");
}

void connectToWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.config(staticIP, dns, gateway, subnet);
  server.on("/", handleRoot);
  server.begin();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  ArduinoOTA.begin();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  wifiReconnectTimer.once(10, connectToWifi);
}

void setup() {
  Rtc.Begin();
  Serial.begin(9600);

  while (!Rtc.GetIsRunning()) {
    Rtc.SetIsRunning(true);
    Serial.println(F("RTC had to be force started"));
  }

  Serial.println(F("RTC started"));

#ifdef DEBUG
  PrintRTCstatus();
#endif

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  SyncWithRTC();
  Udp.begin(NTP_PORT);
  connectToWifi();

  ArduinoOTA.setHostname("ESP8266-NTP");
  ArduinoOTA.setPassword("password");
}

void loop() {
  ArduinoOTA.handle(); 
  server.handleClient();
  processNTP();
}

void processNTP() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();

    packetBuffer[0] = 0b00100100;
    packetBuffer[1] = 4;
    packetBuffer[2] = 6;
    packetBuffer[3] = 0xFA;
    packetBuffer[7] = 0;
    packetBuffer[8] = 0;
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;
    packetBuffer[11] = 0;
    packetBuffer[12] = 0;
    packetBuffer[13] = 0xC;
    packetBuffer[14] = 0;

    time_t t = now();
    unsigned long timestamp = numberOfSecondsSince1900Epoch(year(t), month(t), day(t), hour(t), minute(t), second(t));

    packetBuffer[12] = 71; // "G"
    packetBuffer[13] = 80; // "P"
    packetBuffer[14] = 83; // "S"
    packetBuffer[15] = 0;  // "0"

    packetBuffer[16] = (timestamp >> 24) & 0XFF;
    packetBuffer[17] = (timestamp >> 16) & 0xFF;
    packetBuffer[18] = (timestamp >> 8) & 0xFF;
    packetBuffer[19] = (timestamp) & 0xFF;
    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;

    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    packetBuffer[32] = (timestamp >> 24) & 0XFF;
    packetBuffer[33] = (timestamp >> 16) & 0xFF;
    packetBuffer[34] = (timestamp >> 8) & 0xFF;
    packetBuffer[35] = (timestamp) & 0xFF;
    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    packetBuffer[40] = (timestamp >> 24) & 0XFF;
    packetBuffer[41] = (timestamp >> 16) & 0xFF;
    packetBuffer[42] = (timestamp >> 8) & 0xFF;
    packetBuffer[43] = (timestamp) & 0xFF;
    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;

    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
}

void SyncWithRTC() {
  RtcDateTime time = Rtc.GetDateTime();
  long int a = time.Epoch32Time();
  setTime(a);
  Serial.print("SyncFromRTC: ");
  Serial.println(a);
  Serial.println("Synchronized from RTC");
}

unsigned long int numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mm, uint8_t s) {
  const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(daysInMonth + i - 1);
  if (m > 2 && y % 4 == 0)
    ++days;
  days += 365 * y + (y + 3) / 4 - 1;
  return days * 24UL * 3600UL + h * 3600UL + mm * 60UL + s + 2208988800UL;
}
