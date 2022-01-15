// WiFi enabled GPS NTP server - Cristiano Monteiro <cristianomonteiro@gmail.com> - 06.May.2021
// Based on the work of:
// Bruce E. Hall, W8BH <bhall66@gmail.com> - http://w8bh.net
// and
// https://forum.arduino.cc/u/ziggy2012/summary

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h> // Needed for UDP functionality
#include <TimeLib.h>   // Time functions
#include <Wire.h>      // OLED and DS3231 necessary
#include <RtcDS3231.h> // RTC functions
#include <Ticker.h> // for wifi timeout

// GLOBAL DEFINES
#define WIFI_SSID "ssid"
#define WIFI_PASSWORD "password"
#define NTP_PORT 123 // Time Server Por
IPAddress staticIP(192, 168, 1, 101); //ESP static ip
IPAddress gateway(192, 168, 1, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns(192, 168, 1, 1);  //DNS

static const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE]; // buffers for receiving and sending data
WiFiUDP Udp; // An Ethernet UDP instance
Ticker wifiReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

RtcDS3231<TwoWire> Rtc(Wire);

ESP8266WebServer server(80);

//#define DEBUG // Comment this in order to remove debug code from release version

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

// WiFi Routines
void handleRoot()
{
  server.send(200, "text/html", "<!DOCTYPE html><head><title>ESP8266 NTP Server</title></head><body><h3>You are connected</h3></body></html>");
}

void connectToWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.config(staticIP, dns, gateway, subnet);//WiFi.config(staticIP, subnet, gateway, dns);
  server.on("/", handleRoot);
  server.begin();
}


// --------------------------------------------------------------------------------------------------
// SERIAL MONITOR ROUTINES
// These routines print the date/time information to the serial monitor
// Serial monitor must be initialized in setup() before calling

void PrintDigit(int d)
{
  if (d < 10)
    DEBUG_PRINT('0');
  DEBUG_PRINT(d);
}

void PrintTime(time_t t)
// display time and date to serial monitor
{
  PrintDigit(month(t));
  DEBUG_PRINT("-");
  PrintDigit(day(t));
  DEBUG_PRINT("-");
  PrintDigit(year(t));
  DEBUG_PRINT(" ");
  PrintDigit(hour(t));
  DEBUG_PRINT(":");
  PrintDigit(minute(t));
  DEBUG_PRINT(":");
  PrintDigit(second(t));
  DEBUG_PRINTLN(" UTC");
}

// --------------------------------------------------------------------------------------------------
//  RTC SUPPORT
//  These routines add the ability to get and/or set the time from an attached real-time-clock module
//  such as the DS1307 or the DS3231.  The module should be connected to the I2C pins (SDA/SCL).

// send current RTC information to serial monitor
void PrintRTCstatus()
{
  RtcDateTime Now = Rtc.GetDateTime();
  time_t t = Now.Epoch32Time();
  if (t)
  {
    DEBUG_PRINT("PrintRTCstatus: ");
    DEBUG_PRINTLN("Called PrintTime from PrintRTCstatus");
#ifdef DEBUG
    PrintTime(t);
#endif
  }
  else
    DEBUG_PRINTLN("ERROR: cannot read the RTC.");
}

// Update RTC from current system time
void SetRTC(time_t t)
{
  RtcDateTime timeToSet;
  timeToSet.InitWithEpoch32Time(t);
  Rtc.SetDateTime(timeToSet);
  if (Rtc.LastError() == 0)
  {
    DEBUG_PRINT("SetRTC: ");
    DEBUG_PRINTLN("Called PrintTime from SetRTC");
#ifdef DEBUG
    PrintTime(t);
#endif
  }
  else
    DEBUG_PRINT("ERROR: cannot set RTC time");
}

void ManuallySetRTC()
// Use this routine to manually set the RTC to a specific UTC time.
// This routine is mainly for debugging purposes. Change numeric constants to the time desired.
{
  tmElements_t tm;
  tm.Year   = 2022 - 1970;                              // Year in unix years
  tm.Month  = 1;
  tm.Day    = 15;
  tm.Hour   = 14;
  tm.Minute = 36;
  tm.Second = 30;
  SetRTC(makeTime(tm));                                 // set RTC to desired time (UTC)
}

// --------------------------------------------------------------------------------------------------
// TIME SYNCHONIZATION ROUTINES
// These routines will synchonize time with GPS and/or RTC as necessary
// Sync with GPS occur when the 1pps interrupt signal from the GPS goes high.
// GPS synchonization events are attempted every (SYNC_INTERVAL) seconds.
// If a valid GPS signal is not received within (SYNC_TIMEOUT) seconds, the clock with synchonized
// with RTC instead.  The RTC time is updated with GPS data once every 24 hours.

void SyncWithRTC()
{
  RtcDateTime time = Rtc.GetDateTime();
  long int a = time.Epoch32Time();
  setTime(a); // set system time from RTC
  DEBUG_PRINT("SyncFromRTC: ");
  DEBUG_PRINTLN(a);
  DEBUG_PRINTLN("Synchronized from RTC"); // send message to serial monitor
}

// --------------------------------------------------------------------------------------------------
// Wifi timeout
void onWifiConnect(const WiFiEventStationModeGotIP& event) {
#ifdef DEBUG
  Serial.println("Connected to Wi-Fi.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#endif
  ArduinoOTA.begin();
}
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  wifiReconnectTimer.once(10, connectToWifi); //set re-connect time
}
// --------------------------------------------------------------------------------------------------
// MAIN PROGRAM
void setup()
{
  Rtc.Begin();

#ifdef DEBUG
  Serial.begin(9600); // set serial monitor rate to 9600 bps
#endif

  DEBUG_PRINTLN("Boot");

  // Initialize RTC
  while (!Rtc.GetIsRunning())
  {
    Rtc.SetIsRunning(true);
    DEBUG_PRINTLN(F("RTC had to be force started"));
  }

  DEBUG_PRINTLN(F("RTC started"));

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state


#ifdef DEBUG
  PrintRTCstatus(); // show RTC diagnostics
#endif
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  //ManuallySetRTC();                         // 1 time only
  SyncWithRTC();                         // start clock with RTC data
  Udp.begin(NTP_PORT); // Startup UDP
  connectToWifi();

  //  OTA Setup
  ArduinoOTA.setHostname("ESP8266-NTP");
  ArduinoOTA.setPassword("password");
}
////////////////////////////////////////
const uint8_t daysInMonth[] PROGMEM = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //const or compiler complains

const unsigned long seventyYears = 2208988800UL; // to convert unix time to epoch

// NTP since 1900/01/01
static unsigned long int numberOfSecondsSince1900Epoch(uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mm, uint8_t s)
{
  if (y >= 1970)
    y -= 1970;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(daysInMonth + i - 1);
  if (m > 2 && y % 4 == 0)
    ++days;
  days += 365 * y + (y + 3) / 4 - 1;
  return days * 24L * 3600L + h * 3600L + mm * 60L + s + seventyYears;
}

////////////////////////////////////////
void processNTP()
{
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort();

    packetBuffer[0] = 0b00100100; // LI, Version, Mode
    packetBuffer[1] = 4;    // stratum
    packetBuffer[2] = 6;    // polling minimum
    packetBuffer[3] = 0xFA; // precision

    packetBuffer[7] = 0; // root delay
    packetBuffer[8] = 0;
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;

    packetBuffer[11] = 0; // root dispersion
    packetBuffer[12] = 0;
    packetBuffer[13] = 0xC;
    packetBuffer[14] = 0;

    unsigned long date, time, age;
    uint32_t timestamp, tempval;
    time_t t = now();

    timestamp = numberOfSecondsSince1900Epoch(year(t), month(t), day(t), hour(t), minute(t), second(t));

#ifdef DEBUG
    Serial.println(timestamp);
#endif

    tempval = timestamp;
    packetBuffer[12] = 71; //"G";
    packetBuffer[13] = 80; //"P";
    packetBuffer[14] = 83; //"S";
    packetBuffer[15] = 0;  //"0";
    
    // reference timestamp
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval)&0xFF;
    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;
    
    //copy originate timestamp from incoming UDP transmit timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];
    
    //receive timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval)&0xFF;
    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;
    //transmitt timestamp
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval) & 0xFF;
    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;
    
    // Reply to the IP address and port that sent the NTP request
    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
  }
}
////////////////////////////////////////
void loop()
{
  ArduinoOTA.handle(); 
  server.handleClient();
  processNTP();
}
