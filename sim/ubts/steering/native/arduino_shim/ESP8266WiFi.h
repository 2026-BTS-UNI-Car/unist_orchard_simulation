// ESP8266WiFi subset for the native SIM build: the firmware only switches the radio off.
#ifndef UBTS_SIM_ESP8266WIFI_H
#define UBTS_SIM_ESP8266WIFI_H

enum WiFiMode_t { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3 };

class ESP8266WiFiClass {
 public:
  bool mode(WiFiMode_t m);
};

extern ESP8266WiFiClass WiFi;

#endif  // UBTS_SIM_ESP8266WIFI_H
