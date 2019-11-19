#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include "PubSubClient.h"

#ifndef OTA_H
#define OTA_H


class OTA
{
    public:
        OTA();
        bool loadConfig(String *ssid, String *pass);
        bool saveConfig(String *ssid, String *pass);
        void setup();
        void loop();
        virtual ~OTA();
        
        // Default WiFi connection information.
        char* ap_default_ssid = "NETGEAR98"; ///< Default SSID.
        char* ap_default_psk = "greenvase352"; ///< Default PSK.
    protected:
    private:
};

#endif // OTA_H
