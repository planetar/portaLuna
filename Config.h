#pragma once

#include <ArduinoJson.h>
#include <FS.h>

namespace Config {
    char mqtt_server[80] = "example.tld";
    char mqtt_username[24] = "";
    char mqtt_password[24] = "";
     int mqtt_port         = 1883; 
     
    char OTA_password[24]  = "whatever";
     int OTA_port          = 8266;

     
    byte hue = 34 ;
    byte sat = 221;
    byte val = 200;
    byte bright =64 ;
    byte probab  = 15    ;
    int  stepTime  = 48  ;
    byte effectOption = 0;
    bool stateOn = true;

    void save() {
        DynamicJsonDocument json(512);
        json["mqtt_server"]   = mqtt_server;
        json["mqtt_username"] = mqtt_username;
        json["mqtt_password"] = mqtt_password;
        json["mqtt_port"]     = mqtt_port;
        
        json["OTA_password"]  = OTA_password;
        json["OTA_port"]      = OTA_port;

        
        json["hue"]      = hue;
        json["sat"]      = sat;
        json["val"]      = val;
        json["bright"]      = bright;
        json["probab"]      = probab;
        json["stepTime"]      = stepTime;
        json["effectOption"]      = effectOption;
        
        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
            return;
        }

        serializeJson(json, configFile);
        configFile.close();
    }

    void load() {
        if (SPIFFS.begin()) {

            if (SPIFFS.exists("/config.json")) {
                File configFile = SPIFFS.open("/config.json", "r");

                if (configFile) {
                    const size_t size = configFile.size();
                    std::unique_ptr<char[]> buf(new char[size]);

                    configFile.readBytes(buf.get(), size);
                    DynamicJsonDocument json(512);

                    if (DeserializationError::Ok == deserializeJson(json, buf.get())) {
                        strcpy(mqtt_server, json["mqtt_server"]);
                        strcpy(mqtt_username, json["mqtt_username"]);
                        strcpy(mqtt_password, json["mqtt_password"]);
                        
                        mqtt_port = json["mqtt_port"]; // ??
                        
                        strcpy(OTA_password, json["OTA_password"]);
                        OTA_port = json["OTA_port"];

                        hue = json["hue"];
                        sat = json["sat"];
                        val = json["val"];
                        bright = json["bright"];
                        probab = json["probab"];
                        stepTime = json["stepTime"];
                        effectOption = json["effectOption"];
                    }
                }
            }
        }
    }
} // namespace Config
