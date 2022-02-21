# portaLuna
Arduino Sketch to drive a moon lamp with WS2812b leds and an ESP8266 mcu. It features a captive portal with wifimanager, settings for wifi, mqtt and OTA access, a web server with a page to configure and control the lights (embAjax). The thing announces the web service by bonjour/avahi (mDNS). The sketch provides 7 light modes which focus on allowing a flowing/glowing illumination. The led control is done by FastLed.

The sketch's primary mode of operation was to report to and accept commands from a mqtt broker. That way it integrates with a home automation like openHAB. The webConfig however allows to run and control the lamp without the need to have a mqtt broker and other clients in the network. The commands of the 2 interfaces overlap for the most parts but are not identical.

![WebConfig](https://user-images.githubusercontent.com/29571846/154934706-6903a710-992b-4f15-abe8-27d40e4d6ae4.png)
