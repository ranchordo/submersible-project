#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

IPAddress staticIP(192, 168, 1, 7);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(192, 168, 1, 1);
IPAddress dns2(1, 1, 1, 1);

WiFiServer socket(2244);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(19200);
    // WiFi.mode(WIFI_STA);
    // esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    // WiFi.config(staticIP, gateway, subnet, dns1, dns2);
    // WiFi.begin("Anchornet_2.4", "mineralbasqu3");
    // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //     digitalWrite(LED_BUILTIN, HIGH);
    //     delay(100);
    //     digitalWrite(LED_BUILTIN, LOW);
    //     ESP.restart();
    // }
    // socket.begin();
}

void loop() {
    Serial.println("Here's some data");
    delay(100);
}