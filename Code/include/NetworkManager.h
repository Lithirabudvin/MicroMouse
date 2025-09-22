#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <WiFi.h>
#include "config.h"

class NetworkManager {
private:
    WiFiServer server;
    WiFiClient client;

public:
    NetworkManager() : server(TCP_PORT) {}
    
    bool setup() {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print("Connecting to WiFi");
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("\nConnected! IP address: ");
        Serial.println(WiFi.localIP());
        
        // Start TCP server
        server.begin();
        Serial.println("TCP server started on port " + String(TCP_PORT));
        return true;
    }
    
    void handleClient() {
        if (!client || !client.connected()) {
            client = server.available();
            if (client) {
                #if DEBUG_NETWORK
                Serial.println("New client connected");
                #endif
                client.println("Robot Controller Connected");
            }
        }
    }
    
    void sendMessage(String message) {
        Serial.print(message);
        if (client && client.connected()) {
            client.print(message);
        }
    }
};

#endif