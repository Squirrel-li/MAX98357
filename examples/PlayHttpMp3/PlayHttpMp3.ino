/* Stream an MP3 file from a plain HTTP server to MAX98357. */

#include "WiFi.h"
#include <MAX98357.h>

char ssid[] = "TP-Link_0355_5G";
char pass[] = "29271104";

char server[] = "192.168.1.106";
int port = 8000;
String path = "/sample-15s.mp3";

WiFiClient client;
MAX98357 amp;

bool skipHttpHeaders(WiFiClient &c, uint32_t timeoutMs = 10000)
{
    String line;
    uint32_t t0 = millis();

    while (millis() - t0 < timeoutMs) {
        while (c.available()) {
            char ch = c.read();
            if (ch == '\n') {
                if (line.length() == 0) {
                    return true;
                }
                line = "";
            } else if (ch != '\r') {
                line += ch;
            }
        }
        if (!c.connected() && !c.available()) {
            break;
        }
        delay(1);
    }
    return false;
}

void setup()
{
    Serial.begin(115200);

    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
        Serial.println("connecting to WiFi...");
        delay(2000);
    }
    Serial.println("WiFi connected");

    if (!amp.begin()) {
        Serial.println(amp.lastError());
        return;
    }
    amp.setVolume(0.5f);

    if (!client.connect(server, port)) {
        Serial.println("connection failed");
        return;
    }

    client.println("GET " + path + " HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Connection: close");
    client.println();

    if (!skipHttpHeaders(client)) {
        Serial.println("no HTTP response");
        client.stop();
        return;
    }

    if (!amp.playMp3Stream(client)) {
        Serial.print("Playback failed: ");
        Serial.println(amp.lastError());
    } else {
        Serial.println("Done.");
    }
    client.stop();
}

void loop() {}
