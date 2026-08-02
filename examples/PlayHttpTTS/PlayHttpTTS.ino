/*
 * PlayHttpTTS - fetch audio over HTTP (e.g. from a TTS API) and play it
 * through MAX98357 while it downloads.
 *
 * The sketch sends a request, skips the HTTP response headers, then
 * hands the WiFiClient (positioned at the body) to the library:
 *   - amp.playWavStream(client)  for WAV responses
 *   - amp.playMp3Stream(client)  for MP3 responses
 *
 * Notes:
 *   - Works with Content-Length or connection-close responses.
 *     Chunked transfer encoding is NOT parsed - ask your TTS server
 *     for a plain response if possible.
 *   - WAV must be 16-bit PCM. Typical TTS rates (16k/22.05k/24k) are
 *     all supported.
 */

#include "WiFi.h"
#include <MAX98357.h>

char ssid[] = "your_ssid";
char pass[] = "your_password";

// Example: a server that returns a WAV file / TTS speech as WAV
char server[] = "192.168.50.91";
int port = 80;
String path = "/tts?text=hello";

WiFiClient client;
MAX98357 amp;

// Read the HTTP response headers; returns true when the blank line
// (end of headers) was found.
bool skipHttpHeaders(WiFiClient &c, uint32_t timeoutMs = 10000)
{
    String line = "";
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        while (c.available()) {
            char ch = c.read();
            if (ch == '\n') {
                if (line.length() == 0) {
                    return true;  // blank line = end of headers
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

    amp.begin();
    amp.setVolume(0.8);

    Serial.println("Requesting audio...");
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

    // Body starts here - stream it straight into the amp.
    if (amp.playWavStream(client)) {      // use playMp3Stream() for MP3
        Serial.println("Done.");
    } else {
        Serial.print("Playback failed: ");
        Serial.println(amp.lastError());
    }
    client.stop();
}

void loop()
{
}
