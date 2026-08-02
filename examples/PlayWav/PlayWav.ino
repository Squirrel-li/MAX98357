/*
 * PlayWav - minimal MAX98357 playback test for HUB8735 Ultra.
 *
 * Wiring (I2S0 fixed pin group):
 *   MAX98357 BCLK -> D24 (PF_13)
 *   MAX98357 LRC  -> D12 (PF_15)   * D12 is also the on-board button!
 *   MAX98357 DIN  -> D11 (PF_14)
 *   MAX98357 VIN  -> 5V, GND -> GND
 *   GAIN floating = 9dB, SD_MODE floating = enabled
 *
 * Put a 16-bit PCM WAV named test001.wav in the SD card root.
 */

#include "AmebaFatFS.h"
#include <MAX98357.h>

AmebaFatFS fs;
MAX98357 amp;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    fs.begin();
    amp.begin();
    amp.setVolume(0.8);

    Serial.println("Playing test001.wav ...");
    if (amp.playWav(fs, "test001.wav")) {
        Serial.println("Done.");
    } else {
        Serial.print("Playback failed: ");
        Serial.println(amp.lastError());
    }
}

void loop()
{
}
