/*
 * MAX98357.h - I2S audio playback library for HUB8735 Ultra (RTL8735B)
 *
 * Drives a MAX98357 I2S amplifier through the RTL8735B I2S0 peripheral
 * using the low-level mbed i2s_api (compiled in liboutsrc.a of the
 * AmebaPro2 Arduino package). MP3 decoding uses the Helix decoder
 * (libhmp3.a) that also ships with the package.
 *
 * RTL8735B I2S pins are a FIXED pin group (they cannot be remapped):
 *
 *   I2S signal | SoC pin | HUB8735 Ultra pin | MAX98357 pin
 *   -----------+---------+-------------------+-------------
 *   SCK (BCLK) | PF_13   | D24               | BCLK
 *   WS  (LRCK) | PF_15   | D12               | LRC
 *   SD_TX      | PF_14   | D11               | DIN
 *   SD_RX      | PF_12   | D23               | (unused, still claimed)
 *   MCLK       | PF_11   | D22               | (unused, still claimed)
 *
 * NOTE: begin() claims ALL five pins above. D12 is also the on-board
 * push button (PUSH_BTN), so the button cannot be used together with
 * this library - move your button to another GPIO.
 *
 * Supported sources:
 *   - WAV  (RIFF PCM 16-bit, mono/stereo) from SD card or any Stream
 *     (e.g. a WiFiClient positioned at an HTTP response body)
 *   - MP3  from SD card or any Stream (Helix decoder)
 *   - Raw PCM pushed by the caller (beginPCM / writePCM / endPCM),
 *     e.g. audio returned by a TTS API
 */

#ifndef MAX98357_H
#define MAX98357_H

#include <Arduino.h>
#include "AmebaFatFS.h"

class MAX98357 {
public:
    MAX98357();

    // Initialize I2S0 (master, TX). Claims D11/D12/D22/D23/D24.
    bool begin(void);
    // Release the I2S peripheral and its pins.
    void end(void);

    // Software volume, 0.0 ~ 1.0 (default 1.0). Applied per sample.
    void setVolume(float vol);

    // Optional: wire the MAX98357 SD_MODE pin to a GPIO and register it
    // here. The amp is then held in hardware shutdown except while
    // audio is playing (eliminates idle heating on modules that fail to
    // auto-shutdown on clock loss). Note: SD_MODE driven high by a
    // 3.3V GPIO selects LEFT channel output.
    void setShutdownPin(int pin);

    // ---- WAV ----
    // Play a WAV file from the SD card root, blocking until finished.
    // fs.begin() must have been called already.
    bool playWav(AmebaFatFS &fs, const char *filename);
    // Play from an already-opened file (header parsed from offset 0).
    bool playWav(File &f);
    // Play WAV from a forward-only stream, e.g. a WiFiClient already
    // positioned at the start of an HTTP response body (after headers).
    // Handles unknown data size (streams until the connection ends).
    bool playWavStream(Stream &s);

    // ---- MP3 (Helix decoder) ----
    bool playMp3(AmebaFatFS &fs, const char *filename);
    bool playMp3(File &f);
    bool playMp3Stream(Stream &s);

    // ---- Raw PCM push API (for TTS responses, synthesis, etc.) ----
    // Start the I2S output at the given rate/channels, then push
    // interleaved 16-bit samples with writePCM (blocks while the
    // internal buffer is full), and finish with endPCM.
    bool beginPCM(uint32_t sampleRate, uint16_t channels);
    // sampleCount = number of int16 values in data (all channels).
    // Returns false on internal stall (I2S dead for >2s).
    bool writePCM(const int16_t *data, size_t sampleCount);
    void endPCM(void);

    // Human-readable reason of the last failure.
    const char *lastError(void) const;

    // Byte-source callback used internally by the format decoders.
    typedef size_t (*ReadFn)(void *ctx, uint8_t *buf, size_t len);

private:
    bool startOutput(int srEnum);
    void stopOutput(uint32_t sampleRate);
    bool pushBlock(const int16_t *samples, size_t count, bool stereo);
    bool flushSlot(void);
    bool waitSlotFree(void);
    bool playWavCommon(ReadFn rd, void *ctx, bool canSeek, File *f);
    bool playMp3Common(ReadFn rd, void *ctx);
    int mapSampleRate(uint32_t hz);

    uint16_t _vol_q8;        // volume in Q8 fixed point, 256 = 1.0
    bool _inited;
    bool _outputActive;
    uint16_t _streamChans;   // channels set by beginPCM
    uint32_t _streamRate;    // sample rate set by beginPCM
    int _sdPin;              // GPIO driving SD_MODE, -1 = not used
    const char *_err;
};

#endif  // MAX98357_H
