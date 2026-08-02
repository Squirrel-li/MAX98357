# MAX98357 for HUB8735 Ultra (RTL8735B)

**English** | [繁體中文](README.zh-TW.md)

An Arduino library that plays **WAV and MP3** audio — from the SD card or **streamed over the network** (e.g. a TTS API response) — through a **MAX98357 I2S class-D amplifier** on the **HUB8735 Ultra** (or any RTL8735B / AmebaPro2 board).

## Why this library exists

The Arduino SDK for HUB8735 Ultra ships **no I2S playback library**. This library talks directly to the low-level mbed `i2s_api` (already compiled into the official AmebaPro2 Arduino package as `liboutsrc.a` — nothing extra to install) and follows the initialization order and interrupt model of Realtek's production `module_i2s.c`:

- The RTL8735B low layer has **no true TX-only mode** — the library runs in TXRX mode, arms all RX pages and discards the data
- `i2s_enable()` must be called **only after all DMA pages are pre-filled and submitted**, otherwise the hardware throws a `page own control error` storm
- The TX-done ISR **always refills a page** (silence when no data is ready) so the DMA never starves; all SD card reads happen in the caller's context, never in the ISR

## Wiring

The RTL8735B I2S pins are a **fixed pin group and cannot be remapped**:

| MAX98357 | HUB8735 Ultra | SoC pin | Notes |
|---|---|---|---|
| BCLK | **D24** | PF_13 | I2S bit clock |
| LRC | **D12** | PF_15 | Word select (L/R clock) |
| DIN | **D11** | PF_14 | Audio data |
| VIN | 5V | — | Add a 470µF electrolytic capacitor across VIN–GND (see below) |
| GND | GND | — | Common ground |
| GAIN | floating = 9dB | — | Tie to VIN for 6dB (runs cooler) |
| SD_MODE | floating, or any GPIO | — | Wire to a GPIO to use `setShutdownPin()` (see below) |

**Caveats:**

- `begin()` claims the **entire 5-pin group: D11, D12, D22, D23, D24** (including the unused SD_RX/MCLK — a hardware pin-group limitation)
- **D12 is also the on-board push button (PUSH_BTN)** — the button cannot be used together with this library; wire an external button to another GPIO
- **D13 cannot be used as a button pin**: it drives the camera flash LED (PWM) and is pulled low on the board, so it always reads LOW

## Installation

Drop the `MAX98357/` folder into your Arduino `libraries/` directory (either the sketchbook `libraries/` or the board package's `libraries/`), restart the IDE, then `#include <MAX98357.h>`.

## Usage

```cpp
#include "AmebaFatFS.h"
#include <MAX98357.h>

AmebaFatFS fs;
MAX98357 amp;

void setup() {
    fs.begin();                  // SD card
    amp.begin();                 // I2S init (claims D11/D12/D22/D23/D24)
    amp.setShutdownPin(10);      // optional: SD_MODE wired to D10, hardware shutdown when idle
    amp.setVolume(0.8);          // software volume 0.0 ~ 1.0

    if (!amp.playWav(fs, "test001.wav")) {   // blocking playback from SD root
        Serial.println(amp.lastError());
    }
}

void loop() {}
```

See [examples/PlayWav](examples/PlayWav/PlayWav.ino) for the full example.

### Playing MP3

MP3 decoding uses the Helix decoder already bundled with the AmebaPro2 package (`libhmp3.a`) — no extra install:

```cpp
amp.playMp3(fs, "test001.mp3");
```

### Streaming from the network (TTS)

Hand any Arduino `Stream` positioned at the audio data to the library — e.g. a `WiFiClient` right after the HTTP response headers. Audio plays **while it downloads**:

```cpp
WiFiClient client;
client.connect(server, 80);
client.println("GET /tts?text=hello HTTP/1.1");
client.println("Host: " + String(server));
client.println("Connection: close");
client.println();
skipHttpHeaders(client);              // read until the blank line

amp.playWavStream(client);            // WAV response
// amp.playMp3Stream(client);         // or MP3 response
```

Full example (including the header-skipping helper): [examples/PlayHttpTTS](examples/PlayHttpTTS/PlayHttpTTS.ino). WAV responses with unknown data size (streaming encoders write 0 or 0xFFFFFFFF) play until the connection ends. Chunked transfer encoding is not parsed — request a plain response.

If your TTS returns **raw PCM**, use the push API directly:

```cpp
amp.beginPCM(24000, 1);               // sample rate, channels
amp.writePCM(samples, sampleCount);   // call repeatedly; blocks when buffer is full
amp.endPCM();
```

### API

| Function | Description |
|---|---|
| `bool begin()` | Initialize I2S0 (master/TX). Claims D11/D12/D22/D23/D24 |
| `void end()` | Release the I2S peripheral and its pins |
| `void setVolume(float vol)` | Software volume 0.0–1.0 (per-sample scaling), default 1.0 |
| `void setShutdownPin(int pin)` | Optional: GPIO wired to SD_MODE, driven LOW between playbacks for hardware shutdown (fixes idle heating on some modules). Note: 3.3V on SD_MODE selects left-channel-only mode |
| `bool playWav(AmebaFatFS &fs, const char *filename)` | Play a WAV from the SD card root (blocks until finished) |
| `bool playWav(File &f)` | Play from an already-opened file |
| `bool playWavStream(Stream &s)` | Play WAV from a forward-only stream (HTTP body, etc.) |
| `bool playMp3(AmebaFatFS &fs, const char *filename)` | Play an MP3 from the SD card root |
| `bool playMp3(File &f)` | Play MP3 from an already-opened file |
| `bool playMp3Stream(Stream &s)` | Play MP3 from a stream |
| `bool beginPCM(uint32_t rate, uint16_t ch)` | Start raw PCM output (16-bit interleaved) |
| `bool writePCM(const int16_t *data, size_t n)` | Push n int16 samples; blocks while the buffer is full |
| `void endPCM()` | Flush, drain and stop the PCM output |
| `const char *lastError()` | Reason for the most recent failure |

### Supported audio formats

- **WAV**: RIFF PCM **16-bit**, mono or stereo (mono is duplicated to both channels)
- **MP3**: anything the Helix decoder handles (MPEG-1/2 Layer III, mono/stereo)
- Sample rates: 8k / 11.025k / 12k / 16k / 22.05k / 24k / 32k / 44.1k / 48k / 88.2k / 96k Hz

To convert other formats:

```bash
ffmpeg -i input.m4a -ar 16000 -ac 1 -sample_fmt s16 test001.wav
```

## Hardware tips

- **Power**: add a 470µF electrolytic capacitor across VIN–GND (positive leg to VIN) close to the module. Class-D amps draw bursty current; without local bulk capacitance the 5V rail sags on loud passages
- **Heat**: if a 4Ω speaker warms up even during quiet passages, that is the ~300kHz ripple current of the filterless class-D output — keep speaker wires short, or add a ferrite bead in series with each output plus 470pF to ground
- **Never** place a capacitor across or in series with the speaker outputs (BTL bridge output — a large parallel capacitor can destroy the amplifier)

## Technical background

Pin mapping and initialization order were derived from:

- [ambpro2_sdk `i2s_api.c`](https://github.com/Freertos-kvs-LTS/ambpro2_sdk/blob/main/component/mbed/targets/hal/rtl8735b/i2s_api.c) — the fixed I2S0/I2S1 pin group definitions
- [ameba-rtos-pro2 `module_i2s.c`](https://github.com/Ameba-AIoT/ameba-rtos-pro2/blob/main/component/media/mmfv2/module_i2s.c) — the correct DMA page management and interrupt model

Tested on real hardware: HUB8735 Ultra with the ideasHatch AmebaPro2 4.0.15 Arduino package.

## License

MIT
