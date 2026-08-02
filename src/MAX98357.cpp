/*
 * MAX98357.cpp - implementation. See MAX98357.h for pin mapping.
 *
 * The sequence and interrupt model follow Realtek's production
 * module_i2s.c (ameba-rtos-pro2):
 *  - The low layer has no real TX-only mode: run I2S_DIR_TXRX, register
 *    BOTH irq handlers and arm all RX pages, then ignore RX data.
 *  - Pre-fill and send ALL TX pages BEFORE i2s_enable(); enabling with
 *    unowned pages causes an immediate "page own control error" storm.
 *  - The TX-done ISR must always refill and resend a page (silence when
 *    no data is ready) so the DMA never starves.
 *
 * Data path: source (SD / network / decoder) -> ring buffer (filled in
 * caller context) -> TX-done ISR copies ring slots into DMA pages.
 * Source reads never happen in the ISR; if the source is slower than
 * playback, the ISR plays silence instead of erroring out.
 */

#include "MAX98357.h"

#include "objects.h"     // struct i2s_s (hal_i2s_adapter_t)
#include "i2s_api.h"     // mbed I2S HAL API, implemented in liboutsrc.a
#include "PinNames.h"
#include "mp3dec.h"      // Helix MP3 decoder, implemented in libhmp3.a

#define I2S_DMA_PAGE_NUM  4
#define I2S_DMA_PAGE_SIZE 1024  // bytes per page, multiple of 64, max 4095

#define RING_SLOTS 8             // page-sized slots between producer and ISR

#define MP3_INBUF_SIZE  4096                 // >= MAINBUF_SIZE (1940)
#define MP3_OUTBUF_SAMP (1152 * 2)           // max samples per MP3 frame

static i2s_t s_i2s;
static uint8_t s_tx_buf[I2S_DMA_PAGE_NUM * I2S_DMA_PAGE_SIZE] __attribute__((aligned(32)));
static uint8_t s_rx_buf[I2S_DMA_PAGE_NUM * I2S_DMA_PAGE_SIZE] __attribute__((aligned(32)));
static uint8_t s_readbuf[I2S_DMA_PAGE_SIZE];

static uint8_t s_mp3in[MP3_INBUF_SIZE];
static int16_t s_mp3out[MP3_OUTBUF_SAMP];
static HMP3Decoder s_mp3dec = NULL;          // allocated once, kept forever

// SPSC ring: producer = caller context, consumer = TX-done ISR.
static uint8_t s_ring[RING_SLOTS][I2S_DMA_PAGE_SIZE];
static volatile uint32_t s_ring_head = 0;    // total slots produced
static volatile uint32_t s_ring_tail = 0;    // total slots consumed
static uint32_t s_slot_fill = 0;             // bytes written into current head slot

// Copy the next ring slot (or silence) into a DMA page and hand it back
// to the hardware. Shared by the pre-fill loop and the TX-done ISR.
static void fill_and_send_page(int *page)
{
    if (s_ring_head - s_ring_tail > 0) {
        memcpy(page, s_ring[s_ring_tail % RING_SLOTS], I2S_DMA_PAGE_SIZE);
        s_ring_tail = s_ring_tail + 1;
    } else {
        memset(page, 0, I2S_DMA_PAGE_SIZE);
    }
    i2s_send_page(&s_i2s, (uint32_t *)page);
}

static void tx_done_cb(uint32_t id, char *pbuf)
{
    (void)id;
    (void)pbuf;
    int *page = i2s_get_tx_page(&s_i2s);
    if (page) {
        fill_and_send_page(page);
    }
}

static void rx_done_cb(uint32_t id, char *pbuf)
{
    (void)id;
    (void)pbuf;
    // RX data is unused; just re-arm the page so the RX engine never
    // runs out (the low layer has no TX-only mode).
    i2s_recv_page(&s_i2s);
}

// ---- stream/file readers for the format decoders ----

static size_t readFromFile(void *ctx, uint8_t *buf, size_t len)
{
    File *f = (File *)ctx;
    int got = f->read(buf, len);
    return (got > 0) ? (size_t)got : 0;
}

// Forward-only read from an Arduino Stream (WiFiClient etc.). Returns
// once `len` bytes arrived or the stream stayed silent for ~1.5s.
static size_t readFromStream(void *ctx, uint8_t *buf, size_t len)
{
    Stream *s = (Stream *)ctx;
    size_t got = 0;
    uint32_t lastData = millis();
    while (got < len) {
        int avail = s->available();
        if (avail > 0) {
            int c = s->read();
            if (c < 0) {
                break;
            }
            buf[got++] = (uint8_t)c;
            lastData = millis();
        } else {
            if ((millis() - lastData) > 1500) {
                break;  // source dried up - treat as end of stream
            }
            delay(1);
        }
    }
    return got;
}

MAX98357::MAX98357()
    : _vol_q8(256), _inited(false), _outputActive(false),
      _streamChans(2), _streamRate(16000), _sdPin(-1), _err("")
{
}

void MAX98357::setShutdownPin(int pin)
{
    _sdPin = pin;
    if (_sdPin >= 0) {
        pinMode(_sdPin, OUTPUT);
        digitalWrite(_sdPin, LOW);  // hold the amp in shutdown until playback
    }
}

bool MAX98357::begin(void)
{
    if (_inited) {
        return true;
    }

    // Fixed I2S0 pin group: sck, ws, sd_tx, sd_rx, mck.
    // i2s_init() only accepts exactly this set and registers all 5 pins.
    i2s_init(&s_i2s, PF_13, PF_15, PF_14, PF_12, PF_11);
    i2s_set_format(&s_i2s, FORMAT_I2S);
    i2s_set_master(&s_i2s, I2S_MASTER);
    // Standard I2S timing (module_i2s.c "WS negative edge" setting):
    // the SDK default enables SCK inversion, undo it here.
    i2s_set_sck_inv(&s_i2s, 0);
    i2s_set_data_start_edge(&s_i2s, NEGATIVE_EDGE);
    i2s_tx_irq_handler(&s_i2s, tx_done_cb, (uint32_t)&s_i2s);
    i2s_rx_irq_handler(&s_i2s, rx_done_cb, (uint32_t)&s_i2s);

    _inited = true;
    return true;
}

void MAX98357::end(void)
{
    if (!_inited) {
        return;
    }
    i2s_disable(&s_i2s);
    i2s_deinit(&s_i2s);
    _inited = false;
}

void MAX98357::setVolume(float vol)
{
    if (vol < 0.0f) {
        vol = 0.0f;
    }
    if (vol > 1.0f) {
        vol = 1.0f;
    }
    _vol_q8 = (uint16_t)(vol * 256.0f);
}

const char *MAX98357::lastError(void) const
{
    return _err;
}

int MAX98357::mapSampleRate(uint32_t hz)
{
    switch (hz) {
        case 8000:  return SR_8KHZ;
        case 11025: return SR_11p025KHZ;
        case 12000: return SR_12KHZ;
        case 16000: return SR_16KHZ;
        case 22050: return SR_22p05KHZ;
        case 24000: return SR_24KHZ;
        case 32000: return SR_32KHZ;
        case 44100: return SR_44p1KHZ;
        case 48000: return SR_48KHZ;
        case 88200: return SR_88p2KHZ;
        case 96000: return SR_96KHZ;
        default:    return -1;
    }
}

// ---- output engine ----

bool MAX98357::startOutput(int srEnum)
{
    // Re-arm everything on every playback: i2s_disable() at the end of
    // the previous run resets the controller and page ownership.
    i2s_set_dma_buffer(&s_i2s, (char *)s_tx_buf, (char *)s_rx_buf,
                       I2S_DMA_PAGE_NUM, I2S_DMA_PAGE_SIZE);
    // Always output stereo: mono sources are duplicated to both channels.
    i2s_set_param(&s_i2s, CH_STEREO, srEnum, WL_16b);
    // No TX-only mode in the low layer - run TXRX and discard RX.
    i2s_set_direction(&s_i2s, I2S_DIR_TXRX);

    s_ring_head = 0;
    s_ring_tail = 0;
    s_slot_fill = 0;

    // Arm ALL RX pages and pre-send ALL TX pages, then enable - this
    // exact order comes from module_i2s.c / the official i2s example.
    for (int j = 0; j < I2S_DMA_PAGE_NUM; j++) {
        i2s_recv_page(&s_i2s);
        int *page = i2s_get_tx_page(&s_i2s);
        if (page) {
            fill_and_send_page(page);
        }
    }
    i2s_enable(&s_i2s);

    if (_sdPin >= 0) {
        digitalWrite(_sdPin, HIGH);
        delay(2);  // SD_MODE turn-on time; clocks are already running
    }
    _outputActive = true;
    return true;
}

void MAX98357::stopOutput(uint32_t sampleRate)
{
    if (!_outputActive) {
        return;
    }
    flushSlot();

    // Drain: wait for the ISR to consume the ring, then let the last
    // pages flush out of the DMA.
    uint32_t t0 = millis();
    while ((s_ring_head - s_ring_tail) > 0 && (millis() - t0) < 3000) {
        delay(1);
    }
    if (sampleRate == 0) {
        sampleRate = 16000;
    }
    uint32_t pageMs = (I2S_DMA_PAGE_SIZE * 1000UL) / (sampleRate * 4UL);
    delay(pageMs * (I2S_DMA_PAGE_NUM + 1) + 5);

    if (_sdPin >= 0) {
        digitalWrite(_sdPin, LOW);  // hardware shutdown between playbacks
    }
    i2s_disable(&s_i2s);
    _outputActive = false;
}

bool MAX98357::waitSlotFree(void)
{
    uint32_t t0 = millis();
    while ((s_ring_head - s_ring_tail) >= RING_SLOTS) {
        if ((millis() - t0) > 2000) {
            _err = "I2S TX stalled (no page consumed in 2s)";
            return false;
        }
        delay(1);
    }
    return true;
}

// Volume-scale samples into ring slots. count = number of int16 values
// in `samples` (interleaved when stereo). Mono input is duplicated.
bool MAX98357::pushBlock(const int16_t *samples, size_t count, bool stereo)
{
    for (size_t i = 0; i < count; i++) {
        int16_t v = (int16_t)(((int32_t)samples[i] * _vol_q8) >> 8);

        if (s_slot_fill == 0 && !waitSlotFree()) {
            return false;
        }
        int16_t *slot = (int16_t *)s_ring[s_ring_head % RING_SLOTS];
        slot[s_slot_fill >> 1] = v;
        s_slot_fill += 2;
        if (!stereo) {
            slot[s_slot_fill >> 1] = v;
            s_slot_fill += 2;
        }
        if (s_slot_fill >= I2S_DMA_PAGE_SIZE) {
            s_ring_head = s_ring_head + 1;  // publish full slot
            s_slot_fill = 0;
        }
    }
    return true;
}

// Zero-pad and publish a partially filled slot.
bool MAX98357::flushSlot(void)
{
    if (s_slot_fill == 0) {
        return true;
    }
    uint8_t *slot = s_ring[s_ring_head % RING_SLOTS];
    memset(slot + s_slot_fill, 0, I2S_DMA_PAGE_SIZE - s_slot_fill);
    s_ring_head = s_ring_head + 1;
    s_slot_fill = 0;
    return true;
}

// ---- raw PCM push API ----

bool MAX98357::beginPCM(uint32_t sampleRate, uint16_t channels)
{
    if (!_inited) {
        _err = "begin() not called";
        return false;
    }
    if (channels != 1 && channels != 2) {
        _err = "only mono or stereo supported";
        return false;
    }
    int sr = mapSampleRate(sampleRate);
    if (sr < 0) {
        _err = "unsupported sample rate";
        return false;
    }
    _streamChans = channels;
    _streamRate = sampleRate;
    return startOutput(sr);
}

bool MAX98357::writePCM(const int16_t *data, size_t sampleCount)
{
    if (!_outputActive) {
        _err = "beginPCM() not called";
        return false;
    }
    return pushBlock(data, sampleCount, _streamChans == 2);
}

void MAX98357::endPCM(void)
{
    stopOutput(_streamRate);
}

// ---- WAV ----

bool MAX98357::playWav(AmebaFatFS &fs, const char *filename)
{
    String path = String(fs.getRootPath()) + filename;
    File f = fs.open(path);
    if (!f) {
        _err = "cannot open file";
        return false;
    }
    bool ok = playWav(f);
    f.close();
    return ok;
}

bool MAX98357::playWav(File &f)
{
    f.seek(0);
    return playWavCommon(readFromFile, &f, true, &f);
}

bool MAX98357::playWavStream(Stream &s)
{
    return playWavCommon(readFromStream, &s, false, NULL);
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// Read exactly `want` bytes via rd(); returns bytes actually read.
static size_t rdFully(MAX98357::ReadFn rd, void *ctx, uint8_t *buf, size_t want)
{
    size_t got = 0;
    while (got < want) {
        size_t n = rd(ctx, buf + got, want - got);
        if (n == 0) {
            break;
        }
        got += n;
    }
    return got;
}

// Skip `n` bytes on a forward-only source.
static bool rdSkip(MAX98357::ReadFn rd, void *ctx, uint32_t n)
{
    while (n > 0) {
        size_t chunk = (n > sizeof(s_readbuf)) ? sizeof(s_readbuf) : n;
        if (rdFully(rd, ctx, s_readbuf, chunk) != chunk) {
            return false;
        }
        n -= chunk;
    }
    return true;
}

bool MAX98357::playWavCommon(ReadFn rd, void *ctx, bool canSeek, File *f)
{
    if (!_inited) {
        _err = "begin() not called";
        return false;
    }
    (void)canSeek;
    (void)f;

    uint8_t hdr[12];
    if (rdFully(rd, ctx, hdr, 12) != 12 ||
        memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
        _err = "not a RIFF/WAVE file";
        return false;
    }

    uint16_t audioFormat = 0, channels = 0, bits = 0;
    uint32_t sampleRate = 0, dataSize = 0;
    bool haveFmt = false;

    // Walk chunks (forward-only) until the "data" chunk.
    while (true) {
        uint8_t ch[8];
        if (rdFully(rd, ctx, ch, 8) != 8) {
            _err = "no data chunk found";
            return false;
        }
        uint32_t csize = le32(ch + 4);

        if (memcmp(ch, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            if (csize < 16 || rdFully(rd, ctx, fmt, 16) != 16) {
                _err = "bad fmt chunk";
                return false;
            }
            audioFormat = le16(fmt + 0);
            channels    = le16(fmt + 2);
            sampleRate  = le32(fmt + 4);
            bits        = le16(fmt + 14);
            haveFmt = true;
            if (csize > 16 && !rdSkip(rd, ctx, (csize - 16) + (csize & 1))) {
                _err = "bad fmt chunk";
                return false;
            }
        } else if (memcmp(ch, "data", 4) == 0) {
            dataSize = csize;
            break;
        } else {
            if (!rdSkip(rd, ctx, csize + (csize & 1))) {
                _err = "no data chunk found";
                return false;
            }
        }
    }

    if (!haveFmt) {
        _err = "fmt chunk missing";
        return false;
    }
    if (audioFormat != 1) {
        _err = "only PCM (uncompressed) WAV is supported";
        return false;
    }
    if (bits != 16) {
        _err = "only 16-bit WAV is supported";
        return false;
    }
    if (channels != 1 && channels != 2) {
        _err = "only mono or stereo WAV is supported";
        return false;
    }
    int sr = mapSampleRate(sampleRate);
    if (sr < 0) {
        _err = "unsupported sample rate";
        return false;
    }

    // Streaming TTS encoders often write 0 or 0xFFFFFFFF as the data
    // size - in that case play until the source ends.
    bool untilEof = (dataSize == 0 || dataSize == 0xFFFFFFFF);

    if (!startOutput(sr)) {
        return false;
    }

    bool ok = true;
    uint32_t remaining = dataSize;
    while (untilEof || remaining > 0) {
        uint32_t want = sizeof(s_readbuf);
        if (!untilEof && remaining < want) {
            want = remaining;
        }
        want &= ~(uint32_t)1;
        if (want == 0) {
            break;
        }
        size_t got = rdFully(rd, ctx, s_readbuf, want);
        if (got < 2) {
            break;  // end of source
        }
        if (!untilEof) {
            remaining -= (uint32_t)got;
        }
        if (!pushBlock((const int16_t *)s_readbuf, got / 2, channels == 2)) {
            ok = false;
            break;
        }
        if (got < want && untilEof) {
            break;
        }
    }

    stopOutput(sampleRate);
    return ok;
}

// ---- MP3 (Helix) ----

bool MAX98357::playMp3(AmebaFatFS &fs, const char *filename)
{
    String path = String(fs.getRootPath()) + filename;
    File f = fs.open(path);
    if (!f) {
        _err = "cannot open file";
        return false;
    }
    bool ok = playMp3(f);
    f.close();
    return ok;
}

bool MAX98357::playMp3(File &f)
{
    f.seek(0);
    return playMp3Common(readFromFile, &f);
}

bool MAX98357::playMp3Stream(Stream &s)
{
    return playMp3Common(readFromStream, &s);
}

bool MAX98357::playMp3Common(ReadFn rd, void *ctx)
{
    if (!_inited) {
        _err = "begin() not called";
        return false;
    }
    if (s_mp3dec == NULL) {
        s_mp3dec = MP3InitDecoder();
        if (s_mp3dec == NULL) {
            _err = "MP3 decoder alloc failed";
            return false;
        }
    }

    uint8_t *rp = s_mp3in;
    int bytesLeft = 0;
    bool eof = false;
    bool started = false;
    bool ok = true;
    uint32_t sampleRate = 0;
    int chans = 2;

    while (true) {
        // Refill: move the leftover to the front, top up from source.
        if (bytesLeft > 0 && rp != s_mp3in) {
            memmove(s_mp3in, rp, bytesLeft);
        }
        rp = s_mp3in;
        if (!eof && bytesLeft < (int)sizeof(s_mp3in)) {
            size_t got = rdFully(rd, ctx, s_mp3in + bytesLeft, sizeof(s_mp3in) - bytesLeft);
            if (got == 0) {
                eof = true;
            }
            bytesLeft += (int)got;
        }
        if (bytesLeft <= 0) {
            break;  // source exhausted and buffer empty
        }

        // Decode as many frames as the buffer allows.
        bool progressed = false;
        while (bytesLeft > 4) {
            int off = MP3FindSyncWord(rp, bytesLeft);
            if (off < 0) {
                bytesLeft = 0;  // no sync in the whole buffer - discard
                break;
            }
            rp += off;
            bytesLeft -= off;

            int err = MP3Decode(s_mp3dec, &rp, &bytesLeft, s_mp3out, 0);
            if (err == ERR_MP3_INDATA_UNDERFLOW || err == ERR_MP3_MAINDATA_UNDERFLOW) {
                break;  // need more data
            }
            if (err != ERR_MP3_NONE) {
                // Bad frame - skip one byte and resync.
                if (bytesLeft > 0) {
                    rp++;
                    bytesLeft--;
                }
                continue;
            }
            progressed = true;

            MP3FrameInfo fi;
            MP3GetLastFrameInfo(s_mp3dec, &fi);
            if (!started) {
                int sr = mapSampleRate((uint32_t)fi.samprate);
                if (sr < 0) {
                    _err = "unsupported sample rate";
                    return false;
                }
                sampleRate = (uint32_t)fi.samprate;
                chans = fi.nChans;
                if (!startOutput(sr)) {
                    return false;
                }
                started = true;
            }
            if (fi.outputSamps > 0) {
                if (!pushBlock(s_mp3out, (size_t)fi.outputSamps, chans == 2)) {
                    ok = false;
                    break;
                }
            }
            // Keep a safety margin so MP3Decode never reads past the
            // buffer end mid-frame.
            if (bytesLeft < MAINBUF_SIZE && !eof) {
                break;
            }
        }
        if (!ok) {
            break;
        }
        if (eof && (!progressed || bytesLeft <= 4)) {
            break;
        }
    }

    if (!started) {
        _err = "no decodable MP3 frames found";
        return false;
    }
    stopOutput(sampleRate);
    return ok;
}
