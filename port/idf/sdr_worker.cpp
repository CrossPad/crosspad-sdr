// SPDX-License-Identifier: GPL-3.0-or-later
/* The receiver worker behind sdr_radio.h: one task owns the radio, captures a
 * burst, averages the power of its FFT windows over `average` bursts and
 * publishes one spectrum. The app's calls only swap settings and copy the
 * newest spectrum out, under a spinlock held for a memcpy. */

#include "sdr_radio.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <crosspad/dsp/FftFactory.hpp>

#include "sdr_capture.h"

namespace {

const char* TAG = "SDR";

constexpr uint32_t kFftSize = SDR_BINS;
constexpr uint32_t kWindowsPerBurst = SDR_CAPTURE_MAX_WORDS / kFftSize;
constexpr uint32_t kBurstWords = kWindowsPerBurst * kFftSize;
constexpr float kFullScale = 512.0f;   // 10-bit I/Q
constexpr uint32_t kTaskStack = 6144;  // internal: the PHY writes its calibration to NVS
constexpr UBaseType_t kTaskPriority = 2;
constexpr BaseType_t kTaskCore = 1;
constexpr int64_t kRateWindowUs = 1000000;
constexpr uint32_t kDcGuardBins = 2;   // the LO's residue either side of the centre

constexpr uint32_t kRateHz[SDR_RATE_COUNT] = {16000000, 40000000, 80000000};

portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
sdr_settings_t s_want = {2437, SDR_RATE_40M, -1, 0, 4};
uint32_t s_wantSeq = 1;
int16_t* s_out = nullptr;   // published spectrum, SDR_BINS
sdr_status_t s_status = {};
int16_t* s_iq = nullptr;    // published constellation, 2 * SDR_IQ_POINTS
uint32_t s_iqSeq = 0;
std::atomic<unsigned> s_gainMax{0};
std::atomic<bool> s_peakLog{false};
std::atomic<bool> s_stopReq{false};
std::atomic<bool> s_stopped{false};
TaskHandle_t s_task = nullptr;

/* SDR_DUMP: bursts back to back into PSRAM, then one file on the card. */
enum : int { kDumpIdle, kDumpPending, kDumpBusy, kDumpDone, kDumpFailed };
constexpr const char* kDumpPath = "/sdcard/crosspad/sdr_dump.bin";
std::atomic<int> s_dumpState{kDumpIdle};
sdr_dump_request_t s_dumpReq = {};
uint32_t s_dumpCaptured = 0;
int64_t s_dumpSpanUs = 0;
const char* s_dumpError = "";

/* SDR_LORA: capture strided bursts, dechirp each to its symbol stream, emit
 * text. The host pulls symbols, not raw I/Q. */
enum : int { kLoraIdle, kLoraPending, kLoraBusy, kLoraDone, kLoraFailed };
std::atomic<int> s_loraState{kLoraIdle};
sdr_dump_request_t s_loraReq = {};
uint32_t s_loraCaptured = 0;
const char* s_loraError = "";
char* s_loraText = nullptr;      // PSRAM, the demod result
size_t s_loraLen = 0;
constexpr size_t kLoraTextCap = 48u * 1024u;

float* s_fft = nullptr;     // 2 * kFftSize, interleaved re/im
float* s_window = nullptr;
float* s_power = nullptr;
int16_t* s_db = nullptr;

void* fftAlloc(size_t bytes, size_t align) {
    return heap_caps_aligned_alloc(align, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
void fftLog(const char* line) { ESP_LOGW(TAG, "%s", line); }

inline int sext10(uint32_t v) { return static_cast<int32_t>(v << 22) >> 22; }

/// One burst's windows into s_power: DC removed per window, Q negated
/// (the S3's dump is the conjugate of I + jQ, esp-sdr docs/protocol.md).
void accumulate(crosspad::IFft& fft, const uint32_t* words) {
    for (uint32_t w = 0; w < kWindowsPerBurst; ++w) {
        const uint32_t* p = words + w * kFftSize;
        float mi = 0.0f, mq = 0.0f;
        for (uint32_t k = 0; k < kFftSize; ++k) {
            mi += static_cast<float>(sext10(p[k]));
            mq += static_cast<float>(sext10(p[k] >> 10));
        }
        mi /= kFftSize;
        mq /= kFftSize;
        for (uint32_t k = 0; k < kFftSize; ++k) {
            const float g = s_window[k] / kFullScale;
            s_fft[2 * k] = (static_cast<float>(sext10(p[k])) - mi) * g;
            s_fft[2 * k + 1] = -(static_cast<float>(sext10(p[k] >> 10)) - mq) * g;
        }
        fft.forward(s_fft, kFftSize);
        for (uint32_t k = 0; k < kFftSize; ++k)
            s_power[k] += s_fft[2 * k] * s_fft[2 * k] + s_fft[2 * k + 1] * s_fft[2 * k + 1];
    }
}

/// The burst's first SDR_IQ_POINTS samples into s_iq, the same conjugation
/// and DC removal as the spectrum's.
void publishIq(const uint32_t* words) {
    int32_t si = 0, sq = 0;
    for (uint32_t k = 0; k < SDR_IQ_POINTS; ++k) {
        si += sext10(words[k]);
        sq += sext10(words[k] >> 10);
    }
    const int32_t mi = si / static_cast<int32_t>(SDR_IQ_POINTS);
    const int32_t mq = sq / static_cast<int32_t>(SDR_IQ_POINTS);
    portENTER_CRITICAL(&s_lock);
    for (uint32_t k = 0; k < SDR_IQ_POINTS; ++k) {
        s_iq[2 * k] = static_cast<int16_t>(sext10(words[k]) - mi);
        s_iq[2 * k + 1] = static_cast<int16_t>(-(sext10(words[k] >> 10) - mq));
    }
    ++s_iqSeq;
    portEXIT_CRITICAL(&s_lock);
}

/// The published spectrum's strongest bin away from the LO, on the console.
void logPeak(int64_t t_us, const sdr_settings_t& rx) {
    uint32_t peak = 0;
    for (uint32_t k = 0; k < kFftSize; ++k) {
        if (k + kDcGuardBins >= kFftSize / 2 && k <= kFftSize / 2 + kDcGuardBins) continue;
        if (s_db[k] > s_db[peak]) peak = k;
    }
    const int64_t khz = static_cast<int64_t>(rx.freq_mhz) * 1000 +
                        (static_cast<int64_t>(peak) - kFftSize / 2) * kRateHz[rx.rate] / kFftSize / 1000;
    std::printf("SDRPK %lld %lld %d\n", static_cast<long long>(t_us), static_cast<long long>(khz), s_db[peak]);
}

/// s_power (summed over `count` windows) into dBFS x 10, the LO in the middle.
void toDb(uint32_t count) {
    // A full-scale tone in the centre of a bin: N * 0.5 (Hann coherent gain).
    const float ref = 10.0f * std::log10(static_cast<float>(kFftSize) * kFftSize * 0.25f) +
                      10.0f * std::log10(static_cast<float>(count));
    for (uint32_t k = 0; k < kFftSize; ++k) {
        const float p = s_power[(k + kFftSize / 2) % kFftSize];
        const float db = p > 0.0f ? 10.0f * std::log10(p) - ref : -200.0f;
        s_db[k] = static_cast<int16_t>(std::lround(db * 10.0f));
    }
}

struct DumpHeader {
    char magic[4];
    uint32_t version, count, words, rate_hz, freq_mhz;
    int32_t gain;
    uint32_t bandwidth_mhz;
};

const char* runDump(const sdr_dump_request_t& r) {
    sdr_capture_configure(r.freq_mhz, r.gain, r.bandwidth_mhz);
    sdr_capture_hold_cal(true);
    constexpr uint32_t words = SDR_CAPTURE_MAX_WORDS;
    constexpr uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    auto* buf = static_cast<uint32_t*>(heap_caps_malloc(static_cast<size_t>(r.count) * words * 4, caps));
    auto* ts = static_cast<int64_t*>(heap_caps_malloc(r.count * sizeof(int64_t), caps));
    const char* err = nullptr;
    if (!buf || !ts) err = "no_psram";
    for (uint32_t i = 0; !err && i < r.count; ++i) {
        const uint32_t* w = nullptr;
        int tries = 0;
        while (!sdr_capture_burst(r.rate, words, &w, &ts[i]))
            if (++tries == 3) break;
        if (tries == 3) { err = "capture"; break; }
        std::memcpy(buf + static_cast<size_t>(i) * words, w, words * 4);
        s_dumpCaptured = i + 1;
        if (r.stride_us) esp_rom_delay_us(r.stride_us);   // de-alias a periodic signal
    }
    if (!err) {
        s_dumpSpanUs = ts[r.count - 1] - ts[0];
        FILE* f = std::fopen(kDumpPath, "wb");
        if (!f) {
            err = "fopen";
        } else {
            const DumpHeader h = {{'S', 'D', 'R', 'D'}, 1, r.count, words, r.rate < SDR_RATE_COUNT ? kRateHz[r.rate] : 0, r.freq_mhz, r.gain,
                                  r.bandwidth_mhz};
            const size_t data = static_cast<size_t>(r.count) * words * 4;
            if (std::fwrite(&h, sizeof h, 1, f) != 1 || std::fwrite(ts, sizeof(int64_t), r.count, f) != r.count ||
                std::fwrite(buf, 1, data, f) != data)
                err = "fwrite";
            std::fclose(f);
        }
    }
    sdr_capture_hold_cal(false);
    heap_caps_free(buf);
    heap_caps_free(ts);
    return err;
}

/* ---- On-device ELRS LoRa demod (SF6, BW 812.5 kHz, 16 MS/s capture) -------
 * Mirrors the host prototype proven against the raw dumps: pick the strongest
 * in-band ELRS channel, mix it to DC, boxcar-decimate 16->1.6 MS/s, then
 * dechirp each symbol window (conjugate up-chirp * FFT, fold to 64 bins). The
 * host assembles the packet from many such strided windows. */
namespace lora {
constexpr float kBw = 812500.0f;
constexpr uint32_t kN = 64;                 // chips per symbol (SF6)
constexpr uint32_t kDecim = 10;             // 16 MS/s -> 1.6 MS/s
constexpr float kFsd = 16000000.0f / kDecim;
constexpr uint32_t kSymD = 126;             // round(kN/kBw*kFsd)
constexpr uint32_t kNfft = 128;             // dechirp FFT (kSymD zero-padded)
constexpr double kCh0Hz = 2400.4e6, kChStepHz = 1.0e6;
constexpr int kChCount = 80;
constexpr float kSnrMin = 6.0f;

float* s_cbuf = nullptr;     // 2 * SDR_CAPTURE_MAX_WORDS, mixed signal
float* s_dec = nullptr;      // 2 * (SDR_CAPTURE_MAX_WORDS/kDecim), decimated
float* s_chirp = nullptr;    // 2 * kSymD, conjugate up-chirp
float* s_sf = nullptr;       // 2 * kNfft, FFT scratch

bool alloc() {
    if (s_cbuf) return true;
    s_cbuf = static_cast<float*>(heap_caps_malloc(2u * SDR_CAPTURE_MAX_WORDS * sizeof(float), MALLOC_CAP_SPIRAM));
    s_dec = static_cast<float*>(heap_caps_malloc(2u * (SDR_CAPTURE_MAX_WORDS / kDecim + 1) * sizeof(float), MALLOC_CAP_SPIRAM));
    s_chirp = static_cast<float*>(heap_caps_malloc(2u * kSymD * sizeof(float), MALLOC_CAP_SPIRAM));
    s_sf = static_cast<float*>(heap_caps_malloc(2u * kNfft * sizeof(float), MALLOC_CAP_SPIRAM));
    if (!s_cbuf || !s_dec || !s_chirp || !s_sf) return false;
    const float T = kN / kBw, f0 = -kBw / 2.0f;
    for (uint32_t n = 0; n < kSymD; ++n) {    // conjugate up-chirp
        const float t = n / kFsd;
        const float ph = 2.0f * (float)M_PI * (f0 * t + kBw / (2.0f * T) * t * t);
        s_chirp[2 * n] = std::cos(ph);
        s_chirp[2 * n + 1] = -std::sin(ph);
    }
    return true;
}

/// Convert one burst's words to conjugated, DC-removed complex floats in s_cbuf.
void toComplex(const uint32_t* words) {
    float mi = 0.0f, mq = 0.0f;
    for (uint32_t k = 0; k < SDR_CAPTURE_MAX_WORDS; ++k) {
        mi += (float)sext10(words[k]);
        mq += (float)sext10(words[k] >> 10);
    }
    mi /= SDR_CAPTURE_MAX_WORDS;
    mq /= SDR_CAPTURE_MAX_WORDS;
    for (uint32_t k = 0; k < SDR_CAPTURE_MAX_WORDS; ++k) {
        s_cbuf[2 * k] = (float)sext10(words[k]) - mi;
        s_cbuf[2 * k + 1] = -((float)sext10(words[k] >> 10) - mq);   // conjugate (S3 dump)
    }
}

/// Strongest in-band ELRS channel index, or -1. Coarse power via 1024-pt FFTs.
int pickChannel(crosspad::IFft& fft, uint32_t loMhz) {
    const double lo = (double)loMhz * 1e6;
    const uint32_t nf = 1024, nw = SDR_CAPTURE_MAX_WORDS / nf;
    for (uint32_t b = 0; b < nf; ++b) s_power[b] = 0.0f;
    for (uint32_t w = 0; w < nw; ++w) {
        for (uint32_t k = 0; k < nf; ++k) {
            s_fft[2 * k] = s_cbuf[2 * (w * nf + k)];
            s_fft[2 * k + 1] = s_cbuf[2 * (w * nf + k) + 1];
        }
        fft.forward(s_fft, nf);
        for (uint32_t k = 0; k < nf; ++k) s_power[k] += s_fft[2 * k] * s_fft[2 * k] + s_fft[2 * k + 1] * s_fft[2 * k + 1];
    }
    // per-channel energy = sum of bins within +/- BW/2 of the channel centre
    float best = 0.0f, medacc = 0.0f;
    int bestch = -1, nseen = 0;
    float chpow[kChCount];
    for (int c = 0; c < kChCount; ++c) {
        const double fc = kCh0Hz + c * kChStepHz;
        if (std::fabs(fc - lo) > 16e6 / 2 - kBw) { chpow[c] = -1.0f; continue; }
        float p = 0.0f;
        for (uint32_t k = 0; k < nf; ++k) {
            const double f = (k < nf / 2 ? (double)k : (double)k - nf) * 16e6 / nf;   // baseband bin freq
            if (std::fabs(lo + f - fc) < kBw / 2) p += s_power[k];
        }
        chpow[c] = p;
        medacc += p;
        ++nseen;
        if (p > best) { best = p; bestch = c; }
    }
    if (bestch < 0 || nseen == 0) return -1;
    const float mean = medacc / nseen;
    return (best > 4.0f * mean) ? bestch : -1;    // gate: a real packet stands out
}

/// Dechirp s_dec (already conjugated + decimated), append symbols for one burst.
void dechirpAppend(crosspad::IFft& fft, uint32_t ndec, int ch, char*& p, char* end) {
    // best coarse sample phase
    int bo = 0;
    float best = -1.0f;
    for (uint32_t ph = 0; ph < kSymD; ph += 4) {
        float s = 0.0f;
        for (uint32_t k = 0; ph + (k + 1) * kSymD <= ndec; ++k) {
            const float* seg = s_dec + 2 * (ph + k * kSymD);
            for (uint32_t i = 0; i < kNfft; ++i) {
                if (i < kSymD) {
                    s_sf[2 * i] = seg[2 * i] * s_chirp[2 * i] - seg[2 * i + 1] * s_chirp[2 * i + 1];
                    s_sf[2 * i + 1] = seg[2 * i] * s_chirp[2 * i + 1] + seg[2 * i + 1] * s_chirp[2 * i];
                } else { s_sf[2 * i] = s_sf[2 * i + 1] = 0.0f; }
            }
            fft.forward(s_sf, kNfft);
            float mx = 0.0f, sum = 0.0f;
            for (uint32_t i = 0; i < kN; ++i) {
                const float m = s_sf[2 * i] * s_sf[2 * i] + s_sf[2 * i + 1] * s_sf[2 * i + 1] +
                                s_sf[2 * (i + kN)] * s_sf[2 * (i + kN)] + s_sf[2 * (i + kN) + 1] * s_sf[2 * (i + kN) + 1];
                sum += m;
                if (m > mx) mx = m;
            }
            s += mx / (sum / kN + 1e-9f);
        }
        if (s > best) { best = s; bo = (int)ph; }
    }
    int n = std::snprintf(p, end - p, "ch%d", ch);
    p += (n > 0 ? n : 0);
    for (uint32_t k = 0; bo + (k + 1) * kSymD <= ndec; ++k) {
        const float* seg = s_dec + 2 * (bo + k * kSymD);
        for (uint32_t i = 0; i < kNfft; ++i) {
            if (i < kSymD) {
                s_sf[2 * i] = seg[2 * i] * s_chirp[2 * i] - seg[2 * i + 1] * s_chirp[2 * i + 1];
                s_sf[2 * i + 1] = seg[2 * i] * s_chirp[2 * i + 1] + seg[2 * i + 1] * s_chirp[2 * i];
            } else { s_sf[2 * i] = s_sf[2 * i + 1] = 0.0f; }
        }
        fft.forward(s_sf, kNfft);
        float mx = 0.0f, sum = 0.0f; uint32_t arg = 0;
        for (uint32_t i = 0; i < kN; ++i) {
            const float m = s_sf[2 * i] * s_sf[2 * i] + s_sf[2 * i + 1] * s_sf[2 * i + 1] +
                            s_sf[2 * (i + kN)] * s_sf[2 * (i + kN)] + s_sf[2 * (i + kN) + 1] * s_sf[2 * (i + kN) + 1];
            sum += m;
            if (m > mx) { mx = m; arg = i; }
        }
        const float snr = mx / (sum / kN + 1e-9f);
        n = std::snprintf(p, end - p, " %d", snr >= kSnrMin ? (int)arg : 255);
        p += (n > 0 ? n : 0);
    }
    n = std::snprintf(p, end - p, "\n");
    p += (n > 0 ? n : 0);
}

/// One burst: pick channel, mix, decimate, dechirp, append its line.
void demodBurst(crosspad::IFft& fft, const uint32_t* words, uint32_t loMhz, uint32_t idx, char*& p, char* end) {
    toComplex(words);
    const int ch = pickChannel(fft, loMhz);
    int n = std::snprintf(p, end - p, "B%lu ", (unsigned long)idx);
    p += (n > 0 ? n : 0);
    if (ch < 0) { n = std::snprintf(p, end - p, "-\n"); p += (n > 0 ? n : 0); return; }
    // mix channel to DC
    const double fc = kCh0Hz + ch * kChStepHz;
    const float dphi = -2.0f * (float)M_PI * (float)((fc - (double)loMhz * 1e6) / 16e6);
    float cr = 1.0f, ci = 0.0f;
    const float wr = std::cos(dphi), wi = std::sin(dphi);
    for (uint32_t k = 0; k < SDR_CAPTURE_MAX_WORDS; ++k) {
        const float xr = s_cbuf[2 * k], xi = s_cbuf[2 * k + 1];
        s_cbuf[2 * k] = xr * cr - xi * ci;
        s_cbuf[2 * k + 1] = xr * ci + xi * cr;
        const float ncr = cr * wr - ci * wi, nci = cr * wi + ci * wr;
        cr = ncr; ci = nci;
    }
    // boxcar decimate by kDecim, conjugate (match host)
    const uint32_t ndec = SDR_CAPTURE_MAX_WORDS / kDecim;
    for (uint32_t m = 0; m < ndec; ++m) {
        float ar = 0.0f, ai = 0.0f;
        for (uint32_t j = 0; j < kDecim; ++j) { ar += s_cbuf[2 * (m * kDecim + j)]; ai += s_cbuf[2 * (m * kDecim + j) + 1]; }
        s_dec[2 * m] = ar / kDecim;
        s_dec[2 * m + 1] = -ai / kDecim;    // conjugate
    }
    dechirpAppend(fft, ndec, ch, p, end);
}
}  // namespace lora

const char* runLora(crosspad::IFft& fft, const sdr_dump_request_t& r) {
    if (!lora::alloc()) return "no_psram";
    if (!s_loraText) s_loraText = static_cast<char*>(heap_caps_malloc(kLoraTextCap, MALLOC_CAP_SPIRAM));
    if (!s_loraText) return "no_psram";
    sdr_capture_configure(r.freq_mhz, r.gain, r.bandwidth_mhz);
    sdr_capture_hold_cal(true);
    char* p = s_loraText;
    char* end = s_loraText + kLoraTextCap - 1;
    const char* err = nullptr;
    for (uint32_t i = 0; !err && i < r.count; ++i) {
        const uint32_t* w = nullptr;
        int64_t ts = 0;
        int tries = 0;
        while (!sdr_capture_burst(r.rate, SDR_CAPTURE_MAX_WORDS, &w, &ts))
            if (++tries == 3) break;
        if (tries == 3) { err = "capture"; break; }
        lora::demodBurst(fft, w, r.freq_mhz, i, p, end);
        s_loraCaptured = i + 1;
        if (end - p < 512) break;    // buffer nearly full
        if (r.stride_us) esp_rom_delay_us(r.stride_us);
    }
    *p = '\0';
    s_loraLen = (size_t)(p - s_loraText);
    sdr_capture_hold_cal(false);
    return err;
}

void worker(void*) {
    if (!sdr_capture_init()) {
        ESP_LOGE(TAG, "radio init failed");
        sdr_capture_deinit();
        s_stopped.store(true);
        vTaskDelete(nullptr);
        return;
    }
    s_gainMax.store(sdr_capture_gain_max());
    crosspad::FftMemory mem;
    mem.alloc = fftAlloc;
    mem.free = heap_caps_free;
    mem.log = fftLog;
    crosspad::IFft& fft = crosspad::createFft(mem);

    sdr_settings_t cur = {};
    uint32_t seenSeq = 0, bursts = 0, published = 0, errors = 0, perSec = 0;
    uint8_t gainIndex = 0;
    int64_t firstUs = 0;
    int64_t rateStart = esp_timer_get_time();
    while (!s_stopReq.load()) {
        sdr_settings_t want;
        uint32_t wantSeq;
        portENTER_CRITICAL(&s_lock);
        want = s_want;
        wantSeq = s_wantSeq;
        portEXIT_CRITICAL(&s_lock);
        if (s_dumpState.load() == kDumpPending) {
            s_dumpState.store(kDumpBusy);
            s_dumpError = runDump(s_dumpReq);
            s_dumpState.store(s_dumpError ? kDumpFailed : kDumpDone);
            if (!s_dumpError) s_dumpError = "";
            cur.freq_mhz = 0;   // the dump retuned: apply the app's settings again
            seenSeq = 0;
        }
        if (s_loraState.load() == kLoraPending) {
            s_loraState.store(kLoraBusy);
            s_loraCaptured = 0;
            s_loraError = runLora(fft, s_loraReq);
            s_loraState.store(s_loraError ? kLoraFailed : kLoraDone);
            if (!s_loraError) s_loraError = "";
            cur.freq_mhz = 0;
            seenSeq = 0;
        }
        if (wantSeq != seenSeq) {
            if (want.freq_mhz != cur.freq_mhz || want.gain != cur.gain || want.bandwidth_mhz != cur.bandwidth_mhz)
                sdr_capture_configure(want.freq_mhz, want.gain, want.bandwidth_mhz);
            cur = want;
            seenSeq = wantSeq;
            bursts = 0;
            std::memset(s_power, 0, kFftSize * sizeof(float));
        }

        const uint32_t* words = nullptr;
        int64_t burstUs = 0;
        if (!sdr_capture_burst(cur.rate, kBurstWords, &words, &burstUs)) {
            ++errors;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (bursts == 0) {
            gainIndex = static_cast<uint8_t>(words[0] >> 20);
            firstUs = burstUs;
        }
        publishIq(words);
        accumulate(fft, words);
        if (++bursts >= (cur.average ? cur.average : 1u)) {
            toDb(bursts * kWindowsPerBurst);
            if (s_peakLog.load()) logPeak(firstUs, cur);
            bursts = 0;
            std::memset(s_power, 0, kFftSize * sizeof(float));
            ++perSec;
            portENTER_CRITICAL(&s_lock);
            std::memcpy(s_out, s_db, kFftSize * sizeof(int16_t));
            s_status.seq = ++published;
            s_status.rate_hz = kRateHz[cur.rate];
            s_status.freq_mhz = cur.freq_mhz;
            s_status.gain_index = gainIndex;
            s_status.errors = errors;
            portEXIT_CRITICAL(&s_lock);
        }
        const int64_t now = esp_timer_get_time();
        if (now - rateStart >= kRateWindowUs) {
            portENTER_CRITICAL(&s_lock);
            s_status.spectra_per_s = static_cast<uint8_t>(perSec > 255 ? 255 : perSec);
            portEXIT_CRITICAL(&s_lock);
            perSec = 0;
            rateStart = now;
        }
        vTaskDelay(1);   // the capture and the FFTs busy core 1; give its other tasks a tick
    }
    sdr_capture_deinit();
    ESP_LOGI(TAG, "radio handed back");
    s_stopped.store(true);
    vTaskDelete(nullptr);   // the idle task frees this internal stack too
}

/// PSRAM: the internal RAM left beside the dump bank and the radio is the
/// platform's (its file worker alone needs a 5 kB block).
bool allocate() {
    constexpr uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    s_fft = static_cast<float*>(heap_caps_aligned_alloc(16, 2 * kFftSize * sizeof(float), caps));
    s_window = static_cast<float*>(heap_caps_malloc(kFftSize * sizeof(float), caps));
    s_power = static_cast<float*>(heap_caps_calloc(kFftSize, sizeof(float), caps));
    s_db = static_cast<int16_t*>(heap_caps_malloc(kFftSize * sizeof(int16_t), caps));
    s_out = static_cast<int16_t*>(heap_caps_calloc(kFftSize, sizeof(int16_t), caps));
    s_iq = static_cast<int16_t*>(heap_caps_calloc(2 * SDR_IQ_POINTS, sizeof(int16_t), caps));
    if (!s_fft || !s_window || !s_power || !s_db || !s_out || !s_iq) return false;
    for (uint32_t k = 0; k < kFftSize; ++k)
        s_window[k] = 0.5f - 0.5f * std::cos(2.0f * static_cast<float>(M_PI) * k / kFftSize);
    return true;
}

} // namespace

extern "C" bool sdr_radio_start(const sdr_settings_t* s) {
    if (s) sdr_radio_set(s);
    if (s_task) return true;
    if (!allocate()) {
        ESP_LOGE(TAG, "no internal RAM for the spectrum buffers");
        return false;
    }
    if (xTaskCreatePinnedToCore(worker, "sdr", kTaskStack, nullptr, kTaskPriority, &s_task, kTaskCore) != pdPASS) {
        s_task = nullptr;
        return false;
    }
    return true;
}

extern "C" void sdr_radio_set(const sdr_settings_t* s) {
    portENTER_CRITICAL(&s_lock);
    s_want = *s;
    ++s_wantSeq;
    portEXIT_CRITICAL(&s_lock);
}

extern "C" void sdr_radio_stop(void) { s_stopReq.store(true); }

extern "C" void sdr_radio_set_peak_log(bool on) { s_peakLog.store(on); }

extern "C" bool sdr_radio_stopped(void) { return !s_task || s_stopped.load(); }

extern "C" unsigned sdr_radio_gain_max(void) { return s_gainMax.load(); }

extern "C" uint32_t sdr_radio_rate_hz(uint8_t rate) { return rate < SDR_RATE_COUNT ? kRateHz[rate] : 0; }

extern "C" bool sdr_radio_read_iq(uint32_t* seen_seq, int16_t* iq) {
    if (!s_iq) return false;
    portENTER_CRITICAL(&s_lock);
    const bool fresh = s_iqSeq != *seen_seq;
    if (fresh) {
        std::memcpy(iq, s_iq, 2 * SDR_IQ_POINTS * sizeof(int16_t));
        *seen_seq = s_iqSeq;
    }
    portEXIT_CRITICAL(&s_lock);
    return fresh;
}

extern "C" bool sdr_radio_read(uint32_t seen_seq, int16_t* out, sdr_status_t* st) {
    if (!s_out) return false;
    portENTER_CRITICAL(&s_lock);
    const bool fresh = s_status.seq != seen_seq;
    if (fresh) {
        std::memcpy(out, s_out, SDR_BINS * sizeof(int16_t));
        if (st) *st = s_status;
    }
    portEXIT_CRITICAL(&s_lock);
    return fresh;
}

extern "C" bool sdr_radio_dump_start(const sdr_dump_request_t* r) {
    if (!s_task || s_stopped.load() || r->count == 0) return false;
    if (r->rate >= SDR_RATE_COUNT && r->rate < 100) return false;
    int idle = s_dumpState.load();
    if (idle == kDumpPending || idle == kDumpBusy) return false;
    s_dumpReq = *r;
    s_dumpCaptured = 0;
    s_dumpSpanUs = 0;
    s_dumpState.store(kDumpPending);
    return true;
}

extern "C" void sdr_radio_dump_status(char* out, size_t n) {
    static const char* const names[] = {"idle", "pending", "busy", "done", "failed"};
    uint32_t prep = 0, wait = 0, burst = 0;
    sdr_capture_timing(&prep, &wait, &burst);
    std::snprintf(out, n, "SDR_DUMP: state=%s captured=%u span_us=%lld prep_us=%lu wait_us=%lu burst_us=%lu err=%s file=%s\r\n",
                  names[s_dumpState.load()], static_cast<unsigned>(s_dumpCaptured),
                  static_cast<long long>(s_dumpSpanUs), (unsigned long)prep, (unsigned long)wait, (unsigned long)burst, s_dumpError, kDumpPath);
}

extern "C" bool sdr_radio_lora_start(const sdr_dump_request_t* r) {
    if (!s_task || s_stopped.load() || r->count == 0) return false;
    if (r->rate >= SDR_RATE_COUNT && r->rate < 100) return false;
    int idle = s_loraState.load();
    if (idle == kLoraPending || idle == kLoraBusy) return false;
    s_loraReq = *r;
    s_loraCaptured = 0;
    s_loraLen = 0;
    s_loraState.store(kLoraPending);
    return true;
}

extern "C" void sdr_radio_lora_status(char* out, size_t n) {
    static const char* const names[] = {"idle", "pending", "busy", "done", "failed"};
    std::snprintf(out, n, "SDR_LORA: state=%s captured=%u bytes=%lu err=%s\r\n",
                  names[s_loraState.load()], static_cast<unsigned>(s_loraCaptured),
                  (unsigned long)s_loraLen, s_loraError);
}

extern "C" const char* sdr_radio_lora_text(size_t* len) {
    if (s_loraState.load() != kLoraDone) { if (len) *len = 0; return nullptr; }
    if (len) *len = s_loraLen;
    return s_loraText;
}
