// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/**
 * @file sdr_radio.h
 * @brief The receiver the SDR app draws: settings in, spectra out.
 *
 * A worker owns the radio (ESP-SDR's snapshot capture on the board): it
 * captures a burst, turns it into a power spectrum and publishes it. The
 * app only sets what to receive and reads the newest spectrum; neither call
 * waits for the radio.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// FFT bins in a published spectrum, natural order shifted so bin 0 is the
/// lowest frequency (LO - rate/2) and bin SDR_BINS/2 the LO.
#define SDR_BINS 1024u

/// I/Q pairs published from each burst for the constellation: consecutive
/// samples, DC removed, full scale +-512.
#define SDR_IQ_POINTS 1024u

/// Sample rates the capture offers.
typedef enum { SDR_RATE_16M = 0, SDR_RATE_40M, SDR_RATE_80M, SDR_RATE_COUNT } sdr_rate_t;

typedef struct {
    uint16_t freq_mhz;      ///< LO, whole MHz (SDR_FREQ_MIN..SDR_FREQ_MAX)
    uint8_t  rate;          ///< sdr_rate_t
    int8_t   gain;          ///< -1 = the chip's AGC, else a PHY gain index (not dB)
    uint8_t  bandwidth_mhz; ///< analog filter, 0 = widest
    uint8_t  average;       ///< bursts averaged into one spectrum, 1..
} sdr_settings_t;

typedef struct {
    uint32_t seq;           ///< bumps with every published spectrum
    uint32_t rate_hz;
    uint16_t freq_mhz;      ///< the LO the spectrum was taken at
    uint8_t  gain_index;    ///< the gain the receiver used (the AGC's choice under AGC)
    uint8_t  spectra_per_s;
    uint32_t errors;        ///< captures that did not complete
} sdr_status_t;

#define SDR_FREQ_MIN 1842u
#define SDR_FREQ_MAX 2800u

/// Starts the worker on first call; later calls only apply @p s.
bool sdr_radio_start(const sdr_settings_t* s);
/// Applied before the next capture.
void sdr_radio_set(const sdr_settings_t* s);
/// Asks the worker to hand the radio back (Wi-Fi driver deinit) and end;
/// sdr_radio_stopped() says when it has. Leaving the firmware needs that RAM.
void sdr_radio_stop(void);
bool sdr_radio_stopped(void);
/**
 * With @p on, every published spectrum also prints one console line
 * "SDRPK <t_us> <kHz> <dBFS x 10>": when its first burst was captured and
 * its strongest bin away from the LO -- for timing a hopping transmitter on
 * the host.
 */
void sdr_radio_set_peak_log(bool on);
/// Highest manual gain index (0 until the radio is up).
unsigned sdr_radio_gain_max(void);
uint32_t sdr_radio_rate_hz(uint8_t rate);
/**
 * Copies the newest spectrum (SDR_BINS values, dBFS x 10) into @p out when it
 * is newer than @p seen_seq; returns false and leaves @p out alone otherwise.
 */
bool sdr_radio_read(uint32_t seen_seq, int16_t* out, sdr_status_t* st);
/**
 * Copies the newest burst's SDR_IQ_POINTS pairs (I, Q interleaved) into
 * @p iq when newer than @p seen_seq, which it then updates; false otherwise.
 */
bool sdr_radio_read_iq(uint32_t* seen_seq, int16_t* iq);

typedef struct {
    uint32_t count;         ///< bursts, SDR_CAPTURE_MAX_WORDS words each
    uint8_t  rate;          ///< sdr_rate_t
    uint16_t freq_mhz;
    int8_t   gain;
    uint8_t  bandwidth_mhz;
    uint16_t stride_us;     ///< extra delay between bursts (de-alias a periodic signal)
} sdr_dump_request_t;

/// Raw bursts back to back into PSRAM, then /sdcard/crosspad/sdr_dump.bin.
/// The worker must be running; false while a dump is pending or busy.
bool sdr_radio_dump_start(const sdr_dump_request_t* r);
void sdr_radio_dump_status(char* out, size_t n);

/// On-device LoRa (ELRS SF6/BW812.5) demod: capture `count` strided bursts, and
/// for each burst detect the strongest in-band channel, dechirp and emit its
/// symbol stream as text -- so the host pulls kilobytes of symbols, not
/// megabytes of I/Q. `stride_us` de-aliases the packet period like the dump.
bool sdr_radio_lora_start(const sdr_dump_request_t* r);
/// One status line (state/captured/err). The worker must be running.
void sdr_radio_lora_status(char* out, size_t n);
/// The demod text (one line per burst: "B<i> ch<c> <s0> <s1> ..."); NULL until
/// a run is done. `*len` is its byte length.
const char* sdr_radio_lora_text(size_t* len);

#ifdef __cplusplus
}
#endif
