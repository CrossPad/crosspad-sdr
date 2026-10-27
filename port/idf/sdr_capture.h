// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/* ESP-SDR's S3 snapshot capture without its serial protocol: the radio in
 * Wi-Fi NULL mode, tuned and gained here, one burst at a time into the
 * reserved SRAM bank. Called from one task only (sdr_worker.cpp). */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Largest burst: the 64 KiB bank less the dump engine's last words.
#define SDR_CAPTURE_MAX_WORDS 16380u

bool sdr_capture_init(void);
/// Stops and deinitialises the Wi-Fi driver: its internal RAM comes back.
void sdr_capture_deinit(void);
/// gain: -1 = AGC; bandwidth_mhz: 0 = the PHY's calibrated filter.
void sdr_capture_configure(unsigned mhz, int gain, unsigned bandwidth_mhz);
/// rate: sdr_rate_t. On success *words points at n words (I bits 0..9,
/// Q bits 10..19, gain index 20..27), valid until the next capture, and
/// *t_us is the esp_timer time the dump engine was triggered.
bool sdr_capture_burst(unsigned rate, unsigned n, const uint32_t** words, int64_t* t_us);
unsigned sdr_capture_gain_max(void);
/// While set, a ready receiver is not recalibrated between bursts (dense back-to-back snapshots).
void sdr_capture_hold_cal(bool on);
/// Last burst timing (us): prepare, done-wait, whole burst.
void sdr_capture_timing(uint32_t* prep, uint32_t* wait, uint32_t* burst);

#ifdef __cplusplus
}
#endif
