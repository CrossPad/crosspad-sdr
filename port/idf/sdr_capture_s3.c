// SPDX-License-Identifier: GPL-3.0-or-later
/* ESP32-S3 snapshot capture, after esp-sdr main/targets/esp32s3/receiver.c
 * (capture(), prepare_rx(), s3_tune(), the LPF and gain controls). The
 * register map and the PHY calls are that file's; see esp-sdr
 * docs/how-it-works.md. Only the snapshot path is taken: the ring mode masks
 * interrupts for a whole run, which the display, the encoder and the STM's
 * heartbeat cannot live with. */

#include "sdr_capture.h"

#include "esp_event.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "heap_memory_layout.h"
#include "soc/soc.h"

#include "rx_bandwidth.h"
#include "rx_lo.h"
#include "rx_recalibration.h"

/* The dump engine writes this 64 KiB bank (SRAM owner bit 2); the heap never
 * hands it out and sram_guard.ld keeps static data below it. */
#define IQ_BANK_BASE   0x3fcd0000u
#define IQ_BANK_END    0x3fce0000u
SOC_RESERVE_MEMORY_REGION(IQ_BANK_BASE, IQ_BANK_END, sdr_rf_dump);
#define IQ_BUFFER      ((uint32_t*)IQ_BANK_BASE)
#define IQ_MARKER      0xa5a0055au
#define SRAM_OWNER_REG 0x600c101cu
#define DUMP_CTRL_REG  0x60033d5cu
#define DUMP_LANE_REG  0x60033d90u
#define DUMP_CTRL_RUN  0x80000000u
#define DUMP_CTRL_TRIG (1u << 19)
#define DUMP_CTRL_DONE (1u << 18)
#define DUMP_TIMEOUT_US 20000
#define GAIN_REG       0x6001c02cu
#define GAIN_AGC_CODE  40u   /* esp-sdr's index while the AGC rules */
#define SDR_WIFI_RX_BUFS 2   /* the driver's minimum */
#define SDR_WIFI_TX_BUFS 2

extern void stop_tx_tone(unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(unsigned);
extern void set_chanfreq(unsigned, unsigned);
extern void set_rf_freq_offset(unsigned, unsigned, int);
extern void force_rx_gain(unsigned, unsigned, unsigned);

/* Required by the stock RF test archive; no shell is exposed. */
int cmd_parse(char* cmd, char* name, int* argc, char** argv)
{
    (void)cmd;
    (void)name;
    (void)argc;
    (void)argv;
    return -1;
}

static unsigned s_mhz = 2412;
static int s_gain = -1;
static int s_filter = -1;   /* -1 keeps the PHY-calibrated filter */
static bool s_rx_ready;
static unsigned s_rx_filter_saved[2];

static void tune(unsigned mhz)
{
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    rx_lo_plan_t plan = rx_lo_plan(mhz);
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
    rx_lo_select(false);
    set_chanfreq(channel ? mhz : 2412, 0);
    if (!channel) set_rf_freq_offset(0, plan.mhz, plan.offset_khz);
}

unsigned sdr_capture_gain_max(void)
{
    unsigned maximum = (REG_READ(GAIN_REG) >> 8) & 127u;
    return maximum <= 82u ? maximum : 0u;   /* never an uncalibrated slot */
}

static void gain_apply(void)
{
    const bool manual = s_gain >= 0;
    unsigned code = manual ? (unsigned)s_gain : GAIN_AGC_CODE;
    if (code > sdr_capture_gain_max()) code = sdr_capture_gain_max();
    force_rx_gain(manual, code, 0);
}

static bool s_hold_cal;   /* while set, a ready receiver is never recalibrated */

void sdr_capture_hold_cal(bool on) { s_hold_cal = on; }

static volatile uint32_t s_t_prepare_us, s_t_wait_us, s_t_burst_us;
void sdr_capture_timing(uint32_t* prep, uint32_t* wait, uint32_t* burst)
{ *prep = s_t_prepare_us; *wait = s_t_wait_us; *burst = s_t_burst_us; }

static void prepare_rx(void)
{
    if (s_rx_ready && (s_hold_cal || !rx_recalibration_stale())) return;
    tune(s_mhz);
    stop_tx_tone(1);
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    gain_apply();
    rx_lo_select(rx_lo_plan(s_mhz).alternate);
    esp_rom_delay_us(3000);
    s_rx_ready = true;
}

/* Applied only around a snapshot; restored before any retune. */
static void rx_filter_apply(void)
{
    if (s_filter < 0) return;   /* default filter: nothing to save/restore, and the
                                 * RF-chip I2C reads cost ~ms each -- skip them so a
                                 * dump's snapshots can run densely back to back */
    for (unsigned j = 0; j < 2; j++) {
        s_rx_filter_saved[j] = rom_chip_i2c_readReg(0x67, 0, 4 + j);
        rom_chip_i2c_writeReg(0x67, 0, 4 + j, (s_rx_filter_saved[j] & ~63u) | (unsigned)s_filter);
    }
}

static void rx_filter_restore(void)
{
    if (s_filter >= 0)
        for (unsigned j = 0; j < 2; j++) rom_chip_i2c_writeReg(0x67, 0, 4 + j, s_rx_filter_saved[j]);
}

bool sdr_capture_init(void)
{
    esp_err_t e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return false;
    /* The radio only receives into the dump bank: the fewest frame buffers
     * the driver takes, no aggregation -- they would come out of internal RAM. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.static_rx_buf_num = SDR_WIFI_RX_BUFS;
    cfg.dynamic_rx_buf_num = SDR_WIFI_RX_BUFS;
    if (cfg.static_tx_buf_num > SDR_WIFI_TX_BUFS) cfg.static_tx_buf_num = SDR_WIFI_TX_BUFS;
    cfg.ampdu_rx_enable = 0;
    cfg.ampdu_tx_enable = 0;
    cfg.nvs_enable = 0;
    if (esp_wifi_init(&cfg) != ESP_OK) return false;
    if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK || esp_wifi_set_mode(WIFI_MODE_NULL) != ESP_OK ||
        esp_wifi_start() != ESP_OK || esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK ||
        esp_wifi_set_promiscuous(true) != ESP_OK || esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE) != ESP_OK)
        return false;
    prepare_rx();
    return true;
}

void sdr_capture_deinit(void)
{
    esp_wifi_set_promiscuous(false);
    esp_wifi_stop();
    esp_wifi_deinit();
    s_rx_ready = false;
}

void sdr_capture_configure(unsigned mhz, int gain, unsigned bandwidth_mhz)
{
    s_mhz = mhz;
    s_gain = gain;
    s_filter = bandwidth_mhz ? (int)rx_bandwidth_dcap(bandwidth_mhz) : -1;
    s_rx_ready = false;
}

bool sdr_capture_burst(unsigned rate, unsigned n, const uint32_t** words, int64_t* t_us)
{
    if (n == 0 || n > SDR_CAPTURE_MAX_WORDS) return false;
    const uint32_t rate_bits = rate >= 100 ? (((rate - 100) & 0x7u) << 15)
                             : rate == 0   ? (1u << 16)
                             : rate == 1   ? (1u << 15)
                                           : 0;
    const int64_t t0 = esp_timer_get_time();
    prepare_rx();
    s_t_prepare_us = (uint32_t)(esp_timer_get_time() - t0);

    for (unsigned j = 0; j < n; j++) IQ_BUFFER[j] = IQ_MARKER;
    rx_filter_apply();
    const uint32_t owner = REG_READ(SRAM_OWNER_REG);
    const int64_t start = esp_timer_get_time();
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_LANE_REG, (0u << 0) | (1u << 6) | (2u << 12) | (3u << 18));
    REG_WRITE(SRAM_OWNER_REG, (owner & ~15u) | 4u);
    const uint32_t ctrl = DUMP_CTRL_RUN | rate_bits | n;
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    REG_WRITE(DUMP_CTRL_REG, ctrl | DUMP_CTRL_TRIG);
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    const int64_t tw = esp_timer_get_time();
    while (!(REG_READ(DUMP_CTRL_REG) & DUMP_CTRL_DONE) && esp_timer_get_time() - start < DUMP_TIMEOUT_US) {}
    s_t_wait_us = (uint32_t)(esp_timer_get_time() - tw);
    const bool done = (REG_READ(DUMP_CTRL_REG) & DUMP_CTRL_DONE) != 0;
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(SRAM_OWNER_REG, owner);
    rx_filter_restore();
    if (!done) return false;
    for (unsigned j = 0; j < n; j++)
        if (IQ_BUFFER[j] == IQ_MARKER) return false;
    *words = IQ_BUFFER;
    *t_us = start;
    s_t_burst_us = (uint32_t)(esp_timer_get_time() - t0);
    return true;
}