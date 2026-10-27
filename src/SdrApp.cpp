// SPDX-License-Identifier: GPL-3.0-or-later
/**
 * @file SdrApp.cpp
 * @brief The SDR kit's one app: what the radio receives (ESP-SDR,
 *        sdr_radio.h) on the panel -- the spectrum, its waterfall, or the I/Q
 *        constellation as fading dots -- every receiver setting on the
 *        encoder, and Quit as the way back to the other firmware slot.
 *
 * The settings sit in two rows of chips; the second row holds the view's
 * own (VIEW switches it). Turning moves the highlight; a click edits the
 * highlighted one (turning then changes it) and a second click stops
 * editing; a click on QUIT leaves. Power stops editing, else puts the
 * highlight on QUIT. Everything runs on the LVGL thread: an lv_timer copies
 * the worker's newest spectrum into the chart or the waterfall, or its newest burst into the
 * constellation.
 */

#if USE_LVGL

#include <cmath>
#include <cstdio>
#include <cstring>

#include <crosspad/app/AppRegistrar.hpp>

#include "crosspad-gui/components/app_icon.h"
#include "crosspad-gui/components/power_gesture.h"
#include "crosspad-gui/fw/IFwViewService.h"
#include "crosspad-gui/platform/IGuiPlatform.h"
#include "crosspad-gui/widgets/gui_helpers.h"

#include "lvgl.h"

#include "sdr_radio.h"

class App;

namespace {

constexpr uint32_t kTickMs = 50;
constexpr int32_t kChipH = 21;
constexpr int32_t kChipGap = 2;
constexpr int32_t kChipPad = 5;
constexpr int32_t kChartY = 2 * (kChipH + kChipGap) + 2;
constexpr int32_t kFooterH = 32;
constexpr int kDcGuardBins = 2;   // the LO's residue either side of the centre, left out of the peak
constexpr float kIqFullScale = 512.0f;
constexpr uint8_t kDotHit = 96;   // what one sample adds to its pixel; overlapping samples glow brighter
constexpr uint32_t kMaxCols = 480;

constexpr uint32_t kColChip = 0x1C1C1C;
constexpr uint32_t kColSelect = 0x3FA9FF;
constexpr uint32_t kColText = 0xD0D0D0;
constexpr uint32_t kColGrid = 0x20303A;
constexpr uint32_t kColLive = 0xFFD040;
constexpr uint32_t kColHold = 0xFF4040;

enum Param : int { Freq, Step, Rate, Gain, Bw, View, Avg, Hold, Ref, Span, Log, Fade, Zoom, Quit, kParams };
enum class ViewMode : uint8_t { Spectrum, Waterfall, Iq, kCount };

/// Which chips each view shows on the second row (the first is common).
constexpr bool inRow2(Param p, ViewMode v) {
    switch (p) {
        case View: case Quit: return true;
        case Avg: case Ref: case Span: return v != ViewMode::Iq;
        case Hold: return v == ViewMode::Spectrum;
        case Log: return v == ViewMode::Waterfall;
        case Fade: case Zoom: return v == ViewMode::Iq;
        default: return false;
    }
}
constexpr bool inRow1(Param p) { return p <= Bw; }

constexpr unsigned kSteps[] = {1, 2, 5, 10, 20};
constexpr unsigned kRateMsps[SDR_RATE_COUNT] = {16, 40, 80};
constexpr unsigned kBandwidths[] = {0, 13, 16, 21, 25, 33, 45, 51, 69};   // 0 = the PHY's own; the rest esp-sdr's S3 curve
constexpr unsigned kAverages[] = {1, 2, 4, 8, 16};
constexpr int kRefMin = -80, kRefMax = 0, kRefStep = 10;
constexpr unsigned kSpans[] = {40, 60, 80, 100};
/// What a pixel keeps of its glow per new burst, /256 (FADE 0 = no trail).
constexpr unsigned kFades[] = {0, 128, 192, 224, 240, 248};
constexpr unsigned kZooms[] = {1, 2, 4, 8, 16};

template <typename T, size_t N> constexpr int count(const T (&)[N]) { return static_cast<int>(N); }

struct State {
    sdr_settings_t rx{2437, SDR_RATE_40M, -1, 0, 4};
    int step = 2;      // kSteps index
    int bw = 0;        // kBandwidths index
    int avg = 2;       // kAverages index
    bool hold = false;
    int ref = -20;     // top of the chart, dBFS
    int span = 2;      // kSpans index
    ViewMode view = ViewMode::Spectrum;
    int fade = 3;      // kFades index
    int zoom = 1;      // kZooms index
    bool log = false;  // each spectrum's peak on the console (sdr_radio_set_peak_log)
};

State s_st;
int s_sel = Freq;
bool s_editing = true;
uint32_t s_seenSeq = 0;
uint32_t s_seenIqSeq = 0;
int16_t s_bins[SDR_BINS];
int16_t s_cols[kMaxCols];   // s_bins on the panel's columns, each its bins' maximum
int16_t s_iq[2 * SDR_IQ_POINTS];
sdr_status_t s_status = {};

lv_obj_t* s_root = nullptr;
lv_obj_t* s_input = nullptr;
lv_obj_t* s_row[2] = {};
lv_obj_t* s_chip[kParams] = {};
lv_obj_t* s_chipLabel[kParams] = {};
lv_obj_t* s_chart = nullptr;
lv_chart_series_t* s_live = nullptr;
lv_chart_series_t* s_hold = nullptr;
lv_obj_t* s_canvas = nullptr;
lv_obj_t* s_wfCanvas = nullptr;
lv_obj_t* s_axis[3] = {};
lv_obj_t* s_info = nullptr;
lv_obj_t* s_note = nullptr;
lv_group_t* s_group = nullptr;
lv_timer_t* s_timer = nullptr;
uint32_t s_backToken = 0;
uint32_t s_points = 0;
bool s_leaving = false;

/* The constellation: a glow per pixel that fades a step with every burst,
 * mapped through a phosphor palette into the canvas. Both live for the
 * process -- this firmware runs nothing else -- and are reused by a later
 * start of the app. */
lv_draw_buf_t* s_iqBuf = nullptr;
uint8_t* s_glow = nullptr;
int32_t s_iqSide = 0;
uint16_t s_palette[256];

/* The waterfall: one spectrum a row, the newest on top, older ones pushed
 * down. Lives for the process like the constellation. */
lv_draw_buf_t* s_wfBuf = nullptr;
uint16_t s_wfPalette[256];

lv_indev_t* navEncoder() { return crosspad_gui::getGuiPlatform().getNavigationEncoder(); }

int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

bool visible(int p) {
    return inRow1(static_cast<Param>(p)) || inRow2(static_cast<Param>(p), s_st.view);
}

void chipText(int p, char* out, size_t n) {
    switch (p) {
        case Freq: std::snprintf(out, n, "%u M", s_st.rx.freq_mhz); break;
        case Step: std::snprintf(out, n, "STEP %u", kSteps[s_st.step]); break;
        case Rate: std::snprintf(out, n, "%u MS/s", kRateMsps[s_st.rx.rate]); break;
        case Gain:
            if (s_st.rx.gain < 0) std::snprintf(out, n, "AGC");
            else std::snprintf(out, n, "G %d", s_st.rx.gain);
            break;
        case Bw:
            if (!kBandwidths[s_st.bw]) std::snprintf(out, n, "BW cal");
            else std::snprintf(out, n, "BW %u", kBandwidths[s_st.bw]);
            break;
        case View: {
            static constexpr const char* kNames[] = {"SPEC", "WF", "I/Q"};
            std::snprintf(out, n, "%s", kNames[static_cast<int>(s_st.view)]);
            break;
        }
        case Avg: std::snprintf(out, n, "AVG %u", kAverages[s_st.avg]); break;
        case Hold: std::snprintf(out, n, "HOLD %s", s_st.hold ? LV_SYMBOL_OK : "-"); break;
        case Ref: std::snprintf(out, n, "REF %d", s_st.ref); break;
        case Span: std::snprintf(out, n, "%u dB", kSpans[s_st.span]); break;
        case Log: std::snprintf(out, n, "LOG %s", s_st.log ? LV_SYMBOL_OK : "-"); break;
        case Fade: std::snprintf(out, n, "FADE %d", s_st.fade); break;
        case Zoom: std::snprintf(out, n, "ZOOM x%u", kZooms[s_st.zoom]); break;
        default: std::snprintf(out, n, "QUIT"); break;
    }
}

void paintChips() {
    char buf[16];
    for (int p = 0; p < kParams; ++p) {
        if (!visible(p)) {
            lv_obj_add_flag(s_chip[p], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_remove_flag(s_chip[p], LV_OBJ_FLAG_HIDDEN);
        chipText(p, buf, sizeof(buf));
        lv_label_set_text(s_chipLabel[p], buf);
        const bool sel = p == s_sel;
        const bool edit = sel && s_editing;
        lv_obj_set_style_bg_color(s_chip[p], lv_color_hex(edit ? kColSelect : kColChip), 0);
        lv_obj_set_style_border_width(s_chip[p], sel ? 2 : 0, 0);
        lv_obj_set_style_text_color(s_chipLabel[p], lv_color_hex(edit ? 0x000000 : kColText), 0);
    }
}

void clearHold() {
    if (s_hold) lv_chart_set_all_values(s_chart, s_hold, LV_CHART_POINT_NONE);
}

void clearGlow() {
    if (s_glow) std::memset(s_glow, 0, static_cast<size_t>(s_iqSide) * s_iqSide);
}

void clearWaterfall() {
    if (!s_wfBuf) return;
    lv_draw_buf_clear(s_wfBuf, nullptr);
    if (s_wfCanvas) lv_obj_invalidate(s_wfCanvas);
}

void applyChartRange() {
    const int bottom = s_st.ref - static_cast<int>(kSpans[s_st.span]);
    lv_chart_set_axis_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, bottom * 10, s_st.ref * 10);
}

void showIf(lv_obj_t* o, bool show) {
    if (show) lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}

/// The spectrum, the waterfall or the constellation, and the footer that
/// goes with it; a view comes back empty rather than with a stale history.
void applyView() {
    const ViewMode v = s_st.view;
    showIf(s_chart, v == ViewMode::Spectrum);
    showIf(s_wfCanvas, v == ViewMode::Waterfall);
    showIf(s_canvas, v == ViewMode::Iq);
    for (auto* a : s_axis) showIf(a, v != ViewMode::Iq);
    if (v == ViewMode::Waterfall) clearWaterfall();
    if (v == ViewMode::Iq) clearGlow();
}

/// The receiver's settings changed: the worker restarts its average, the
/// hold and the trail no longer describe what is on screen.
void applyRx() {
    s_st.rx.bandwidth_mhz = static_cast<uint8_t>(kBandwidths[s_st.bw]);
    s_st.rx.average = static_cast<uint8_t>(kAverages[s_st.avg]);
    sdr_radio_set(&s_st.rx);
    clearHold();
    clearGlow();
    clearWaterfall();
}

void change(int dir) {
    switch (s_sel) {
        case Freq:
            s_st.rx.freq_mhz = static_cast<uint16_t>(
                clampi(s_st.rx.freq_mhz + dir * static_cast<int>(kSteps[s_st.step]), SDR_FREQ_MIN, SDR_FREQ_MAX));
            applyRx();
            break;
        case Step: s_st.step = clampi(s_st.step + dir, 0, count(kSteps) - 1); break;
        case Rate:
            s_st.rx.rate = static_cast<uint8_t>(clampi(s_st.rx.rate + dir, 0, SDR_RATE_COUNT - 1));
            applyRx();
            break;
        case Gain: {
            const int maxGain = static_cast<int>(sdr_radio_gain_max());
            s_st.rx.gain = static_cast<int8_t>(clampi(s_st.rx.gain + dir, -1, maxGain));
            applyRx();
            break;
        }
        case Bw:
            s_st.bw = clampi(s_st.bw + dir, 0, count(kBandwidths) - 1);
            applyRx();
            break;
        case View: {
            constexpr int n = static_cast<int>(ViewMode::kCount);
            s_st.view = static_cast<ViewMode>((static_cast<int>(s_st.view) + dir + n) % n);
            applyView();
            break;
        }
        case Avg:
            s_st.avg = clampi(s_st.avg + dir, 0, count(kAverages) - 1);
            applyRx();
            break;
        case Hold:
            s_st.hold = !s_st.hold;
            clearHold();
            break;
        case Ref:
            s_st.ref = clampi(s_st.ref + dir * kRefStep, kRefMin, kRefMax);
            applyChartRange();
            break;
        case Span:
            s_st.span = clampi(s_st.span + dir, 0, count(kSpans) - 1);
            applyChartRange();
            break;
        case Log:
            s_st.log = !s_st.log;
            sdr_radio_set_peak_log(s_st.log);
            break;
        case Fade: s_st.fade = clampi(s_st.fade + dir, 0, count(kFades) - 1); break;
        case Zoom:
            s_st.zoom = clampi(s_st.zoom + dir, 0, count(kZooms) - 1);
            clearGlow();
            break;
        default: break;
    }
}

/// The next visible chip after @p from in direction @p dir.
int nextVisible(int from, int dir) {
    int p = from;
    do {
        p = (p + dir + kParams) % kParams;
    } while (!visible(p));
    return p;
}

/// Back to the other firmware slot, once the radio is handed back: the switch
/// needs the internal RAM the Wi-Fi driver holds.
void leaveFirmware() {
    if (s_leaving) return;
    s_leaving = true;
    lv_label_set_text(s_note, "Leaving...");
    sdr_radio_stop();
}

void leaveWhenStopped() {
    if (!s_leaving || !sdr_radio_stopped()) return;
    s_leaving = false;
    std::printf("[SDR] Quit: back to the other firmware\n");
    if (crosspad_gui::fwViewService().flowSwitchAndRestart(false)) return;
    lv_label_set_text(s_note, "There is no other firmware to go back to.");
}

bool onPowerBack(void*) {
    if (s_editing) s_editing = false;
    else s_sel = Quit;
    paintChips();
    return true;
}

void onEncoderKey(lv_event_t* e) {
    const uint32_t k = lv_event_get_key(e);
    if (k != LV_KEY_LEFT && k != LV_KEY_RIGHT && k != LV_KEY_ENTER) return;
    lv_event_stop_processing(e);
    if (k == LV_KEY_ENTER) {
        if (s_sel == Quit) {
            leaveFirmware();
            return;
        }
        s_editing = !s_editing;
    } else {
        const int dir = k == LV_KEY_RIGHT ? 1 : -1;
        if (s_editing) change(dir);
        else s_sel = nextVisible(s_sel, dir);
    }
    paintChips();
}

void paintAxis() {
    const float half = static_cast<float>(s_status.rate_hz) / 2e6f;
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1f", s_status.freq_mhz - half);
    lv_label_set_text(s_axis[0], buf);
    std::snprintf(buf, sizeof(buf), "%u", s_status.freq_mhz);
    lv_label_set_text(s_axis[1], buf);
    std::snprintf(buf, sizeof(buf), "%.1f", s_status.freq_mhz + half);
    lv_label_set_text(s_axis[2], buf);
}

/// s_bins onto the panel's columns, keeping each column's strongest bin so a
/// narrow carrier survives the squeeze.
void toColumns() {
    for (uint32_t c = 0; c < s_points; ++c) {
        const uint32_t b0 = c * SDR_BINS / s_points;
        const uint32_t b1 = (c + 1) * SDR_BINS / s_points;
        int16_t m = s_bins[b0];
        for (uint32_t b = b0 + 1; b < b1; ++b)
            if (s_bins[b] > m) m = s_bins[b];
        s_cols[c] = m;
    }
}

void drawChart() {
    int32_t* live = lv_chart_get_series_y_array(s_chart, s_live);
    int32_t* hold = lv_chart_get_series_y_array(s_chart, s_hold);
    for (uint32_t c = 0; c < s_points; ++c) {
        live[c] = s_cols[c];
        if (s_st.hold && (hold[c] == LV_CHART_POINT_NONE || s_cols[c] > hold[c])) hold[c] = s_cols[c];
    }
    lv_chart_refresh(s_chart);
}

/// The rows move down one, the newest spectrum is the top row, coloured over
/// the same REF and dB range the chart uses.
void drawWaterfall() {
    if (!s_wfBuf) return;
    const uint32_t stride = s_wfBuf->header.stride;
    const uint32_t rows = s_wfBuf->header.h;
    std::memmove(s_wfBuf->data + stride, s_wfBuf->data, stride * (rows - 1));
    const int32_t bottom = (s_st.ref - static_cast<int>(kSpans[s_st.span])) * 10;
    const int32_t span = static_cast<int32_t>(kSpans[s_st.span]) * 10;
    auto* row = reinterpret_cast<uint16_t*>(s_wfBuf->data);
    for (uint32_t c = 0; c < s_points; ++c) {
        const int32_t level = (s_cols[c] - bottom) * 255 / span;
        row[c] = s_wfPalette[level < 0 ? 0 : level > 255 ? 255 : level];
    }
    lv_obj_invalidate(s_wfCanvas);
}

/// The strongest bin away from the LO, and the receiver's state, in the footer.
void paintPeak() {
    int peak = 0;
    for (int b = 0; b < static_cast<int>(SDR_BINS); ++b) {
        if (b >= static_cast<int>(SDR_BINS) / 2 - kDcGuardBins && b <= static_cast<int>(SDR_BINS) / 2 + kDcGuardBins)
            continue;
        if (s_bins[b] > s_bins[peak]) peak = b;
    }
    const double binMhz = static_cast<double>(s_status.rate_hz) / 1e6 / SDR_BINS;
    const double peakMhz = s_status.freq_mhz + (peak - static_cast<int>(SDR_BINS) / 2) * binMhz;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "PK %.2f MHz %d dBFS  g%u  %u/s", peakMhz, s_bins[peak] / 10,
                  s_status.gain_index, s_status.spectra_per_s);
    lv_label_set_text(s_info, buf);
    paintAxis();
}

/// One burst into the glow, the glow through the palette into the canvas.
void drawConstellation() {
    const int32_t side = s_iqSide;
    const unsigned keep = kFades[s_st.fade];
    for (int32_t i = 0; i < side * side; ++i) s_glow[i] = static_cast<uint8_t>((s_glow[i] * keep) >> 8);

    const float scale = static_cast<float>(kZooms[s_st.zoom]) * (side / 2) / kIqFullScale;
    double power = 0.0;
    for (uint32_t k = 0; k < SDR_IQ_POINTS; ++k) {
        const float i = s_iq[2 * k], q = s_iq[2 * k + 1];
        power += static_cast<double>(i) * i + static_cast<double>(q) * q;
        const int32_t x = side / 2 + static_cast<int32_t>(std::lround(i * scale));
        const int32_t y = side / 2 - static_cast<int32_t>(std::lround(q * scale));
        if (x < 0 || y < 0 || x >= side || y >= side) continue;
        uint8_t& g = s_glow[y * side + x];
        g = static_cast<uint8_t>(g > 255 - kDotHit ? 255 : g + kDotHit);
    }

    const uint16_t grid = lv_color_to_u16(lv_color_hex(kColGrid));
    const uint32_t stride = s_iqBuf->header.stride;
    for (int32_t y = 0; y < side; ++y) {
        auto* row = reinterpret_cast<uint16_t*>(s_iqBuf->data + y * stride);
        const uint8_t* g = s_glow + y * side;
        for (int32_t x = 0; x < side; ++x) {
            const bool axis = x == side / 2 || y == side / 2;
            row[x] = g[x] ? s_palette[g[x]] : axis ? grid : 0;
        }
    }
    lv_obj_invalidate(s_canvas);

    const double rms = std::sqrt(power / SDR_IQ_POINTS) / kIqFullScale;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "I/Q %u M  rms %d dBFS  g%u  %u/s", s_status.freq_mhz,
                  rms > 0.0 ? static_cast<int>(std::lround(20.0 * std::log10(rms))) : -99, s_status.gain_index,
                  s_status.spectra_per_s);
    lv_label_set_text(s_info, buf);
}

void tick(lv_timer_t*) {
    leaveWhenStopped();
    const bool fresh = sdr_radio_read(s_seenSeq, s_bins, &s_status);
    if (fresh) s_seenSeq = s_status.seq;
    if (s_st.view != ViewMode::Iq) {
        if (!fresh) return;
        toColumns();
        if (s_st.view == ViewMode::Spectrum) drawChart();
        else drawWaterfall();
        paintPeak();
    } else if (s_iqBuf && s_glow && sdr_radio_read_iq(&s_seenIqSeq, s_iq)) {
        drawConstellation();
    }
}

/// Black through green to a yellow-white core, as a phosphor trace glows.
void buildPalette() {
    for (int i = 0; i < 256; ++i) {
        const float t = i / 255.0f;
        const auto ch = [](float v) { return static_cast<uint8_t>(v < 0.0f ? 0 : v > 1.0f ? 255 : v * 255.0f); };
        s_palette[i] = lv_color_to_u16(lv_color_make(ch((t - 0.55f) / 0.45f), ch(t * 1.4f), ch((t - 0.8f) / 0.2f)));
    }
    s_palette[0] = 0;
}

/// The usual waterfall heat scale: black, blue, cyan, yellow, red, white.
void buildWaterfallPalette() {
    struct Stop { float t; uint8_t r, g, b; };
    static constexpr Stop kStops[] = {{0.00f, 0, 0, 0},       {0.25f, 0, 0, 160},   {0.45f, 0, 170, 220},
                                      {0.65f, 240, 230, 0},   {0.85f, 255, 40, 0},  {1.00f, 255, 255, 255}};
    for (int i = 0; i < 256; ++i) {
        const float t = i / 255.0f;
        size_t k = 1;
        while (k < sizeof(kStops) / sizeof(kStops[0]) - 1 && t > kStops[k].t) ++k;
        const Stop& a = kStops[k - 1];
        const Stop& b = kStops[k];
        const float f = (t - a.t) / (b.t - a.t);
        const auto mix = [f](uint8_t x, uint8_t y) { return static_cast<uint8_t>(x + (y - x) * f); };
        s_wfPalette[i] = lv_color_to_u16(lv_color_make(mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b)));
    }
}

/// The waterfall canvas in the chart's place. Kept once made.
void makeWaterfall(lv_obj_t* parent, int32_t w, int32_t h) {
    if (!s_wfBuf) {
        s_wfBuf = lv_draw_buf_create(w, h, LV_COLOR_FORMAT_RGB565, 0);
        buildWaterfallPalette();
    }
    s_wfCanvas = lv_canvas_create(parent);
    lv_obj_set_pos(s_wfCanvas, 0, kChartY);
    if (s_wfBuf) lv_canvas_set_draw_buf(s_wfCanvas, s_wfBuf);
    clearWaterfall();
}

lv_obj_t* makeChip(lv_obj_t* parent, int p) {
    lv_obj_t* chip = lv_obj_create(parent);
    lv_obj_remove_style_all(chip);
    lv_obj_set_size(chip, LV_SIZE_CONTENT, kChipH);
    lv_obj_set_style_pad_hor(chip, kChipPad, 0);
    lv_obj_set_style_bg_opa(chip, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(chip, 4, 0);
    lv_obj_set_style_border_color(chip, lv_color_hex(kColSelect), 0);
    lv_obj_remove_flag(chip, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
    lv_obj_t* label = lv_label_create(chip);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    s_chipLabel[p] = label;
    return chip;
}

lv_obj_t* makeRow(lv_obj_t* parent, int32_t y, int32_t w) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_pos(row, 0, y);
    lv_obj_set_size(row, w, kChipH);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(row, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
    return row;
}

lv_obj_t* makeLabel(lv_obj_t* parent, const lv_font_t* font, uint32_t color) {
    lv_obj_t* l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_label_set_text(l, "");
    return l;
}

/// The canvas and its glow, square in the chart's place. Kept once made.
void makeConstellation(lv_obj_t* parent, int32_t w, int32_t h) {
    const int32_t side = h < w ? h : w;
    if (!s_iqBuf) {
        s_iqBuf = lv_draw_buf_create(side, side, LV_COLOR_FORMAT_RGB565, 0);
        s_glow = static_cast<uint8_t*>(lv_malloc_zeroed(static_cast<size_t>(side) * side));
        s_iqSide = side;
        buildPalette();
    }
    s_canvas = lv_canvas_create(parent);
    lv_obj_set_pos(s_canvas, (w - side) / 2, kChartY);
    if (s_iqBuf) {
        lv_draw_buf_clear(s_iqBuf, nullptr);
        lv_canvas_set_draw_buf(s_canvas, s_iqBuf);
    }
    clearGlow();
}

lv_obj_t* lv_CreateSdr(lv_obj_t* parent, App* app) {
    (void)app;
    s_root = lv_obj_create(parent);
    lv_obj_remove_style_all(s_root);
    lv_obj_set_size(s_root, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(s_root, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_update_layout(parent);
    const int32_t w = lv_obj_get_content_width(parent);
    const int32_t h = lv_obj_get_content_height(parent);

    s_row[0] = makeRow(s_root, 1, w);
    s_row[1] = makeRow(s_root, 1 + kChipH + kChipGap, w);
    for (int p = 0; p < kParams; ++p) s_chip[p] = makeChip(s_row[inRow1(static_cast<Param>(p)) ? 0 : 1], p);

    const int32_t viewH = h - kChartY - kFooterH;
    s_chart = lv_chart_create(s_root);
    lv_obj_remove_style_all(s_chart);
    lv_obj_set_pos(s_chart, 0, kChartY);
    lv_obj_set_size(s_chart, w, viewH);
    lv_obj_set_style_line_color(s_chart, lv_color_hex(kColGrid), LV_PART_MAIN);
    lv_obj_set_style_line_width(s_chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_width(s_chart, 1, LV_PART_ITEMS);
    lv_obj_set_style_size(s_chart, 0, 0, LV_PART_INDICATOR);
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(s_chart, 5, 9);
    s_points = static_cast<uint32_t>(w) < kMaxCols ? static_cast<uint32_t>(w) : kMaxCols;
    lv_chart_set_point_count(s_chart, s_points);
    s_hold = lv_chart_add_series(s_chart, lv_color_hex(kColHold), LV_CHART_AXIS_PRIMARY_Y);
    s_live = lv_chart_add_series(s_chart, lv_color_hex(kColLive), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_values(s_chart, s_live, LV_CHART_POINT_NONE);
    clearHold();
    applyChartRange();

    makeConstellation(s_root, w, viewH);
    makeWaterfall(s_root, static_cast<int32_t>(s_points), viewH);

    const int32_t axisY = h - kFooterH;
    for (int i = 0; i < 3; ++i) s_axis[i] = makeLabel(s_root, &lv_font_montserrat_10, kColText);
    lv_obj_align(s_axis[0], LV_ALIGN_TOP_LEFT, 2, axisY);
    lv_obj_align(s_axis[1], LV_ALIGN_TOP_MID, 0, axisY);
    lv_obj_align(s_axis[2], LV_ALIGN_TOP_RIGHT, -2, axisY);
    s_info = makeLabel(s_root, &lv_font_montserrat_14, kColText);
    lv_obj_align(s_info, LV_ALIGN_BOTTOM_LEFT, 2, -1);
    lv_label_set_text(s_info, "Starting the receiver...");
    s_note = makeLabel(s_root, &lv_font_montserrat_14, kColHold);
    lv_obj_align(s_note, LV_ALIGN_CENTER, 0, 0);

    /* The encoder's group: one object always in edit mode, so a turn is a
     * key here rather than a move of focus. */
    s_input = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_input);
    lv_obj_set_size(s_input, 1, 1);
    s_group = lv_group_create();
    lv_group_add_obj(s_group, s_input);
    lv_obj_add_event_cb(s_input, onEncoderKey, static_cast<lv_event_code_t>(LV_EVENT_KEY | LV_EVENT_PREPROCESS),
                        nullptr);
    if (lv_indev_t* nav = navEncoder()) lv_indev_set_group(nav, s_group);
    lv_group_focus_obj(s_input);
    lv_group_set_editing(s_group, true);

    s_backToken = crosspad_gui::ui_back_push(onPowerBack, nullptr);

    s_st.rx.bandwidth_mhz = static_cast<uint8_t>(kBandwidths[s_st.bw]);
    s_st.rx.average = static_cast<uint8_t>(kAverages[s_st.avg]);
    if (!sdr_radio_start(&s_st.rx)) lv_label_set_text(s_note, "The receiver did not start.");
    applyView();
    paintChips();
    s_timer = lv_timer_create(tick, kTickMs, nullptr);
    return s_root;
}

void lv_DestroySdr(lv_obj_t* root) {
    if (s_timer) {
        lv_timer_delete(s_timer);
        s_timer = nullptr;
    }
    crosspad_gui::ui_back_remove(s_backToken);
    s_backToken = 0;
    if (s_group) {   // the framework restores the encoder's previous group
        lv_group_delete_deferred(s_group);
        s_group = nullptr;
    }
    s_root = s_input = s_chart = s_canvas = s_wfCanvas = s_info = s_note = nullptr;
    s_live = s_hold = nullptr;
    for (auto& o : s_row) o = nullptr;
    for (auto& o : s_chip) o = nullptr;
    for (auto& o : s_chipLabel) o = nullptr;
    for (auto& o : s_axis) o = nullptr;
    if (root) lv_obj_delete_async(root);
}

} // namespace

REGISTER_APP(Sdr, nullptr, crosspad_gui::resolveAppIcon("sdr.png"), lv_CreateSdr, lv_DestroySdr, nullptr, nullptr,
             nullptr, 0)

#endif // USE_LVGL
