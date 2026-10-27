// SPDX-License-Identifier: GPL-3.0-or-later
/* CDC verbs of the sdr kit, reached through main's weak sdr_verbs_handle hook.
 *   SDR_DUMP <count> <rate 0=16|1=40|2=80 MS/s> <MHz> <gain|-1> [bw MHz] [stride us]
 *   SDR_DUMP?   state of the last dump
 *   SDR_LORA <count> <MHz> <gain|-1> [stride us]   on-device ELRS SF6 demod (16 MS/s)
 *   SDR_LORA?   state of the last demod
 *   SDR_LORA_GET   the demod text (one line per burst) */

#include <cstdio>
#include <cstring>

#include "sdr_radio.h"

extern "C" bool sdr_verbs_handle(const char* cmd, void (*reply)(const char* text)) {
    if (std::strcmp(cmd, "SDR_DUMP?") == 0) {
        char line[160];
        sdr_radio_dump_status(line, sizeof line);
        reply(line);
        return true;
    }
    if (std::strcmp(cmd, "SDR_LORA?") == 0) {
        char line[160];
        sdr_radio_lora_status(line, sizeof line);
        reply(line);
        return true;
    }
    if (std::strcmp(cmd, "SDR_LORA_GET") == 0) {
        size_t len = 0;
        const char* t = sdr_radio_lora_text(&len);
        reply(t && len ? t : "ERR not_done\r\n");
        return true;
    }
    {
        unsigned count, mhz, stride = 0;
        int gain;
        if (std::sscanf(cmd, "SDR_LORA %u %u %d %u", &count, &mhz, &gain, &stride) >= 3) {
            const sdr_dump_request_t r = {count, 0 /*16 MS/s*/, static_cast<uint16_t>(mhz),
                                          static_cast<int8_t>(gain), 0, static_cast<uint16_t>(stride)};
            reply(sdr_radio_lora_start(&r) ? "OK" : "ERR busy_or_args");
            return true;
        }
    }
    unsigned count, rate, mhz, bw = 0, stride = 0;
    int gain;
    if (std::sscanf(cmd, "SDR_DUMP %u %u %u %d %u %u", &count, &rate, &mhz, &gain, &bw, &stride) >= 4) {
        const sdr_dump_request_t r = {count, static_cast<uint8_t>(rate), static_cast<uint16_t>(mhz),
                                      static_cast<int8_t>(gain), static_cast<uint8_t>(bw),
                                      static_cast<uint16_t>(stride)};
        reply(sdr_radio_dump_start(&r) ? "OK" : "ERR busy_or_args");
        return true;
    }
    return false;
}
