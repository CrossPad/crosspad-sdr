# crosspad-sdr

The ESP32-S3's own Wi-Fi radio as a receiver, after
[ESP-SDR](https://github.com/ESPARGOS/esp-sdr): the one app of the `sdr`
firmware kit, started from the Firmware app's library like any other firmware
and left through its own **QUIT**, which switches back to the other firmware
slot.

The radio captures a snapshot of raw I/Q (16, 40 or 80 MS/s, up to 16380
samples into a reserved 64 KiB SRAM bank), a worker turns it into a 1024-bin
power spectrum, and the panel shows it as a spectrum, a waterfall or an I/Q
constellation. Receive only: nothing is transmitted, and there is no Wi-Fi or
Bluetooth while the kit runs (the kit's profile turns the radio users and the
audio engine off, so the capture has the internal RAM it needs).

## Controls

Two rows of chips. The first is the receiver, the second the view's own
settings:

| Chip | | Chip | |
|---|---|---|---|
| `2440 M` | LO, 1842-2800 MHz | `SPEC` / `WF` / `I/Q` | view |
| `STEP` | LO step, 1-20 MHz | `AVG` | spectra averaged, 1-16 (SPEC, WF) |
| `MS/s` | 16, 40 or 80 MS/s | `HOLD` | max hold (SPEC) |
| `AGC` / `G` | the chip's AGC or a fixed PHY gain index | `REF`, `dB` | top of the scale and its span (SPEC, WF) |
| `BW` | analog filter (`cal` = the PHY's own) | `LOG` | waterfall log (WF) |
| | | `FADE`, `ZOOM` | dot trail and scale (I/Q) |

- **Encoder**: turning moves the highlight; a click edits the highlighted
  chip (turning then changes it), a second click stops editing.
- **QUIT**, clicked: the radio stops, then back to the firmware in the other
  slot.
- **Power click**: stops editing, else puts the highlight on QUIT.
- **Pad 16 held while the board boots**: the platform starts the main
  firmware instead, whatever is in the slots. That is the platform's, not
  this app's.

## CDC verbs

Through the firmware's `sdr_verbs_handle` hook, for captures off the board:

| Verb | |
|---|---|
| `SDR_DUMP <count> <rate 0/1/2> <MHz> <gain\|-1> [bw MHz] [stride us]` | `count` raw bursts to `/sdcard/crosspad/sdr_dump.bin`; `stride` waits between bursts |
| `SDR_DUMP?` | state of the last dump |
| `SDR_LORA <count> <MHz> <gain\|-1> [stride us]` | LoRa SF6 / BW 812.5 kHz demod on the board (16 MS/s): per burst the strongest 1 MHz channel, mixed to DC, decimated, dechirped, one symbol per FFT |
| `SDR_LORA?`, `SDR_LORA_GET` | its state, and its text: one line per burst, `B<n> ch<c> <symbols>` (255 = below the SNR gate) |

## Layout

| Path | |
|---|---|
| `src/SdrApp.cpp` | the app: chips, spectrum, waterfall, constellation, QUIT |
| `include/sdr_radio.h` | what the app asks of the receiver: settings in, spectra out |
| `port/idf/sdr_worker.cpp` | the worker task that owns the radio: capture, FFT, dumps, the LoRa demod |
| `port/idf/sdr_capture_s3.c` | ESP-SDR's S3 snapshot capture without its serial protocol |
| `port/idf/sram_guard.ld` | keeps static data below the dump bank |
| `esp-sdr/` | upstream ESP-SDR (submodule): calibration and tuning helpers |
| `CMakeLists.txt` | the ESP-IDF component: built into the `sdr` kit only (`crosspad-app.json` `kits`), empty in every other image |

Clone with `--recursive` (or `git submodule update --init` inside), or the
build finds no `esp-sdr/`.

Only the snapshot path of ESP-SDR is used: its ring mode masks interrupts for
a whole run and takes core 1 bare-metal, which the display, USB and the STM32's
heartbeat cannot live with.

## Licence

GPL-3.0-or-later, as ESP-SDR, which this is built on (`LICENSE`).
