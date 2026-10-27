/* The dump engine owns 0x3fcd0000..0x3fce0000 while it captures. */
ASSERT(_data_end <= 0x3fcd0000, "SDR capture bank overlaps data")
ASSERT(_bss_end <= 0x3fcd0000, "SDR capture bank overlaps BSS")
