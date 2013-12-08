/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _STATS_H_
#define _STATS_H_

struct fcs_stats_counter_t {
    /* Comms/AHRS interface packet counters */
    uint64_t ioboard_packet_rx[2];
    uint64_t ioboard_packet_rx_err[2];
    uint64_t ioboard_packet_tx[2];
    uint64_t ioboard_reset[2];

    uint64_t cpu_packet_rx;
    uint64_t cpu_packet_rx_err;
    uint64_t cpu_packet_tx;

    /* Stream byte, reset and error counters */
    uint64_t stream_rx_byte[5];
    uint64_t stream_rx_err[5];
    uint64_t stream_rx_overrun[5];
    uint64_t stream_tx_byte[5];
    uint64_t stream_tx_overrun[5];
    uint64_t stream_reset[5];

    /* Maximum per-iteration cycle counter */
    uint32_t main_loop_count;
    uint32_t main_loop_cycle_max;
};

extern struct fcs_stats_counter_t fcs_global_counters;

#endif
