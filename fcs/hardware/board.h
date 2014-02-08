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

#ifndef _FCS_BOARD_H
#define _FCS_BOARD_H

/* Convert an address in an individual core's L2 SRAM to global */
#define GLOBAL_FROM_L2_ADDRESS(n) ((1u << 28) | \
    (DNUM << 24) | ((uint32_t)(n) & 0x00FFFFFFu))

#define GLOBAL_FROM_CORE_L2_ADDRESS(n, core) ((1u << 28) | \
    (core << 24) | ((uint32_t)(n) & 0x00FFFFFFu))

/*
GPIO pin map

Pin    Direction    Net                    Description
0      INPUT        /DSP_EXT_UART0_INT     EMIF16 UART 0 byte ready
1      INPUT        /DSP_EXT_UART1_INT     EMIF16 UART 1 byte ready
2      OUTPUT       IOBOARD_1_RESET_OUT    I/O board 0 reset (ahrs.c)
3      OUTPUT       IOBOARD_2_RESET_OUT    I/O board 1 reset (ahrs.c)
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21                  N/C
22     OUTPUT       D11_GREEN              UART 0 TX indicator (int-uart.c)
23     OUTPUT       D12_RED                UART 0 RX indicator (int-uart.c)
24                  N/C
25                  N/C
26     OUTPUT       D13_GREEN              UART 1 TX indicator (int-uart.c)
27     OUTPUT       D14_RED                UART 1 RX indicator (int-uart.c)

*/

void fcs_board_init_platform(void);
uint32_t fcs_board_init_core(void);

void fcs_board_tick(void);

#endif
