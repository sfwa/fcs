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

#ifndef _FCS_CONFIG_H
#define _FCS_CONFIG_H

#define FCS_CORE_AHRS 0
#define FCS_CORE_NMPC 1u
#define FCS_CORE_UTIL 0
#define FCS_CORE_COMMS 0
#define FCS_CORE_CONFIG 0
#define FCS_CORE_PLATFORM 0

#define FCS_CLOCK_HZ 1000000000u
#define FCS_CORE0_TICK_HZ 1000u
#define FCS_CORE1_TICK_HZ 50u

/* Semaphore 0 always seems to be busy, so don't use it */
#define FCS_SEMAPHORE_CORE0_BOOT 1u
#define FCS_SEMAPHORE_CORE1_BOOT 2u
#define FCS_SEMAPHORE_GLOBAL_STATE 3u
#define FCS_SEMAPHORE_GLOBAL_WAYPOINTS 4u
#define FCS_SEMAPHORE_GLOBAL_CONTROL 5u

#endif
