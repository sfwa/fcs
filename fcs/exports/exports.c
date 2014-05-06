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

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>

#ifdef __TI_COMPILER_VERSION__
#include "../c66x-csl/ti/csl/cslr_device.h"
#include "../c66x-csl/ti/csl/cslr_sem.h"
#include "../c66x-csl/ti/csl/cslr_gpio.h"
#endif

#include "exports.h"
#include "../util/util.h"

#ifdef __TI_COMPILER_VERSION__
#define FCS_SEMAPHORE_ID_EXPORTS 1u
#define FCS_SEMAPHORE_TIMEOUT 2000u

#define L1DWIBAR (*((volatile uint32_t*)0x01844030))
#define L1DWIWC (*((volatile uint32_t*)0x01844034))
#define L1DIBAR (*((volatile uint32_t*)0x01844048))
#define L1DIWC (*((volatile uint32_t*)0x0184404C))
#define L1DWBAR (*((volatile uint32_t*)0x01844040))
#define L1DWWC (*((volatile uint32_t*)0x01844044))
#endif

#pragma DATA_SECTION(exports_shared_logs, ".shared")
volatile static struct fcs_log_t exports_shared_logs[FCS_LOG_TYPE_LAST - 1u];

static struct fcs_log_t exports_local_logs[FCS_LOG_TYPE_LAST - 1u];
static enum fcs_log_open_mode_t exports_local_states[FCS_LOG_TYPE_LAST - 1u] =
    { FCS_MODE_CLOSED, FCS_MODE_CLOSED, FCS_MODE_CLOSED, FCS_MODE_CLOSED,
      FCS_MODE_CLOSED };


void fcs_exports_init(void) {
    size_t i;
    struct fcs_log_t l;

    for (i = 1; i < FCS_LOG_TYPE_LAST - 1u; i++) {
        exports_local_states[i] = FCS_MODE_CLOSED;
        fcs_log_init(&l, (enum fcs_log_type_t)i, 0);
        exports_shared_logs[i] = l;
    }

#ifdef __TI_COMPILER_VERSION__
    /* Release the state and control semaphores */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    semaphore->SEM[FCS_SEMAPHORE_ID_EXPORTS] = 1u;
#endif
}

struct fcs_log_t *fcs_exports_log_open(enum fcs_log_type_t type,
enum fcs_log_open_mode_t mode) {
    /* printf("fcs_exports_log_open(%d, %c)\n", (int)type, (char)mode); */

    fcs_assert(FCS_LOG_TYPE_INVALID < type && type < FCS_LOG_TYPE_LAST);
    fcs_assert(mode == FCS_MODE_READ || mode == FCS_MODE_WRITE ||
               mode == FCS_MODE_APPEND);
    fcs_assert(exports_local_states[type] == FCS_MODE_CLOSED);

#ifdef __TI_COMPILER_VERSION__
    /* Wait until we can acquire the semaphore */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = 0, i = 0;
    while (sem_val != 1u && i < FCS_SEMAPHORE_TIMEOUT) {
        sem_val = semaphore->SEM[FCS_SEMAPHORE_ID_EXPORTS];
        i++;
    }
    if (sem_val != 1u) {
        return NULL;
    }
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Invalidate L1 */
    L1DIBAR = (uint32_t)&exports_shared_logs[type];
    L1DIWC = sizeof(struct fcs_log_t) / 4u;
    while (L1DIWC & 0xFFFFu);
#endif

    exports_local_logs[type] = exports_shared_logs[type];
    if (mode == FCS_MODE_WRITE) {
        /* Set up a new log -- track frame ID */
        fcs_log_init(&exports_local_logs[type], type,
                     fcs_log_get_frame_id(&exports_local_logs[type]) + 1u);
    } else if (exports_shared_logs[type].length < FCS_LOG_MIN_LENGTH) {
        /* Invalid source log; just hand back an empty log */
        fcs_log_init(&exports_local_logs[type], type, 0);
    }

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_ID_EXPORTS] = 1u;
#endif

    exports_local_states[type] = mode;

    return &exports_local_logs[type];
}

struct fcs_log_t *fcs_exports_log_close(struct fcs_log_t *l) {
    fcs_assert(l);

    enum fcs_log_type_t type;
    size_t idx;
    for (idx = 1; idx < FCS_LOG_TYPE_LAST; idx++) {
        if (l == &exports_local_logs[(enum fcs_log_type_t)idx]) {
            break;
        }
    }

    /* printf("fcs_exports_log_close(%u)\n", (int)idx); */

    fcs_assert(0 < idx && idx < FCS_LOG_TYPE_LAST);
    type = (enum fcs_log_type_t)idx;

    fcs_assert(exports_local_states[type] == FCS_MODE_READ ||
               exports_local_states[type] == FCS_MODE_WRITE ||
               exports_local_states[type] == FCS_MODE_APPEND);
    fcs_assert(l->length >= FCS_LOG_MIN_LENGTH &&
               l->length <= FCS_LOG_MAX_LENGTH);

    if (exports_local_states[type] == FCS_MODE_WRITE ||
            exports_local_states[type] == FCS_MODE_APPEND) {
#ifdef __TI_COMPILER_VERSION__
        /* Acquire the global state semaphore; exit if that's not possible */
        volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
        uint32_t sem_val = 0, i = 0;
        while (sem_val != 1u && i < FCS_SEMAPHORE_TIMEOUT) {
            sem_val = semaphore->SEM[FCS_SEMAPHORE_ID_EXPORTS];
            i++;
        }
        if (sem_val != 1u) {
            return NULL;
        }
#endif

        exports_shared_logs[type] = *l;

#ifdef __TI_COMPILER_VERSION__
        /* Write back L1 */
        L1DWBAR = (uint32_t)&exports_shared_logs[type];
        L1DWWC = sizeof(struct fcs_log_t) / 4u;
        while (L1DWWC & 0xFFFFu);
#endif

#ifdef __TI_COMPILER_VERSION__
        /* Release the semaphore */
        semaphore->SEM[FCS_SEMAPHORE_ID_EXPORTS] = 1u;
#endif
    }

    exports_local_states[idx] = FCS_MODE_CLOSED;

    return NULL;
}
