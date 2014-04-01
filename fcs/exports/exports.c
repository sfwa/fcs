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
#include <assert.h>
#include <float.h>

#ifdef __TI_COMPILER_VERSION__
#include "../c66x-csl/ti/csl/cslr_device.h"
#include "../c66x-csl/ti/csl/cslr_sem.h"
#include "../c66x-csl/ti/csl/cslr_gpio.h"
#endif

#include "../config/config.h"
#include "../util/util.h"
#include "../util/3dmath.h"
#include "../ukf/cukf.h"
#include "../nmpc/cnmpc.h"
#include "../stats/stats.h"
#include "../TRICAL/TRICAL.h"
#include "../ahrs/measurement.h"
#include "../ahrs/ahrs.h"
#include "../control/control.h"
#include "exports.h"

#ifdef __TI_COMPILER_VERSION__
#define FCS_SEMAPHORE_STATE 1u
#define FCS_SEMAPHORE_CONTROL 2u
#define FCS_SEMAPHORE_PATH 3u
#define FCS_SEMAPHORE_WAYPOINT 4u
#define FCS_SEMAPHORE_TIMEOUT 1000u

#define L1DWIBAR (*((volatile uint32_t*)0x01844030))
#define L1DWIWC (*((volatile uint32_t*)0x01844034))
#define L1DIBAR (*((volatile uint32_t*)0x01844048))
#define L1DIWC (*((volatile uint32_t*)0x0184404C))
#define L1DWBAR (*((volatile uint32_t*)0x01844040))
#define L1DWWC (*((volatile uint32_t*)0x01844044))
#endif


#pragma DATA_SECTION(fcs_export_state, ".shared")
volatile static struct fcs_state_estimate_t fcs_export_state;

#pragma DATA_SECTION(fcs_export_control, ".shared")
volatile static struct fcs_control_output_t fcs_export_control;

#pragma DATA_SECTION(fcs_export_waypoint_update, ".shared");
volatile static struct fcs_waypoint_update_t fcs_export_waypoint_update;

#pragma DATA_SECTION(fcs_export_path_update, ".shared");
volatile static struct fcs_path_update_t fcs_export_path_update;


void fcs_exports_init(void) {
#ifdef __TI_COMPILER_VERSION__
    /* Release the state and control semaphores */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    semaphore->SEM[FCS_SEMAPHORE_STATE] = 1u;
    semaphore->SEM[FCS_SEMAPHORE_CONTROL] = 1u;
#endif
}

void fcs_exports_send_state(void) {
#ifdef __TI_COMPILER_VERSION__
    /* Acquire the global state semaphore; exit if that's not possible */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = semaphore->SEM[FCS_SEMAPHORE_STATE];
    if (sem_val != 1u) {
        return;
    }
#endif

    fcs_export_state.lat = fcs_global_ahrs_state.lat;
    fcs_export_state.lon = fcs_global_ahrs_state.lon;
    fcs_export_state.alt = (float)fcs_global_ahrs_state.alt;
    fcs_export_state.velocity[0] = (float)fcs_global_ahrs_state.velocity[0];
    fcs_export_state.velocity[1] = (float)fcs_global_ahrs_state.velocity[1];
    fcs_export_state.velocity[2] = (float)fcs_global_ahrs_state.velocity[2];
    fcs_export_state.attitude[0] = (float)fcs_global_ahrs_state.attitude[0];
    fcs_export_state.attitude[1] = (float)fcs_global_ahrs_state.attitude[1];
    fcs_export_state.attitude[2] = (float)fcs_global_ahrs_state.attitude[2];
    fcs_export_state.attitude[3] = (float)fcs_global_ahrs_state.attitude[3];
    fcs_export_state.angular_velocity[0] =
        (float)fcs_global_ahrs_state.angular_velocity[0];
    fcs_export_state.angular_velocity[1] =
        (float)fcs_global_ahrs_state.angular_velocity[1];
    fcs_export_state.angular_velocity[2] =
        (float)fcs_global_ahrs_state.angular_velocity[2];
    fcs_export_state.wind_velocity[0] =
        (float)fcs_global_ahrs_state.wind_velocity[0];
    fcs_export_state.wind_velocity[1] =
        (float)fcs_global_ahrs_state.wind_velocity[1];
    fcs_export_state.wind_velocity[2] =
        (float)fcs_global_ahrs_state.wind_velocity[2];
    fcs_export_state.mode = (uint8_t)fcs_global_ahrs_state.mode;

#ifdef __TI_COMPILER_VERSION__
    /* Write back L1 */
    L1DWBAR = (uint32_t)&fcs_export_state;
    L1DWWC = sizeof(fcs_export_state) / 4u;
    while (L1DWWC & 0xFFFFu);
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_STATE] = 1u;
#endif
}

void fcs_exports_send_control(void) {
#ifdef __TI_COMPILER_VERSION__
    /* Wait until we can acquire the semaphore */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = 0, i = 0;
    while (sem_val != 1u && i < FCS_SEMAPHORE_TIMEOUT) {
        sem_val = semaphore->SEM[FCS_SEMAPHORE_CONTROL];
        i++;
    }
#else
    uint32_t i;
#endif

    for (i = 0; i < FCS_CONTROL_CHANNELS; i++) {
        fcs_export_control.values[i] =
            fcs_global_control_state.controls[i].setpoint;
        fcs_export_control.rates[i] =
            fcs_global_control_state.controls[i].rate;
    }
    fcs_export_control.gpio = fcs_global_control_state.gpio_state;
    fcs_export_control.mode = (uint8_t)fcs_global_control_state.mode;

    fcs_export_control.objective_val =
        fcs_global_counters.nmpc_objective_value;
    fcs_export_control.cycles =
    	fcs_global_counters.nmpc_last_cycle_count;
    fcs_export_control.nmpc_errors =
        fcs_global_counters.nmpc_errors;
    fcs_export_control.nmpc_resets =
        fcs_global_counters.nmpc_resets;
    fcs_export_control.nav_state_version = fcs_global_nav_state.version;

    /* Copy the current reference point into the packet as well */
    fcs_export_control.reference_lat =
        fcs_global_nav_state.reference_trajectory[0].lat;
    fcs_export_control.reference_lon =
        fcs_global_nav_state.reference_trajectory[0].lon;
    fcs_export_control.reference_alt =
        fcs_global_nav_state.reference_trajectory[0].alt;
    fcs_export_control.reference_airspeed =
        fcs_global_nav_state.reference_trajectory[0].airspeed;
    fcs_export_control.reference_yaw =
        fcs_global_nav_state.reference_trajectory[0].yaw;
    fcs_export_control.reference_pitch =
        fcs_global_nav_state.reference_trajectory[0].pitch;
    fcs_export_control.reference_roll =
        fcs_global_nav_state.reference_trajectory[0].roll;

    fcs_export_control.path_id = fcs_global_nav_state.reference_path_id[0];

#ifdef __TI_COMPILER_VERSION__
    /* Write back L1 */
    L1DWBAR = (uint32_t)&fcs_export_control;
    L1DWWC = sizeof(fcs_export_control) / 4u;
    while (L1DWWC & 0xFFFFu);
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_CONTROL] = 1u;
#endif
}

void fcs_exports_recv_state(struct fcs_state_estimate_t *state) {
#ifdef __TI_COMPILER_VERSION__
    /* Wait until we can acquire the semaphore */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = 0, i = 0;
    while (sem_val != 1u && i < FCS_SEMAPHORE_TIMEOUT) {
        sem_val = semaphore->SEM[FCS_SEMAPHORE_STATE];
        i++;
    }
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Invalidate L1 */
    L1DIBAR = (uint32_t)&fcs_export_state;
    L1DIWC = sizeof(fcs_export_state) / 4u;
    while (L1DIWC & 0xFFFFu);
#endif

    *state = fcs_export_state;

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_STATE] = 1u;
#endif
}

void fcs_exports_recv_control(struct fcs_control_output_t *control) {
#ifdef __TI_COMPILER_VERSION__
    /* Acquire the global control semaphore; exit if that's not possible */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = semaphore->SEM[FCS_SEMAPHORE_CONTROL];
    if (sem_val != 1u) {
        return;
    }
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Invalidate L1 */
    L1DIBAR = (uint32_t)&fcs_export_control;
    L1DIWC = sizeof(fcs_export_control) / 4u;
    while (L1DIWC & 0xFFFFu);
#endif

    *control = fcs_export_control;

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_CONTROL] = 1u;
#endif
}

void fcs_exports_send_waypoint_update(uint32_t nav_state_version,
uint16_t waypoint_id, const struct fcs_waypoint_t *waypoint) {
#ifdef __TI_COMPILER_VERSION__
    /* Acquire the global control semaphore; exit if that's not possible */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = semaphore->SEM[FCS_SEMAPHORE_WAYPOINT];
    if (sem_val != 1u) {
        return;
    }
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Invalidate L1 */
    L1DIBAR = (uint32_t)&fcs_export_waypoint_update;
    L1DIWC = sizeof(fcs_export_waypoint_update) / 4u;
    while (L1DIWC & 0xFFFFu);
#endif

    fcs_export_waypoint_update.nav_state_version = nav_state_version;
    fcs_export_waypoint_update.waypoint_id = waypoint_id;
    fcs_export_waypoint_update.waypoint = *waypoint;

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_WAYPOINT] = 1u;
#endif
}

void fcs_exports_send_path_update(uint32_t nav_state_version,
uint16_t path_id, const struct fcs_path_t *path) {
#ifdef __TI_COMPILER_VERSION__
    /* Acquire the global control semaphore; exit if that's not possible */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = semaphore->SEM[FCS_SEMAPHORE_PATH];
    if (sem_val != 1u) {
        return;
    }
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Invalidate L1 */
    L1DIBAR = (uint32_t)&fcs_export_path_update;
    L1DIWC = sizeof(fcs_export_path_update) / 4u;
    while (L1DIWC & 0xFFFFu);
#endif

    fcs_export_path_update.nav_state_version = nav_state_version;
    fcs_export_path_update.path_id = path_id;
    fcs_export_path_update.path = *path;

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_PATH] = 1u;
#endif
}

void fcs_exports_recv_waypoint_update(struct fcs_waypoint_update_t *update){
#ifdef __TI_COMPILER_VERSION__
    /* Wait until we can acquire the semaphore */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = 0, i = 0;
    while (sem_val != 1u && i < FCS_SEMAPHORE_TIMEOUT) {
        sem_val = semaphore->SEM[FCS_SEMAPHORE_WAYPOINT];
        i++;
    }
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Invalidate L1 */
    L1DIBAR = (uint32_t)&fcs_export_waypoint_update;
    L1DIWC = sizeof(fcs_export_waypoint_update) / 4u;
    while (L1DIWC & 0xFFFFu);
#endif

    *update = fcs_export_waypoint_update;

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_WAYPOINT] = 1u;
#endif
}

void fcs_exports_recv_path_update(struct fcs_path_update_t *update) {
#ifdef __TI_COMPILER_VERSION__
    /* Wait until we can acquire the semaphore */
    volatile CSL_SemRegs *const semaphore =
            (CSL_SemRegs*)CSL_SEMAPHORE_REGS;
    uint32_t sem_val = 0, i = 0;
    while (sem_val != 1u && i < FCS_SEMAPHORE_TIMEOUT) {
        sem_val = semaphore->SEM[FCS_SEMAPHORE_PATH];
        i++;
    }
#endif

#ifdef __TI_COMPILER_VERSION__
    /* Invalidate L1 */
    L1DIBAR = (uint32_t)&fcs_export_path_update;
    L1DIWC = sizeof(fcs_export_path_update) / 4u;
    while (L1DIWC & 0xFFFFu);
#endif

    *update = fcs_export_path_update;

#ifdef __TI_COMPILER_VERSION__
    /* Release the semaphore */
    semaphore->SEM[FCS_SEMAPHORE_PATH] = 1u;
#endif
}
