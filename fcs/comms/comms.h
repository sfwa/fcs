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

#ifndef _FCS_COMMS_H
#define _FCS_COMMS_H

#define FCS_COMMS_MIN_PACKET_SIZE 13u

/*
State (FCS->CPU): $PSFWAS
- packet time (ms) -- 9 chars
- next waypoint ID -- 4 chars
- lat (deg to 7dp) -- 11 chars
- lon (deg to 7dp) -- 12 chars
- alt (above ellipsoid, m to 2dp) -- 7 chars
- velocity N (m/s to 2dp) -- 7 chars
- velocity E (m/s to 2dp) -- 7 chars
- velocity D (m/s to 2dp) -- 7 chars
- wind N (m/s to 2dp) -- 6 chars
- wind E (m/s to 2dp) -- 6 chars
- wind D (m/s to 2dp) -- 6 chars
- attitude (yaw/pitch/roll, degrees to 1dp) -- 16 chars
- angular velocity (yaw/pitch/roll rate, degrees/s to 1dp) -- 18 chars
- horizontal position uncertainty (95%, integral m) -- 3 chars
- alt uncertainty (95%, m to 1dp) -- 4 chars
- velocity N uncertainty (95%, integral m/s) -- 2 chars
- velocity E uncertainty (95%, integral m/s) -- 2 chars
- velocity D uncertainty (95%, integral m/s) -- 2 chars
- wind N uncertainty (95%, m/s) -- 2 chars
- wind E uncertainty (95%, m/s) -- 2 chars
- wind D uncertainty (95%, m/s) -- 2 chars
- yaw uncertainty (95%, degrees up to 90deg) -- 2 chars
- pitch uncertainty (95%, degrees up to 90deg) -- 2 chars
- roll uncertainty (95%, degrees up to 90deg) -- 2 chars
- yaw rate uncertainty (95%, degreees/s up to 90deg/s) -- 2 chars
- pitch rate uncertainty (95%, degrees/s up to 90deg/s) -- 2 chars
- roll rate uncertainty (95%, degrees/s up to 90deg/s) -- 2 chars
- FAA mode indicator ('A' = autonomous, 'D' = differential,
  'E' = dead-reckoning, 'M' = manual input, 'S' = simulated,
  'N' = not valid) -- 1 char
- flags (GPS SVs, GPS fix status, taken photo) -- 7 chars
- CRC32 -- 8 chars

=> 170 bytes + 34 separators + 7 bytes prefix + * + 2 bytes checksum + CRLF =
   216 bytes total


Waypoint information (CPU->FCS, FCS->CPU): $PSFWAP
- packet time (ms) -- 9 chars
- waypoint ID -- 4 chars
- waypoint role ("H" for home, "R" for recovery, "M" for mission boundary,
  "C" for course, "I" for image) -- 1 char
- flags -- 3 chars (say)
- target lat (up to 7dp) -- 12 chars
- target lon (up to 7dp) -- 12 chars
- target alt (above ellipsoid, up to 2dp) -- 7 chars
- target attitude (yaw/pitch/roll, up to 1dp) -- 13 chars
- target airspeed (m/s, up to 2dp) -- 7 chars
- hold duration (min, up to 60 + 1dp) -- 4 chars
- CRC32 -- 8 chars

=> 77 bytes + 9 separators + 7 bytes prefix + * + 2 bytes checksum + CRLF =
   98 bytes total


GCS information (CPU->FCS): $PSFWAG
- packet time (ms) -- 9 chars
- flags -- 4 chars
- lat (up to 7dp) -- 12 chars
- lon (up to 7dp) -- 12 chars
- alt (m, up to 2dp) -- 7 chars
- barometric pressure (mbar, up to 2dp) -- 7 chars
- Piksi RTK data? -- ?? chars
- CRC32 -- 8 chars

=> 59 bytes + Piksi data + 8 separators + 7 bytes prefix + * +
   2 bytes checksum + CRLF = 79 bytes total


Status information (FCS->CPU): $PSFWAT
- packet time (ms) -- 9 chars
- flags -- 4 chars
- I/O board resets -- 2x 3 chars
- TRICAL resets -- 2x 3 chars
- UKF resets -- 3 chars
- Main loop cycle max -- 2x 8 chars
- CPU packet RX -- 9 chars
- CPU packet RX errors -- 9 chars
- GPS num SVs -- 2 chars
- Telemetry signal (dB) -- 4 chars
- Telemetry noise (dB) -- 4 chars
- Telemetry packet RX -- 6 chars
- Telemetry packet RX errors -- 6 chars
- CRC32 -- 8 chars

=> 109 bytes + 7 bytes prefix + * +
   2 bytes checksum + CRLF = 121 bytes total


Mode packet (CPU->FCS, FCS->CPU): $PSFWAM
- packet time (ms) -- 9 chars
- mode -- up to 24 chars
- CRC32 -- 8 chars

=> 41 bytes + 3 separators + 7 bytes prefix + * + 2 bytes checksum + CRLF =
   56 bytes
*/

/* Init functions for comms module */
void fcs_comms_init(void);
void fcs_comms_tick(void);

enum fcs_deserialization_result_t {
    FCS_DESERIALIZATION_OK,
    FCS_DESERIALIZATION_ERROR
};

/* In all these cases, buf must be at least 256 chars long */

/* State serialization */
size_t fcs_comms_serialize_state(uint8_t *restrict buf,
const struct fcs_ahrs_state_t *restrict state);

/* Waypoint serialization/deserialization
size_t fcs_comms_serialize_waypoint(uint8_t *restrict buf,
const struct fcs_packet_waypoint_t *restrict waypoint);

enum fcs_deserialization_result_t fcs_comms_deserialize_waypoint(
struct fcs_packet_waypoint_t *restrict waypoint, uint8_t *restrict buf,
size_t len);

enum fcs_validation_result_t fcs_comms_validate_waypoint(
const struct fcs_packet_waypoint_t *restrict waypoint);
*/

/* Status serialization */
size_t fcs_comms_serialize_status(uint8_t *restrict buf,
const struct fcs_ahrs_state_t *restrict state,
const struct fcs_stats_counter_t *restrict counters,
const struct fcs_peripheral_state_t *restrict peripheral_state);

/* Config serialization and deserialization */
enum fcs_deserialization_result_t fcs_comms_deserialize_config(
const uint8_t *restrict packet, size_t packet_length);

size_t fcs_comms_serialize_config(const uint8_t *restrict packet,
const uint8_t *restrict config_key, size_t config_key_len);

#endif
