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
- time of solution (ms) -- 9 chars
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
- attitude (yaw/pitch/roll, degrees to 2dp) -- 19 chars
- angular velocity (yaw/pitch/roll rate, degrees/s to 2dp) -- 21 chars
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
*/
struct fcs_packet_state_t {
    int32_t solution_time;
    uint8_t next_waypoint_id[4];
    double lat, lon, alt;
    double velocity[3];
    double wind_velocity[3];
    double yaw, pitch, roll;
    double angular_velocity[3];
    double lat_lon_uncertainty, alt_uncertainty;
    double velocity_uncertainty[3];
    double wind_velocity_uncertainty[3];
    double yaw_uncertainty, pitch_uncertainty, roll_uncertainty;
    double angular_velocity_uncertainty[3];
    uint8_t mode_indicator;
    uint8_t flags[7];
    uint32_t crc32;
};

#define FCS_STATE_MAX_LAT_LON_UNCERTAINTY 1000.0
#define FCS_STATE_MAX_ALT_UNCERTAINTY 20.0
#define FCS_STATE_MAX_VELOCITY_UNCERTAINTY 10.0
#define FCS_STATE_MAX_ATTITUDE_UNCERTAINTY 30.0
#define FCS_STATE_MAX_ANGULAR_VELOCITY_UNCERTAINTY 90.0

/*
Waypoint information (CPU->FCS, FCS->CPU): $PSFWAP
- waypoint ID -- 4 chars
- waypoint role ("H" for home, "R" for recovery, "M" for mission boundary,
  "C" for course, "I" for image) -- 1 char
- flags -- 3 chars (say)
- target lat (up to 7dp) -- 12 chars
- target lon (up to 7dp) -- 12 chars
- target alt (above ellipsoid, up to 2dp) -- 7 chars
- target attitude (yaw/pitch/roll, up to 3dp) -- 19 chars
- target airspeed (m/s, up to 2dp) -- 7 chars
- CRC32 -- 8 chars

=> 73 bytes + 8 separators + 7 bytes prefix + * + 2 bytes checksum + CRLF =
   93 bytes total
*/
struct fcs_packet_waypoint_t {
    uint8_t waypoint_id[4];
    uint8_t waypoint_role;
    uint8_t flags[3];
    double target_lat, target_lon, target_alt;
    double target_yaw, target_pitch, target_roll;
    double target_airspeed;
    uint32_t crc32;
};

/*
GCS information (CPU->FCS): $PSFWAG
- time of solution (ms) -- 9 chars
- flags -- 4 chars
- lat (up to 7dp) -- 12 chars
- lon (up to 7dp) -- 12 chars
- alt (m, up to 2dp) -- 7 chars
- barometric pressure (mbar, up to 2dp) -- 7 chars
- Piksi RTK data? -- ?? chars
- CRC32 -- 8 chars

=> 59 bytes + Piksi data + 8 separators + 7 bytes prefix + * +
   2 bytes checksum + CRLF = 79 bytes total
*/
struct fcs_packet_gcs_t {
    int32_t solution_time;
    uint8_t flags[4];
    double lat, lon, alt;
    double pressure;
    uint32_t crc32;
};

/*
Config information (CPU->FCS, FCS->CPU): $PSFWAC
- param name -- 24 chars
- param value (Base64) -- up to 192 chars (128 bytes)

=> up to 216 bytes + 3 separators + 7 bytes prefix + * + 2 bytes checksum +
   CRLF = 231 bytes total
*/
struct fcs_packet_config_t {
    uint8_t param_name_len;
    uint8_t param_name[24];
    uint8_t param_value_len;
    uint8_t param_value[128];
    uint32_t crc32;
};

/*
Status information (FCS->CPU): $PSFWAT
- time of status (ms) -- 9 chars
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
*/
struct fcs_packet_status_t {
    int32_t solution_time;
    uint8_t flags[4];
    int32_t ioboard_resets[2];
    int32_t trical_resets[2];
    int32_t ukf_resets;
    int32_t main_loop_cycle_max[2];
    int32_t cpu_packet_rx;
    int32_t cpu_packet_rx_err;
    int32_t gps_num_svs;
    int32_t telemetry_signal_db;
    int32_t telemetry_noise_db;
    int32_t telemetry_packet_rx;
    int32_t telemetry_packet_rx_err;
    uint32_t crc32;
};

/*
Sensor calibration information (CPU->FCS, FCS->CPU): $PSFWAL
- Sensor type -- 1 char (lookup table)
- Sensor ID -- 1 char (0-3)
- Flags -- 4 chars
- Calibration type -- 1 char (lookup table)
- Error -- 8 chars
- Params * 9 -- 8 chars * 9
-
*/

/* Init functions for comms module */
void fcs_comms_init(void);
void fcs_comms_tick(void);

enum fcs_deserialization_result_t {
    FCS_DESERIALIZATION_OK,
    FCS_DESERIALIZATION_ERROR
};

enum fcs_validation_result_t {
    FCS_VALIDATION_OK,
    FCS_VALIDATION_ERROR
};

/* In all these cases, buf must be at least 256 chars long */

/* State serialization/deserialization */
size_t fcs_comms_serialize_state(uint8_t *restrict buf,
const struct fcs_packet_state_t *restrict state);

enum fcs_deserialization_result_t fcs_comms_deserialize_state(
struct fcs_packet_state_t *restrict state, uint8_t *restrict buf,
size_t len);

enum fcs_validation_result_t fcs_comms_validate_state(
const struct fcs_packet_state_t *restrict state);

/* Waypoint serialization/deserialization */
size_t fcs_comms_serialize_waypoint(uint8_t *restrict buf,
const struct fcs_packet_waypoint_t *restrict waypoint);

enum fcs_deserialization_result_t fcs_comms_deserialize_waypoint(
struct fcs_packet_waypoint_t *restrict waypoint, uint8_t *restrict buf,
size_t len);

enum fcs_validation_result_t fcs_comms_validate_waypoint(
const struct fcs_packet_waypoint_t *restrict waypoint);

/* Config serialization/deserialization */
size_t fcs_comms_serialize_config(uint8_t *restrict buf,
const struct fcs_packet_config_t *restrict config);

enum fcs_deserialization_result_t fcs_comms_deserialize_config(
struct fcs_packet_config_t *restrict config, uint8_t *restrict buf,
size_t len);

enum fcs_validation_result_t fcs_comms_validate_config(
const struct fcs_packet_config_t *restrict config);

/* GCS deserialization */
enum fcs_deserialization_result_t fcs_comms_deserialize_gcs(
struct fcs_packet_gcs_t *restrict gcs, uint8_t *restrict buf, size_t len);

enum fcs_validation_result_t fcs_comms_validate_gcs(
const struct fcs_packet_gcs_t *restrict gcs);

/* Status serialization */
size_t fcs_comms_serialize_status(uint8_t *restrict buf,
const struct fcs_packet_status_t *restrict status);

enum fcs_validation_result_t fcs_comms_validate_status(
const struct fcs_packet_status_t *restrict status);

#endif
