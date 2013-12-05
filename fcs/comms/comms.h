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
- flags (GPS SVs, GPS fix status, taken photo) -- 4 chars (say)

=> 159 bytes + 33 separators + 7 bytes prefix + * + 2 bytes checksum + CRLF =
   204 bytes total
*/
struct fcs_packet_state_t {
    int32_t solution_time;
    uint8_t next_waypoint_id[4u];
    double lat, lon, alt;
    double velocity[3u];
    double wind_velocity[3u];
    double attitude[3u];
    double angular_velocity[3u];
    double lat_lon_uncertainty, alt_uncertainty;
    double velocity_uncertainty[3u];
    double wind_velocity_uncertainty[3u];
    double attitude_uncertainty[3u];
    double angular_velocity_uncertainty[3u];
    uint8_t mode_indicator;
    uint8_t flags[4u];
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
- target lat (up to 7dp) -- 12 chars
- target lon (up to 7dp) -- 12 chars
- target alt (above ellipsoid, up to 2dp) -- 7 chars
- target attitude (yaw/pitch/roll, up to 3dp) -- 19 chars
- target airspeed (m/s, up to 2dp) -- 7 chars
- flags -- 5 chars (say)

=> 67 bytes + 8 separators + 7 bytes prefix + * + 2 bytes checksum + CRLF =
   87 bytes total
*/
struct fcs_packet_waypoint_t {
    uint8_t waypoint_id[4u];
    uint8_t waypoint_role;
    double target_lat, target_lon, target_alt;
    double target_attitude[3u];
    double target_airspeed;
    uint8_t flags[5u];
};

/*
GCS information (CPU->FCS): $PSFWAG
- time of solution (ms) -- 9 chars
- lat (up to 7dp) -- 12 chars
- lon (up to 7dp) -- 12 chars
- alt (m, up to 2dp) -- 7 chars
- barometric pressure (mbar, up to 2dp) -- 7 chars
- Piksi RTK data? -- ?? chars

=> 47 bytes + Piksi data + 6 separators + 7 bytes prefix + * +
   2 bytes checksum + CRLF = 65 bytes total
*/
struct fcs_packet_gcs_t {
    int32_t solution_time;
    double lat, lon, alt;
    double pressure;
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
    uint8_t param_name[24u];
    uint8_t param_value_len;
    uint8_t param_value[128u];
};

void fcs_comms_init(void);
void fcs_comms_tick(void);

enum fcs_deserialization_result_t {
    FCS_DESERIALIZATION_OK,
    FCS_DESERIALIZATION_ERROR
};

/* In all these cases, buf must be at least 256 chars long */

size_t fcs_comms_serialize_state(uint8_t *restrict buf,
const struct fcs_packet_state_t *restrict state);

enum fcs_deserialization_result_t fcs_comms_deserialize_state(
struct fcs_packet_state_t *restrict state, uint8_t *restrict buf,
size_t len);

size_t fcs_comms_serialize_waypoint(uint8_t *restrict buf,
const struct fcs_packet_waypoint_t *restrict waypoint);

enum fcs_deserialization_result_t fcs_comms_deserialize_waypoint(
struct fcs_packet_waypoint_t *restrict waypoint, uint8_t *restrict buf,
size_t len);

size_t fcs_comms_serialize_config(uint8_t *restrict buf,
const struct fcs_packet_config_t *restrict config);

enum fcs_deserialization_result_t fcs_comms_deserialize_config(
struct fcs_packet_config_t *restrict config, uint8_t *restrict buf,
size_t len);

enum fcs_deserialization_result_t fcs_comms_deserialize_gcs(
struct fcs_packet_gcs_t *restrict gcs, uint8_t *restrict buf, size_t len);

#endif
