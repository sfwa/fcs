#include "test.h"

#include <cstdlib>
#include <cmath>

extern "C" {
#include "config/config.h"
#include "util/util.h"
#include "util/3dmath.h"
#include "nmpc/cnmpc.h"
#include "control/control.h"

/* Prototypes for private test functions */

/* from control/control.c */
float _interpolate_linear(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t);

float _interpolate_figure_eight(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t);

float _interpolate_dubins(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
float t);

float _next_point_from_path(struct fcs_waypoint_t *new_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind,
const struct fcs_waypoint_t *start, const struct fcs_waypoint_t *end,
enum fcs_path_type_t type, float t);

void _ned_from_point_diff(float *restrict ned,
const struct fcs_waypoint_t *restrict ref,
const struct fcs_waypoint_t *restrict point);

void _get_ahrs_state(float *restrict state, float *restrict wind,
volatile struct fcs_ahrs_state_t *ahrs_state,
const struct fcs_waypoint_t *restrict reference);

void _make_reference(float *restrict reference,
const struct fcs_waypoint_t *current_point,
const struct fcs_waypoint_t *last_point, const float *restrict wind);

void _next_point(struct fcs_waypoint_t *restrict new_point,
uint16_t *restrict new_point_path_id,
const struct fcs_waypoint_t *restrict last_point,
const uint16_t *restrict last_point_path_id, const float *restrict wind,
struct fcs_nav_state_t *nav);

void _shift_horizon(struct fcs_nav_state_t *nav, const float *restrict wind);

void _recalculate_horizon(struct fcs_nav_state_t *nav,
const float *restrict wind);
}

TEST(Control, Initialisation) {
    fcs_control_init();
}

TEST(Control, NEDFromLatLonSE_N) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = -37.0 * (M_PI/180.0);
    ref.lon = 145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree north should result in 111.3km north value */
    point.lat = ref.lat + 1.0 * (M_PI/180.0);
    point.lon = ref.lon;
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(111319.5, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonSE_S) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = -37.0 * (M_PI/180.0);
    ref.lon = 145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree south should result in -111.3km north value */
    point.lat = ref.lat - 1.0 * (M_PI/180.0);
    point.lon = ref.lon;
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(-111319.5, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonSE_E) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = -37.0 * (M_PI/180.0);
    ref.lon = 145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree east should result in 88.904km east value */
    point.lat = ref.lat;
    point.lon = ref.lon + 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(88903.7, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonSE_W) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = -37.0 * (M_PI/180.0);
    ref.lon = 145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree west should result in -88.904km east value */
    point.lat = ref.lat;
    point.lon = ref.lon - 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(-88903.7, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonSE_U) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = -37.0 * (M_PI/180.0);
    ref.lon = 145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 10m up should result in -10m down value */
    point.lat = ref.lat;
    point.lon = ref.lon;
    point.alt = ref.alt + 10.0;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(-10.0, result[2]);
}

TEST(Control, NEDFromLatLonSE_D) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = -37.0 * (M_PI/180.0);
    ref.lon = 145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 10m down should result in 10m down value */
    point.lat = ref.lat;
    point.lon = ref.lon;
    point.alt = ref.alt - 10.0;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(10.0, result[2]);
}

TEST(Control, NEDFromLatLonNW_N) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 37.0 * (M_PI/180.0);
    ref.lon = -145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree north should result in 111.3km north value */
    point.lat = ref.lat + 1.0 * (M_PI/180.0);
    point.lon = ref.lon;
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(111319.5, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonNW_S) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 37.0 * (M_PI/180.0);
    ref.lon = -145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree south should result in -111.3km north value */
    point.lat = ref.lat - 1.0 * (M_PI/180.0);
    point.lon = ref.lon;
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(-111319.5, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonNW_E) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 37.0 * (M_PI/180.0);
    ref.lon = -145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree east should result in 88.904km east value */
    point.lat = ref.lat;
    point.lon = ref.lon + 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(88903.7, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonNW_W) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 37.0 * (M_PI/180.0);
    ref.lon = -145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree west should result in -88.904km east value */
    point.lat = ref.lat;
    point.lon = ref.lon - 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(-88903.7, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonNW_U) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 37.0 * (M_PI/180.0);
    ref.lon = -145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 10m up should result in -10m down value */
    point.lat = ref.lat;
    point.lon = ref.lon;
    point.alt = ref.alt + 10.0;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(-10.0, result[2]);
}

TEST(Control, NEDFromLatLonNW_D) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 37.0 * (M_PI/180.0);
    ref.lon = -145.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 10m down should result in 10m down value */
    point.lat = ref.lat;
    point.lon = ref.lon;
    point.alt = ref.alt - 10.0;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(10.0, result[2]);
}

TEST(Control, NEDFromLatLonEquatorN) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 0.0 * (M_PI/180.0);
    ref.lon = 0.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree north should result in 111.3km north value */
    point.lat = ref.lat + 1.0 * (M_PI/180.0);
    point.lon = ref.lon;
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(111319.5, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonEquatorS) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 0.0 * (M_PI/180.0);
    ref.lon = 0.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree south should result in -111.3km north value */
    point.lat = ref.lat - 1.0 * (M_PI/180.0);
    point.lon = ref.lon;
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(-111319.5, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonEquatorE) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 0.0 * (M_PI/180.0);
    ref.lon = 0.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree east should result in 111.3km east value */
    point.lat = ref.lat;
    point.lon = ref.lon + 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(111319.5, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonEquatorW) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 0.0 * (M_PI/180.0);
    ref.lon = 0.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree west should result in -111.3km east value */
    point.lat = ref.lat;
    point.lon = ref.lon - 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(-111319.5, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonEquatorU) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 0.0 * (M_PI/180.0);
    ref.lon = 0.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 10m up should result in -10m down value */
    point.lat = ref.lat;
    point.lon = ref.lon;
    point.alt = ref.alt + 10.0;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(-10.0, result[2]);
}

TEST(Control, NEDFromLatLonEquatorD) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    ref.lat = 0.0 * (M_PI/180.0);
    ref.lon = 0.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 10m down should result in 10m down value */
    point.lat = ref.lat;
    point.lon = ref.lon;
    point.alt = ref.alt - 10.0;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(10.0, result[2]);
}

TEST(Control, NEDFromLatLonSouthPole) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    /* South pole */
    ref.lat = -90.0 * (M_PI/180.0);
    ref.lon = 0.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree north should result in 111.3km north value */
    point.lat = ref.lat + 1.0 * (M_PI/180.0);
    point.lon = ref.lon;
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(111319.5, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);

    /* 1 degree east should result in 0km east value */
    point.lat = ref.lat;
    point.lon = ref.lon + 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_NEAR(0.0, result[1], 1e-10);
    EXPECT_FLOAT_EQ(0.0, result[2]);

    /* 1 degree west should result in 0km east value */
    point.lat = ref.lat;
    point.lon = ref.lon - 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_NEAR(0.0, result[1], 1e-10);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, NEDFromLatLonNorthPole) {
    struct fcs_waypoint_t ref, point;
    float result[3];

    /* North pole */
    ref.lat = 90.0 * (M_PI/180.0);
    ref.lon = 0.0 * (M_PI/180.0);
    ref.alt = 10.0;

    /* 1 degree south should result in -111.3km north value */
    point.lat = ref.lat - 1.0 * (M_PI/180.0);
    point.lon = ref.lon;
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(-111319.5, result[0]);
    EXPECT_FLOAT_EQ(0.0, result[1]);
    EXPECT_FLOAT_EQ(0.0, result[2]);

    /* 1 degree east should result in 0km east value */
    point.lat = ref.lat;
    point.lon = ref.lon + 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_NEAR(0.0, result[1], 1e-10);
    EXPECT_FLOAT_EQ(0.0, result[2]);

    /* 1 degree west should result in 0km east value */
    point.lat = ref.lat;
    point.lon = ref.lon - 1.0 * (M_PI/180.0);
    point.alt = ref.alt;

    _ned_from_point_diff(result, &ref, &point);
    EXPECT_FLOAT_EQ(0.0, result[0]);
    EXPECT_NEAR(0.0, result[1], 1e-10);
    EXPECT_FLOAT_EQ(0.0, result[2]);
}

TEST(Control, InterpolateLinearPathNoWindNE) {
    struct fcs_waypoint_t result, last_point, start, end;
    float t = OCP_STEP_LENGTH, result_t, wind[3] = {0.0, 0.0, 0.0};

    start.lat = 0.0 * (M_PI/180.0);
    start.lon = 145.0 * (M_PI/180.0);
    start.alt = 10.0;
    start.airspeed = 100.0;
    start.yaw = 0.0;
    start.pitch = 0.0;
    start.roll = 145.0 * (M_PI/180.0);

    /* 1 degree north and east, 10m up */
    end.lat = start.lat + 1.0 * (M_PI/180.0);
    end.lon = start.lon + 1.0 * (M_PI/180.0);
    end.alt = start.alt + 10.0;
    end.airspeed = 200.0;
    end.yaw = 0.0;
    end.pitch = 0.0;
    end.roll = -145.0 * (M_PI/180.0);

    /* Test the start of the path */
    memcpy(&last_point, &start, sizeof(last_point));
    result_t = _interpolate_linear(&result, &last_point, wind, &start, &end,
                                   t);
    EXPECT_FLOAT_EQ(0.02, result_t);
    EXPECT_DOUBLE_EQ(2.2172830881518887e-07, result.lat);
    EXPECT_DOUBLE_EQ(2.5307276371200866, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(100.0, result.airspeed);
    EXPECT_NEAR(M_PI / 4.0, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(145.0 * (M_PI/180.0), result.roll);

    /* Test the midpoint of the path */
    last_point.lat = start.lat + 0.5 * (M_PI/180.0);
    last_point.lon = start.lon + 0.5 * (M_PI/180.0);
    result_t = _interpolate_linear(&result, &last_point, wind, &start, &end,
                                   t);
    EXPECT_FLOAT_EQ(0.02, result_t);
    EXPECT_DOUBLE_EQ(0.008726867992491014, result.lat);
    EXPECT_DOUBLE_EQ(2.5394542833842668, result.lon);
    EXPECT_FLOAT_EQ(15.0, result.alt);
    EXPECT_FLOAT_EQ(150.0, result.airspeed);
    EXPECT_NEAR(M_PI / 4.0, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(180.0 * (M_PI/180.0), absval(result.roll));

    /* Test the end of the path */
    memcpy(&last_point, &end, sizeof(last_point));
    result_t = _interpolate_linear(&result, &last_point, wind, &start, &end,
                                   t);
    EXPECT_FLOAT_EQ(0.0, result_t);
    EXPECT_DOUBLE_EQ(end.lat, result.lat);
    EXPECT_DOUBLE_EQ(end.lon, result.lon);
    EXPECT_FLOAT_EQ(end.alt, result.alt);
    EXPECT_FLOAT_EQ(200.0, result.airspeed);
    EXPECT_NEAR(M_PI / 4.0, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(-145.0 * (M_PI/180.0), result.roll);
}

TEST(Control, InterpolateLinearPathNoWindS) {
    struct fcs_waypoint_t result, last_point, start, end;
    float t = OCP_STEP_LENGTH, result_t, wind[3] = {0.0, 0.0, 0.0};

    start.lat = 0.0 * (M_PI/180.0);
    start.lon = 145.0 * (M_PI/180.0);
    start.alt = 10.0;
    start.airspeed = 100.0;
    start.yaw = 0.0;
    start.pitch = 0.0;
    start.roll = 145.0 * (M_PI/180.0);

    /* 1 degree south, 10m down */
    end.lat = start.lat - 1.0 * (M_PI/180.0);
    end.lon = start.lon;
    end.alt = start.alt - 10.0;
    end.airspeed = 100.0;
    end.yaw = 0.0;
    end.pitch = 0.0;
    end.roll = 0.0 * (M_PI/180.0);

    /* Test the start of the path */
    memcpy(&last_point, &start, sizeof(last_point));
    result_t = _interpolate_linear(&result, &last_point, wind, &start, &end,
                                   t);
    EXPECT_FLOAT_EQ(0.02, result_t);
    EXPECT_DOUBLE_EQ(-3.1357118156861731e-07, result.lat);
    EXPECT_DOUBLE_EQ(start.lon, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(100.0, result.airspeed);
    EXPECT_NEAR(M_PI, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(145.0 * (M_PI/180.0), result.roll);

    /* Test the midpoint of the path */
    last_point.lat = start.lat - 0.5 * (M_PI/180.0);
    last_point.lon = start.lon;
    result_t = _interpolate_linear(&result, &last_point, wind, &start, &end,
                                   t);
    EXPECT_FLOAT_EQ(0.02, result_t);
    EXPECT_DOUBLE_EQ(-0.0087269598311532168, result.lat);
    EXPECT_DOUBLE_EQ(start.lon, result.lon);
    EXPECT_FLOAT_EQ(5.0, result.alt);
    EXPECT_FLOAT_EQ(100.0, result.airspeed);
    EXPECT_NEAR(M_PI, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(72.5 * (M_PI/180.0), result.roll);

    /* Test the end of the path */
    memcpy(&last_point, &end, sizeof(last_point));
    result_t = _interpolate_linear(&result, &last_point, wind, &start, &end,
                                   t);
    EXPECT_FLOAT_EQ(0.0, result_t);
    EXPECT_DOUBLE_EQ(end.lat, result.lat);
    EXPECT_DOUBLE_EQ(start.lon, result.lon);
    EXPECT_FLOAT_EQ(end.alt, result.alt);
    EXPECT_FLOAT_EQ(100.0, result.airspeed);
    EXPECT_NEAR(M_PI, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(0.0, result.roll);
}

TEST(Control, InterpolateLinearPathHeadwindS) {
    struct fcs_waypoint_t result, last_point, start, end;
    float t = OCP_STEP_LENGTH, result_t, wind[3] = {0.0, 0.0, 0.0};

    start.lat = 0.0 * (M_PI/180.0);
    start.lon = 145.0 * (M_PI/180.0);
    start.alt = 10.0;
    start.airspeed = 100.0;
    start.yaw = 0.0;
    start.pitch = 0.0;
    start.roll = 145.0 * (M_PI/180.0);

    /* 1 degree south, 10m down */
    end.lat = start.lat - 1.0 * (M_PI/180.0);
    end.lon = start.lon;
    end.alt = start.alt - 10.0;
    end.airspeed = 100.0;
    end.yaw = 0.0;
    end.pitch = 0.0;
    end.roll = 0.0 * (M_PI/180.0);

    wind[0] = 50.0;

    memcpy(&last_point, &start, sizeof(last_point));
    result_t = _interpolate_linear(&result, &last_point, wind, &start, &end,
                                   t);
    EXPECT_FLOAT_EQ(0.02, result_t);
    EXPECT_DOUBLE_EQ(-3.1357118156861731e-07 * 0.5, result.lat);
    EXPECT_DOUBLE_EQ(start.lon, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(100.0, result.airspeed);
    EXPECT_NEAR(M_PI, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(145.0 * (M_PI/180.0), result.roll);
}

TEST(Control, InterpolateFigureEightPathEW) {
    struct fcs_waypoint_t result, last_point, start, end;
    float t = OCP_STEP_LENGTH, result_t, wind[3] = {0.0, 0.0, 0.0};
    size_t i;

    start.lat = 0.0 * (M_PI/180.0);
    start.lon = 0.0 * (M_PI/180.0);
    start.alt = 10.0;
    start.airspeed = 20.0;
    start.yaw = 0.0;
    start.pitch = 0.0;
    start.roll = 0.0;

    memcpy(&end, &start, sizeof(end));
    memcpy(&last_point, &start, sizeof(last_point));

    /* Initial point (centre, heading north and turning clockwise) */
    result_t = _interpolate_figure_eight(
        &result, &last_point, wind, &start, &end, t);
    EXPECT_FLOAT_EQ(0.02, result_t);
    EXPECT_DOUBLE_EQ(6.2710056695540948e-08, result.lat);
    EXPECT_DOUBLE_EQ(6.2705954174843071e-10, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(0.02, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * 0.25, result.roll);

    for (i = 0; i < 78; i++) {
        memcpy(&last_point, &result, sizeof(last_point));
        result_t = _interpolate_figure_eight(
            &result, &last_point, wind, &start, &end, t);
        EXPECT_FLOAT_EQ(0.02, result_t);
    }

    /* North-est, heading east */
    EXPECT_DOUBLE_EQ(3.1355791098885189e-06, result.lat);
    EXPECT_DOUBLE_EQ(3.1645685117255828e-06, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(M_PI / 2.0, result.yaw, 1e-2);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * 0.25, result.roll);

    for (; i < 156; i++) {
        memcpy(&last_point, &result, sizeof(last_point));
        result_t = _interpolate_figure_eight(
            &result, &last_point, wind, &start, &end, t);
        EXPECT_FLOAT_EQ(0.02, result_t);
    }

    /* Due east, heading south */
    EXPECT_DOUBLE_EQ(5.0012478285552483e-09, result.lat);
    EXPECT_DOUBLE_EQ(6.2714201830121243e-06, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(M_PI, result.yaw, 1e-2);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * 0.25, result.roll);

    for (; i < 313; i++) {
        memcpy(&last_point, &result, sizeof(last_point));
        result_t = _interpolate_figure_eight(
            &result, &last_point, wind, &start, &end, t);
        EXPECT_FLOAT_EQ(0.02, result_t);
    }

    /* Back to the origin, heading north */
    EXPECT_NEAR(0.0, result.lat, 1e-7);
    EXPECT_NEAR(0.0, result.lon, 1e-7);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(2.0 * M_PI, result.yaw, 1e-2);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * 0.25, result.roll);

    for (; i < 391; i++) {
        memcpy(&last_point, &result, sizeof(last_point));
        result_t = _interpolate_figure_eight(
            &result, &last_point, wind, &start, &end, t);
        EXPECT_FLOAT_EQ(0.02, result_t);
    }

    /* North-west, heading west */
    EXPECT_DOUBLE_EQ(3.1356212752037558e-06, result.lat);
    EXPECT_DOUBLE_EQ(-3.1118573820081286e-06, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(1.5 * M_PI, result.yaw, 1e-2);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * -0.25, result.roll);
}

TEST(Control, InterpolateFigureEightPathNS) {
    struct fcs_waypoint_t result, last_point, start, end;
    float t = OCP_STEP_LENGTH, result_t, wind[3] = {0.0, 0.0, 0.0};
    size_t i;

    start.lat = 0.0 * (M_PI/180.0);
    start.lon = 0.0 * (M_PI/180.0);
    start.alt = 10.0;
    start.airspeed = 20.0;
    start.yaw = M_PI * 1.5;
    start.pitch = 0.0;
    start.roll = 0.0;

    memcpy(&end, &start, sizeof(end));
    memcpy(&last_point, &start, sizeof(last_point));

    /* Initial point (centre, heading west and turning clockwise) */
    result_t = _interpolate_figure_eight(
        &result, &last_point, wind, &start, &end, t);
    EXPECT_FLOAT_EQ(0.02, result_t);
    EXPECT_DOUBLE_EQ(6.2706027183824737e-10, result.lat);
    EXPECT_DOUBLE_EQ(-6.2709995952068201e-08, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(M_PI * 1.5 + 0.02, result.yaw, 1e-4);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * 0.25, result.roll);

    for (i = 0; i < 78; i++) {
        memcpy(&last_point, &result, sizeof(last_point));
        result_t = _interpolate_figure_eight(
            &result, &last_point, wind, &start, &end, t);
        EXPECT_FLOAT_EQ(0.02, result_t);
    }

    /* North-west, heading north */
    EXPECT_DOUBLE_EQ(3.1645664184120605e-06, result.lat);
    EXPECT_DOUBLE_EQ(-3.1355791098885189e-06, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(0.0, result.yaw, 1e-2);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * 0.25, result.roll);

    for (; i < 156; i++) {
        memcpy(&last_point, &result, sizeof(last_point));
        result_t = _interpolate_figure_eight(
            &result, &last_point, wind, &start, &end, t);
        EXPECT_FLOAT_EQ(0.02, result_t);
    }

    /* Due north, heading east */
    EXPECT_DOUBLE_EQ(6.2714201830121243e-06, result.lat);
    EXPECT_DOUBLE_EQ(-5.0017215108283092e-09, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(M_PI * 0.5, result.yaw, 1e-2);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * 0.25, result.roll);

    for (; i < 313; i++) {
        memcpy(&last_point, &result, sizeof(last_point));
        result_t = _interpolate_figure_eight(
            &result, &last_point, wind, &start, &end, t);
        EXPECT_FLOAT_EQ(0.02, result_t);
    }

    /* Back to the origin, heading west */
    EXPECT_NEAR(0.0, result.lat, 1e-7);
    EXPECT_NEAR(0.0, result.lon, 1e-7);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(1.5 * M_PI, result.yaw, 1e-2);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * 0.25, result.roll);

    for (; i < 391; i++) {
        memcpy(&last_point, &result, sizeof(last_point));
        result_t = _interpolate_figure_eight(
            &result, &last_point, wind, &start, &end, t);
        EXPECT_FLOAT_EQ(0.02, result_t);
    }

    /* South-west, heading south */
    EXPECT_DOUBLE_EQ(-3.1118576810529175e-06, result.lat);
    EXPECT_DOUBLE_EQ(-3.1356212752037558e-06, result.lon);
    EXPECT_FLOAT_EQ(10.0, result.alt);
    EXPECT_FLOAT_EQ(20.0, result.airspeed);
    EXPECT_NEAR(M_PI, result.yaw, 1e-2);
    EXPECT_FLOAT_EQ(0.0, result.pitch);
    EXPECT_FLOAT_EQ(M_PI * -0.25, result.roll);
}
