#ifndef _3DMATH_H_
#define _3DMATH_H_

#define X 0
#define Y 1
#define Z 2
#define W 3

#ifndef absval
#define absval(x) ((x) < 0 ? -x : x)
#endif

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef M_PI
#define M_PI ((real_t)3.14159265358979323846)
#define M_PI_2 (M_PI * 0.5)
#define M_PI_4 (M_PI * 0.25)
#endif

/* Non-TI compatibility */
#ifndef __TI_COMPILER_VERSION__
#define _nassert(x)
#endif

static inline void vector3_cross_d(double *restrict result,
const double *restrict v1,
const double *restrict v2) {
    assert(result && v1 && v2 && result != v1 && result != v2 && v1 != v2);
    _nassert((size_t)result % 8 == 0);
    _nassert((size_t)v1 % 8 == 0);
    _nassert((size_t)v2 % 8 == 0);

    double r0, r1, r2, r3, r4, r5;

    r0 = v1[Y]*v2[Z];
    r1 = v1[Z]*v2[Y];
    r2 = v1[Z]*v2[X];
    r3 = v1[X]*v2[Z];
    r4 = v1[X]*v2[Y];
    r5 = v1[Y]*v2[X];

    result[X] = r0 - r1;
    result[Y] = r2 - r3;
    result[Z] = r4 - r5;
}

static inline void vector3_cross_f(float *restrict result,
const float *restrict v1,
const float *restrict v2) {
    assert(result && v1 && v2 && result != v1 && result != v2 && v1 != v2);
    _nassert((size_t)result % 4 == 0);
    _nassert((size_t)v1 % 4 == 0);
    _nassert((size_t)v2 % 4 == 0);

    float r0, r1, r2, r3, r4, r5;

    r0 = v1[Y]*v2[Z];
    r1 = v1[Z]*v2[Y];
    r2 = v1[Z]*v2[X];
    r3 = v1[X]*v2[Z];
    r4 = v1[X]*v2[Y];
    r5 = v1[Y]*v2[X];

    result[X] = r0 - r1;
    result[Y] = r2 - r3;
    result[Z] = r4 - r5;
}

static inline void quaternion_vector3_multiply_d(double *restrict result,
const double *restrict q,
const double *restrict v) {
    /*
    Multiply a quaternion by a vector (i.e. transform a vectory by a
    quaternion)

    v' = q * v * conjugate(q), or:
    t = 2 * cross(q.xyz, v)
    v' = v + q.w * t + cross(q.xyz, t)

    http://molecularmusings.wordpress.com/2013/05/24/a-faster-quaternion-vector-multiplication/
    */

    assert(result && q && v && result != v && result != q);
    _nassert((size_t)result % 8 == 0);
    _nassert((size_t)q % 8 == 0);
    _nassert((size_t)v % 8 == 0);

    register double rx, ry, rz, tx, ty, tz;

    tx = q[Y]*v[Z];
    ty = q[Z]*v[X];
    tx -= q[Z]*v[Y];
    ty -= q[X]*v[Z];
    tz = q[X]*v[Y];
    ty *= 2.0;
    tz -= q[Y]*v[X];
    tx *= 2.0;
    tz *= 2.0;

    rx = v[X];
    rx += q[W]*tx;
    rx += q[Y]*tz;
    rx -= q[Z]*ty;
    result[X] = rx;

    ry = v[Y];
    ry += q[W]*ty;
    ry += q[Z]*tx;
    ry -= q[X]*tz;
    result[Y] = ry;

    rz = v[Z];
    rz += q[W]*tz;
    rz -= q[Y]*tx;
    rz += q[X]*ty;
    result[Z] = rz;
}

static inline void quaternion_vector3_multiply_f(float *restrict result,
const float *restrict q,
const float *restrict v) {
    /*
    Multiply a quaternion by a vector (i.e. transform a vectory by a
    quaternion)

    v' = q * v * conjugate(q), or:
    t = 2 * cross(q.xyz, v)
    v' = v + q.w * t + cross(q.xyz, t)

    http://molecularmusings.wordpress.com/2013/05/24/a-faster-quaternion-vector-multiplication/
    */

    assert(result && q && v && result != v && result != q);
    _nassert((size_t)result % 4 == 0);
    _nassert((size_t)q % 4 == 0);
    _nassert((size_t)v % 4 == 0);

    register float rx, ry, rz, tx, ty, tz;

    tx = q[Y]*v[Z];
    ty = q[Z]*v[X];
    tx -= q[Z]*v[Y];
    ty -= q[X]*v[Z];
    tz = q[X]*v[Y];
    ty *= 2.0;
    tz -= q[Y]*v[X];
    tx *= 2.0;
    tz *= 2.0;

    rx = v[X];
    rx += q[W]*tx;
    rx += q[Y]*tz;
    rx -= q[Z]*ty;
    result[X] = rx;

    ry = v[Y];
    ry += q[W]*ty;
    ry += q[Z]*tx;
    ry -= q[X]*tz;
    result[Y] = ry;

    rz = v[Z];
    rz += q[W]*tz;
    rz -= q[Y]*tx;
    rz += q[X]*ty;
    result[Z] = rz;
}

static inline void quaternion_normalize_d(double result[4], const double q[4],
bool force_positive) {
    assert(result && q);
    _nassert((double)result % 8 == 0);
    _nassert((double)q % 8 == 0);

    double norm = 1.0 / sqrt(q[X]*q[X] + q[Y]*q[Y] + q[Z]*q[Z] + q[W]*q[W]);
    if (force_positive && q[W] < 0.0) {
        norm = -norm;
    }

    result[X] = q[X] * norm;
    result[Y] = q[Y] * norm;
    result[Z] = q[Z] * norm;
    result[W] = q[W] * norm;
}

static inline void quaternion_normalize_f(float result[4], const float q[4],
bool force_positive) {
    assert(result && q);
    _nassert((float)result % 4 == 0);
    _nassert((float)q % 4 == 0);

    float norm = 1.0 / sqrt(q[X]*q[X] + q[Y]*q[Y] + q[Z]*q[Z] + q[W]*q[W]);
    if (force_positive && q[W] < 0.0) {
        norm = -norm;
    }

    result[X] = q[X] * norm;
    result[Y] = q[Y] * norm;
    result[Z] = q[Z] * norm;
    result[W] = q[W] * norm;
}

static inline void quaternion_multiply_d(double *restrict result,
const double q1[4], const double q2[4]) {
    assert(result && q1 && q2 && result != q1 && result != q2);
    _nassert((size_t)result % 8 == 0);
    _nassert((size_t)q1 % 8 == 0);
    _nassert((size_t)q2 % 8 == 0);

    result[W] = q1[W]*q2[W] - q1[X]*q2[X] - q1[Y]*q2[Y] - q1[Z]*q2[Z];
    result[X] = q1[W]*q2[X] + q1[X]*q2[W] + q1[Y]*q2[Z] - q1[Z]*q2[Y];
    result[Y] = q1[W]*q2[Y] - q1[X]*q2[Z] + q1[Y]*q2[W] + q1[Z]*q2[X];
    result[Z] = q1[W]*q2[Z] + q1[X]*q2[Y] - q1[Y]*q2[X] + q1[Z]*q2[W];
}

static inline void quaternion_multiply_f(float *restrict result,
const float q1[4], const float q2[4]) {
    assert(result && q1 && q2 && result != q1 && result != q2);
    _nassert((size_t)result % 4 == 0);
    _nassert((size_t)q1 % 4 == 0);
    _nassert((size_t)q2 % 4 == 0);

    result[W] = q1[W]*q2[W] - q1[X]*q2[X] - q1[Y]*q2[Y] - q1[Z]*q2[Z];
    result[X] = q1[W]*q2[X] + q1[X]*q2[W] + q1[Y]*q2[Z] - q1[Z]*q2[Y];
    result[Y] = q1[W]*q2[Y] - q1[X]*q2[Z] + q1[Y]*q2[W] + q1[Z]*q2[X];
    result[Z] = q1[W]*q2[Z] + q1[X]*q2[Y] - q1[Y]*q2[X] + q1[Z]*q2[W];
}

static void matrix_multiply_d(double *restrict C, const double B[],
const double A[], const size_t AR, const size_t AC, const size_t BR,
const size_t BC, const double mul) {
#pragma unused(BC)
    /*
    Calculate C = AB, where A has AN rows and AM columns, and B has BN rows
    and BM columns. C must be AN x BM.
    */
    assert(A && B && C && AR == BC);
    _nassert((size_t)C % 8 == 0);
    _nassert((size_t)B % 8 == 0);
    _nassert((size_t)A % 8 == 0);

    size_t i, j, k;
    #pragma MUST_ITERATE(1,24)
    for (i = 0; i < AC; i++) {
        #pragma MUST_ITERATE(1,24)
        for (j = 0; j < BR; j++) {
            double t = 0.0;
            #pragma MUST_ITERATE(1,24)
            for (k = 0; k < AR; k++) {
                t += A[i * AR + k] * B[k * BR + j];
            }

            C[i * BR + j] = t * mul;
        }
    }
}

static void matrix_multiply_f(float *restrict C, const float B[],
const float A[], const size_t AR, const size_t AC, const size_t BR,
const size_t BC, const float mul) {
#pragma unused(BC)
    /*
    Calculate C = AB, where A has AN rows and AM columns, and B has BN rows
    and BM columns. C must be AN x BM.
    */
    assert(A && B && C && AR == BC);
    _nassert((size_t)C % 4 == 0);
    _nassert((size_t)B % 4 == 0);
    _nassert((size_t)A % 4 == 0);

    size_t i, j, k;
    #pragma MUST_ITERATE(1,24)
    for (i = 0; i < AC; i++) {
        #pragma MUST_ITERATE(1,24)
        for (j = 0; j < BR; j++) {
            float t = 0.0;
            #pragma MUST_ITERATE(1,24)
            for (k = 0; k < AR; k++) {
                t += A[i * AR + k] * B[k * BR + j];
            }

            C[i * BR + j] = t * mul;
        }
    }
}

#endif
