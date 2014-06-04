# coding=utf-8
import sys
import math
import ctypes
import time
import socket
import pyglet
import vectors
import numpy as np
import cv2
import os
import functools
import collections


def norm(x, y, z):
    return math.sqrt(x*x + y*y + z*z)


def q_to_euler(q):
    q0, q1, q2, q3 = q[3], q[0], q[1], q[2]
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                     1.0 - 2.0 * (q2 ** 2.0 + q3 ** 2.0))
    pitch = math.asin(2.0 * (q0 * q2 - q3 * q1))
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3),
                      1.0 - 2.0 * (q1 ** 2.0 + q2 ** 2.0))

    yaw = (math.degrees(yaw) + 360.0) % 360.0
    pitch = math.degrees(pitch)
    roll = math.degrees(roll)
    return yaw, pitch, roll


def euler_to_q(yaw, pitch, roll):
    return (vectors.Q.rotate("Z", math.radians(yaw)) *
            vectors.Q.rotate("Y", math.radians(pitch)) *
            vectors.Q.rotate("X", math.radians(roll)))


VIDEO_START_FRAME = 273.687
CAMERA_Q = euler_to_q(0.0, 0.0, 0.5)

CAMERA_INTRINSICS = [
    [ 399.39646639,    0.,          419.96165812],
    [   0.,          392.5602385,   230.25411049],
    [   0.,            0.,            1.        ]
]
FIELDS = (
    "t", "lat", "lon", "alt", "vn", "ve", "vd", "q0", "q1", "q2", "q3", "yaw",
    "pitch", "roll", "vroll", "vpitch", "vyaw", "wn", "we", "wd", "mode"
)

def attitude_from_horizon(x0, y0, x1, y1, expected_roll):
    # Determine pitch and roll from a horizon line segment identified by the
    # points (x0, x1) and (y0, y1) in an image.
    # Uses the CAMERA_INTRINSICS values (fx, 0, cx, 0, fy, cy, 0, 0, 1) to
    # convert the point to a normalized (u, v) representation, then uses
    # formulas 27 and 33 from http://eprints.qut.edu.au/12839/1/3067a485.pdf
    # to determine pitch and roll.

    x, y = x0, y0
    mx, my = x1 - x0, y1 - y0
    fx, fy = CAMERA_INTRINSICS[0][0], CAMERA_INTRINSICS[1][1]
    cx, cy = CAMERA_INTRINSICS[0][2], CAMERA_INTRINSICS[1][2]

    # X and Y are flipped in the intrinsics matrix relative to the fitLine
    # output -- no, I don't know why.
    u = (y - cx) / fx
    v = (x - cy) / fy
    f = 1.0

    roll = math.atan2(-mx, my)
    # Ensure that roll is pointing in approx the same direction as
    # expected_roll
    if expected_roll - math.degrees(roll) > 135.0:
        roll += math.pi
    elif expected_roll - math.degrees(roll) < -135.0:
        roll -= math.pi

    pitch = math.atan((u * math.sin(roll) + v * math.cos(roll)) / f)

    return math.degrees(pitch), math.degrees(roll)


PITCH_ERRORS = []
ROLL_ERRORS = []
PITCH_ERRORS_SIGNED = []
ROLL_ERRORS_SIGNED = []


def headless_update(datafile, horizon_info, t):
    global FIELDS, PITCH_ERRORS, ROLL_ERRORS, ROLL_ERRORS_SIGNED, \
           PITCH_ERRORS_SIGNED

    horizon_line = map(float, horizon_info.next().strip("\n").split(","))
    horizon_yaw, horizon_pitch, horizon_roll = 0.0, 0.0, 0.0

    estimate = None
    line = datafile.next()
    while line:
        estimate = dict(zip(FIELDS, line.strip("\n").split(",")))
        if float(estimate['t']) >= t:
            break
        else:
            line = datafile.next()

    # Work out yaw, pitch and roll from information overlay
    q = vectors.Q(-float(estimate['q0']), -float(estimate['q1']),
                  -float(estimate['q2']), float(estimate['q3']))

    if horizon_line:
        vx, vy, x0, y0 = tuple(horizon_line)
        horizon_pitch, horizon_roll = attitude_from_horizon(
            x0, y0, x0 + vx, y0 + vy, float(estimate['roll']))
        horizon_q = euler_to_q(
            float(estimate['yaw']), horizon_pitch, horizon_roll) * CAMERA_Q.conjugate()
        horizon_yaw, horizon_pitch, horizon_roll = q_to_euler(horizon_q)

    # Collect summary stats
    if not math.isnan(horizon_pitch) and not math.isnan(horizon_roll) \
            and float(estimate['t']) > 345.0:
        pitch_error = float(estimate['pitch']) - horizon_pitch
        roll_error = float(estimate['roll']) - horizon_roll

        PITCH_ERRORS.append(abs(pitch_error))
        ROLL_ERRORS.append(abs(roll_error))
        PITCH_ERRORS_SIGNED.append(pitch_error)
        ROLL_ERRORS_SIGNED.append(roll_error)

        print "%.3f,%.2f,%.2f,%.2f,%.2f" % (
            float(estimate['t']), horizon_roll, horizon_pitch,
            float(estimate['roll']), float(estimate['pitch'])
        )
    else:
        print "%.3f,,,%.2f,%.2f" % (
            float(estimate['t']), float(estimate['roll']), float(estimate['pitch'])
        )


print "t,ref_roll,ref_pitch,roll,pitch"
try:
    t = VIDEO_START_FRAME
    infile = open(sys.argv[1], "rU")
    horizon_info = open(sys.argv[2].rpartition(".")[0] + "-horizon.txt", "rU")

    infile.next() # get rid of headers

    while True:
        headless_update(infile, horizon_info, t)
        t += (1.0 / 240.0)
except StopIteration:
    sys.stderr.write("""
Frames: %d
Pitch offset: %.6f
Roll offset: %.6f

Pitch error percentiles: 50th %.3f, 75th %.3f, 95th %.3f, RMS %.3f
Roll error percentiles: 50th %.3f, 75th %.3f, 95th %.3f, RMS %.3f
""" % (
    len(PITCH_ERRORS),
    np.percentile(PITCH_ERRORS_SIGNED, 50),
    np.percentile(ROLL_ERRORS_SIGNED, 50),
    np.percentile(PITCH_ERRORS, 50), np.percentile(PITCH_ERRORS, 75),
    np.percentile(PITCH_ERRORS, 95), np.sqrt(np.mean(np.square(PITCH_ERRORS))),
    np.percentile(ROLL_ERRORS, 50), np.percentile(ROLL_ERRORS, 75),
    np.percentile(ROLL_ERRORS, 95), np.sqrt(np.mean(np.square(ROLL_ERRORS))))
)
