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


#WGS84 reference ellipsoid constants
wgs84_a = 6378137.0
wgs84_b = 6356752.314245
wgs84_e2 = 0.0066943799901975848
wgs84_a2 = wgs84_a**2 #to speed things up a bit
wgs84_b2 = wgs84_b**2


#convert ECEF to lat/lon/alt without geoid correction
def ecef2llh((x,y,z)):
    ep  = math.sqrt((wgs84_a2 - wgs84_b2) / wgs84_b2)
    p   = math.sqrt(x**2+y**2)
    th  = math.atan2(wgs84_a*z, wgs84_b*p)
    lon = math.atan2(y, x)
    lat = math.atan2(z+ep**2*wgs84_b*math.sin(th)**3, p-wgs84_e2*wgs84_a*math.cos(th)**3)
    N   = wgs84_a / math.sqrt(1-wgs84_e2*math.sin(lat)**2)
    alt = p / math.cos(lat) - N

    lon *= (180. / math.pi)
    lat *= (180. / math.pi)

    return (lat, lon, alt)


def ecef2ned(lat_ref, lng_ref, dx, dy, dz):
    # dx, dy and dz are the offset from lat_ref/lng_ref to the desired point,
    # in ECEF coordinates.
    lat_ref = math.radians(lat_ref)
    lng_ref = math.radians(lng_ref)

    e = -1.0 * math.sin(lng_ref) * dx + math.cos(lng_ref) * dy
    n = -1.0 * math.sin(lat_ref) * math.cos(lng_ref) * dx + -1.0 * math.sin(lat_ref) * math.sin(lng_ref) * dy + math.cos(lat_ref) * dz
    u = math.cos(lat_ref) * math.cos(lng_ref) * dx + math.cos(lat_ref) * math.sin(lng_ref) * dy + math.sin(lat_ref) * dz

    return (n, e, -u)


def llh2ecef(lat, lng, alt):
    lat = math.radians(lat)
    lng = math.radians(lng)
    chi = math.sqrt(1.0 - wgs84_e2 * math.sin(lat)**2)

    x = (wgs84_a / chi + alt) * math.cos(lat) * math.cos(lng)
    y = (wgs84_a / chi + alt) * math.cos(lat) * math.sin(lng)
    z = (wgs84_a * (1.0 - wgs84_e2) / chi + alt) * math.sin(lat)

    return (x, y, z)


UKF_ORIGIN = 0, 0, 0
UKF_TIME_DILATION = 1.0 # as a fraction of normal speed

LAST_SCREENSHOT = 0

VIDEO_START_FRAME = 273.704

#CAMERA_INTRINSICS = [
#    [ 299.39646639,    0.,          419.96165812],
#    [   0.,          302.5602385,   230.25411049],
#    [   0.,            0.,            1.        ]
#]

CAMERA_Q = euler_to_q(0.0, -3.15, 1.0)

CAMERA_INTRINSICS = [
    [ 429.39646639,    0.,          424.96165812],
    [   0.,          432.5602385,   240.25411049],
    [   0.,            0.,            1.        ]
]
FIELDS = (
    "t", "lat", "lon", "alt", "vn", "ve", "vd", "q0", "q1", "q2", "q3", "yaw",
    "pitch", "roll", "vroll", "vpitch", "vyaw", "wn", "we", "wd", "mode"
)


infile = open(sys.argv[1], "rU")
for line in infile:
    data = dict(zip(FIELDS, line.strip("\n").split(",")))
    if data["t"] == "t":
        continue
    elif float(data["t"]) < VIDEO_START_FRAME:
        continue
    else:
        UKF_ORIGIN = llh2ecef(float(data['lat']), float(data['lon']), float(data['alt']))
        break

i = 0

# Set up the video display
source = pyglet.media.load(sys.argv[2], streaming=True)
horizon_info = open(sys.argv[2].rpartition(".")[0] + "-horizon.txt", "rU")

video_format = source.video_format
texture = pyglet.image.Texture.create(video_format.width, video_format.height,
    rectangle=True)
texture = texture.get_transform(flip_y=True)
texture.anchor_y = 0

window = pyglet.window.Window(video_format.width, video_format.height)

label = pyglet.text.Label("Sample",
                          font_name="Monaco",
                          font_size=10,
                          x=4, y=window.height - 4,
                          anchor_x="left", anchor_y="top",
                          multiline=True, width=150,
                          color=(255, 0, 0, 255))
pyglet.gl.glBlendFunc(pyglet.gl.GL_SRC_ALPHA, pyglet.gl.GL_ONE_MINUS_SRC_ALPHA)
pyglet.gl.glEnable(pyglet.gl.GL_BLEND)
pyglet.gl.glEnable(pyglet.gl.GL_LINE_SMOOTH);
pyglet.gl.glHint(pyglet.gl.GL_LINE_SMOOTH_HINT, pyglet.gl.GL_NICEST)
pyglet.gl.glLineWidth(3)

# Start the video

@window.event
def on_key_press(symbol, modifiers):
    global UKF_TIME_DILATION
    if symbol in (pyglet.window.key.PLUS, pyglet.window.key.EQUAL):
        UKF_TIME_DILATION = max(UKF_TIME_DILATION - 1.0, 1.0)
    elif symbol in (pyglet.window.key.MINUS, pyglet.window.key.UNDERSCORE):
        UKF_TIME_DILATION = min(UKF_TIME_DILATION + 1.0, 100.0)


def draw_horizon(show_compass=True, _cache={}):
    if "horizon" not in _cache:
        # Draw the horizon at 10km out
        iterations = 36
        radius = 10000.0
        s = math.sin(2.0 * math.pi / float(iterations))
        c = math.cos(2.0 * math.pi / float(iterations))
        horizon_vertices = []
        compass_vertices = []
        dx = radius
        dy = 0

        for i in range(iterations):
            horizon_vertices += [dx, dy, 0]
            if i % 3 == 0:
                compass_vertices += [dx, dy, 0,
                                     dx, dy, -500.0 if i else -4000.0]
            dx, dy = (dx * c - dy * s), (dy * c + dx * s)

        _cache["horizon"] = pyglet.graphics.vertex_list(
            iterations, ("v3f/static", tuple(horizon_vertices)))
        _cache["compass_points"] = pyglet.graphics.vertex_list(
            iterations / 3 * 2, ("v3f/static", tuple(compass_vertices)))

    _cache["horizon"].draw(pyglet.gl.GL_LINE_LOOP)
    if show_compass:
        _cache["compass_points"].draw(pyglet.gl.GL_LINES)


def convert_hz_intrinsic_to_opengl_projection(K, x0, y0, width, height, znear,
                                              zfar):
    znear = float(znear)
    zfar = float(zfar)
    depth = zfar - znear
    q = -(zfar + znear) / depth
    qn = -2 * (zfar * znear) / depth

    proj = np.array([[ 2*K[0,0]/width, -2*K[0,1]/width, (-2*K[0,2]+width+2*x0)/width, 0 ],
                         [  0,              2*K[1,1]/height,( 2*K[1,2]-height+2*y0)/height, 0],
                         [0,0,q,qn],  # This row is standard glPerspective and sets near and far planes.
                         [0,0,-1,0]]) # This row is also standard glPerspective.
    return proj


def camera_projection():
    # Set up OpenGL projection matrix for the current camera. According to
    # OpenCV, the GoPro Hero3 Black in 480p240 mode's parameters are:
    # RMS: 0.334648205672
    # camera matrix:
    # [[ 299.39646639    0.          419.96165812]
    #  [   0.          302.5602385   230.25411049]
    #  [   0.            0.            1.        ]]
    # distortion coefficients:  [-0.16792771  0.03121603  0.00218195 -0.00026904 -0.00263317]
    znear, zfar = 1, 20000
    intrinsic = np.array(CAMERA_INTRINSICS)
    proj = convert_hz_intrinsic_to_opengl_projection(
        intrinsic,
        # origin X, origin Y, width, height, znear, zfar
        0, 0, 848, 480, 10, 12000
    )

    m = map(float, proj.T.flat)
    m = (pyglet.gl.GLfloat * 16)(*m)

    pyglet.gl.glMatrixMode(pyglet.gl.GL_PROJECTION)
    pyglet.gl.glLoadMatrixf(m)
    pyglet.gl.glMatrixMode(pyglet.gl.GL_MODELVIEW)


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


def update(datafile, dt):
    global UKF_TIME_DILATION, VIDEO_START_FRAME, FIELDS, \
           UKF_ORIGIN, LAST_SCREENSHOT, horizon_info

    window.clear()

    horizon_line = None
    horizon_yaw, horizon_pitch, horizon_roll = 0.0, 0.0, 0.0
    estimate = None

    # Update the UKF based on the time elapsed (should normally be 1/60th of a
    # second). UKF_TIME_DILATION determines the real time:UKF time ratio.
    line = datafile.next()
    while line:
        estimate = dict(zip(FIELDS, line.strip("\n").split(",")))

        dt -= 0.001 * UKF_TIME_DILATION
        if dt < 0.0:
            break
        else:
            line = datafile.next()

    # If the UKF timestamp has reached the next video frame, get a copy of it
    # in our texture object
    frame = None
    video_frame_ts = source.get_next_video_timestamp()
    while video_frame_ts <= float(estimate['t']) - VIDEO_START_FRAME:
        frame = source.get_next_video_frame()
        video_frame_ts = source.get_next_video_timestamp()
        if horizon_info:
            horizon_line = map(float, horizon_info.next().strip("\n").split(","))

    if frame:
        texture.blit_into(frame, 0, 0, 0)

    # Switch back to pyglet default view
    pyglet.gl.glMatrixMode(pyglet.gl.GL_PROJECTION)
    pyglet.gl.glLoadIdentity()
    pyglet.gl.glOrtho(0, window.width, 0, window.height, -1, 1)
    pyglet.gl.glMatrixMode(pyglet.gl.GL_MODELVIEW)
    pyglet.gl.glLoadIdentity()

    # Display the video texture
    pyglet.gl.glColor4f(1.0, 1.0, 1.0, 1.0)
    texture.blit(0, 0)

    # Work out yaw, pitch and roll from information overlay
    q = vectors.Q(-float(estimate['q0']), -float(estimate['q1']),
                  -float(estimate['q2']), float(estimate['q3']))

    airflow = [
        float(estimate['vn']) - float(estimate['wn']),
        float(estimate['ve']) - float(estimate['we']),
        float(estimate['vd']) - float(estimate['wd'])
    ]

    #print airflow

    wind = q.conjugate() * \
           vectors.Q(airflow[0], airflow[1], airflow[2], 0.0) * q
    alpha = math.degrees(math.atan2(wind[2], wind[0]))
    beta = math.degrees(math.atan2(wind[1], math.sqrt(wind[0] ** 2 + wind[2] ** 2)))

    # Display the current UKF sample time and information overlay
    label.text = "t %7.0f @ %2.0f\nyaw %5.0f\npitch %3.0f\nroll %4.0f\n" \
                 "alt %7.1f\nias %7.1f\ngs %8.1f\nalpha %5.1f\nbeta %6.1f\nwind\n  N %7.1f\n  E %7.1f\n  s %7.1f\n" % (
                    float(estimate['t']), UKF_TIME_DILATION,
                    float(estimate['yaw']), float(estimate['pitch']),
                    float(estimate['roll']),
                    float(estimate['alt']),
                    norm(*airflow),
                    norm(float(estimate['vn']), float(estimate['ve']), 0),
                    alpha, beta,
                    float(estimate['wn']), float(estimate['we']),
                    norm(float(estimate['wn']), float(estimate['we']), 0),
                 )
    label.draw()

    if horizon_line:
        vx, vy, x0, y0 = tuple(horizon_line)
        horizon_pitch, horizon_roll = attitude_from_horizon(
            x0, y0, x0 + vx, y0 + vy, float(estimate['roll']))
        horizon_q = euler_to_q(float(estimate['yaw']), horizon_pitch, horizon_roll) * CAMERA_Q.conjugate()
        horizon_yaw, horizon_pitch, horizon_roll = q_to_euler(horizon_q)

    # Switch to 3D mode
    camera_projection()
    pyglet.gl.glMatrixMode(pyglet.gl.GL_MODELVIEW)
    pyglet.gl.glLoadIdentity()

    # Work out the current modelview transform based on UKF attitude and
    # position

    # Camera was mounted about 10° down
    q = vectors.Q(-float(estimate['q0']), -float(estimate['q1']),
                  -float(estimate['q2']), float(estimate['q3']))
    q *= CAMERA_Q

    forward = (q * vectors.Q(1.0, 0.0, 0.0, 0.0)) * q.conjugate()
    up = (q * vectors.Q(0.0, 0.0, -1.0, 0.0)) * q.conjugate()

    x, y, z = llh2ecef(float(estimate['lat']),
                       float(estimate['lon']),
                       float(estimate['alt']))
    n, e, d = ecef2ned(float(estimate['lat']), float(estimate['lon']),
                       x - UKF_ORIGIN[0], y - UKF_ORIGIN[1],
                       z - UKF_ORIGIN[2])

    pyglet.gl.gluLookAt(
        n, -e, -float(estimate['alt']) + 150.0,
        n + forward[0], -e + forward[1],
        -float(estimate['alt']) + 150.0 + forward[2],
        up[0], up[1], up[2]
    )

    # Render the artifical horizon
    pyglet.gl.glColor4f(1.0, 0.0, 0.0, 1.0)
    draw_horizon()

    # Render the origin marker
    pyglet.graphics.draw_indexed(3, pyglet.gl.GL_TRIANGLES,
        [0, 1, 2],
        ('v3f', (0, 2, 0,
                 0, -2, 0,
                 2, 0, 0))
    )

    # Render the detected horizon -- transform based on what the modelview
    # matrix WOULD be if the pitch and roll were as calculated by the horizon
    # detection code
    if False and horizon_line:
        horizon_q = euler_to_q(horizon_yaw, horizon_pitch, horizon_roll) * CAMERA_Q

        forward = (horizon_q * vectors.Q(1.0, 0.0, 0.0, 0.0)) * horizon_q.conjugate()
        up = (horizon_q * vectors.Q(0.0, 0.0, -1.0, 0.0)) * horizon_q.conjugate()

        pyglet.gl.glLoadIdentity()
        pyglet.gl.gluLookAt(
            n, -e, -float(estimate['alt']) + 150.0,
            n + forward[0], -e + forward[1],
            -float(estimate['alt']) + 150.0 + forward[2],
            up[0], up[1], up[2]
        )

        pyglet.gl.glColor4f(1.0, 1.0, 0.0, 1.0)
        draw_horizon(show_compass=False)

    # Save a screenshot every second
    #if 720000 <= float(estimate['t']) <= 900000:
    #    pyglet.image.get_buffer_manager().get_color_buffer().save('/Users/bendyer/Desktop/example/ukf-%d.png' % float(estimate['t']))
    #    LAST_SCREENSHOT = float(estimate['t'])


pyglet.clock.schedule_interval(functools.partial(update, infile), 1.0 / 60.0)
pyglet.app.run()
