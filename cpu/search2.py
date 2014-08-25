import os
import sys
import plog
import math
import numpy
import subprocess
import skimage.feature
import skimage.color
import skimage.exposure
import skimage.io
import skimage.draw


WGS84_A = 6378137.0
FOV_X = 1.712
FOV_Y = FOV_X

def cam_from_img(img):
    # Convert x, y pixel coordinates to a point on a plane at one metre
    # from the camera
    pt_x = (img[0] - 640.0) / 640.0 * FOV_X
    pt_y = (img[1] - 480.0) / 480.0 * FOV_Y

    return (pt_x, pt_y)


def geo_from_cam(cam, v_lat, v_lon, v_alt, v_q):
    # Convert x, y image coordinates (in metres at 1.0m viewing distance) to
    # body-frame coordinates
    cx = -cam[1]
    cy = cam[0]
    cz = 1.0

    # Transform body frame to world frame (TODO: may need to take conjugate)
    qx, qy, qz, qw = v_q
    qx = -qx
    qy = -qy
    qz = -qz

    tx = 2.0 * (qy * cz - qz * cy)
    ty = 2.0 * (qz * cx - qx * cz)
    tz = 2.0 * (qx * cy - qy * cx)

    rx = cx + qw * tx + qy * tz - qz * ty
    ry = cy + qw * ty + qz * tx - qx * tz
    rz = cz + qw * tz - qy * tx + qx * ty

    # Project the ray down to where it intersects with the ground plane.
    # If the z-component is negative or zero, the point is in the sky.
    if rz < 0.001:
        return None

    fac = v_alt / rz
    n = rx * fac
    e = ry * fac

    # Convert north/east offsets to lat/lon
    return (
        v_lat + n / WGS84_A,
        n,
        v_lon + e / (WGS84_A * math.cos(v_lat)),
        e
        )


def array_from_pgm(data, byteorder='>'):
    header, _, image = data.partition("\n")
    width, height, maxval = [int(item) for item in header.split()[1:]]
    return numpy.fromstring(
                image,
                dtype='u1' if maxval < 256 else byteorder + 'u2'
            ).reshape((height, width, 3))


def detect(pgm_path):
    lat = None
    lon = None
    alt = None
    q = None

    # Extract the attitude estimate for the current image
    fdir, fname = os.path.split(pgm_path)
    telem_path = os.path.join(
        fdir, "telem" + fname[len("img"):].rpartition(".")[0] + ".txt")
    with open(telem_path, "r") as telem_in:
        last_telem = list(p for p in plog.iterlogs_raw(telem_in))[-1]
        attitude_est = plog.ParameterLog.deserialize(last_telem)

        lat, lon, alt = attitude_est.find_by(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_ESTIMATED_POSITION_LLA).values
        lat = math.degrees(lat * math.pi / 2**31)
        lon = math.degrees(lon * math.pi / 2**31)
        alt *= 1e-2

        q = attitude_est.find_by(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_ESTIMATED_ATTITUDE_Q).values
        q = map(lambda x: float(x) / 32767.0, q)

    # Find blob coordinates
    with open(pgm_path, "r") as img_in:
        # Debayer the image
        debayer = subprocess.Popen(
            ["./debayer", "GBRG"], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True)
        img_out, blob_out = debayer.communicate(img_in.read())

        # Convert the output to a numpy array
        img_arr = skimage.img_as_float(array_from_pgm(img_out))

        h, w, d = img_arr.shape

        blobs = [eval(l) for l in blob_out.split("\n") if l]
        blue_blobs = sorted(list((b[3], b[1], b[2]) for b in blobs if b[0] == 'b'), reverse=True)
        yellow_blobs = sorted(list((b[3], b[1], b[2]) for b in blobs if b[0] == 'y'), reverse=True)
        L_blobs = sorted(list((b[3], b[1], b[2]) for b in blobs if b[0] == 'L'), reverse=True)

        target_blobs = {}

        for b1 in blue_blobs:
            best_hue = 0.0
            for b2 in yellow_blobs:
                if abs(b1[1] - b2[1]) < 5 and abs(b1[2] - b2[2]) < 5 and b1[0] * b2[0] > best_hue:
                    best_hue = b1[0] * b2[0]

            if best_hue < b1[0]:
                continue

            best_L = 0.0
            best_coords = None
            for b3 in L_blobs:
                if abs(b1[1] - b3[1]) < 5 and abs(b1[2] - b3[2]) < 5 and best_hue * b3[0] > best_L:
                    best_L = best_hue * b3[0]
                    best_coords = (b3[1], b3[2])

            if best_coords and target_blobs.get(best_coords, 0) < best_L:
                target_blobs[best_coords] = best_L

        # Draw a red line around the features by subtracting a circle
        # pattern from the G and B channels, and adding it to R.
        for coords, val in target_blobs.iteritems():
            print geo_from_cam(cam_from_img(coords), lat, lon, alt, q)

            if coords[0] < 10 or coords[0] > w - 10 or coords[1] < 10 or coords[1] > h - 10:
                continue

            rr, cc, val = skimage.draw.circle_perimeter_aa(
                coords[1], coords[0], 6)
            img_arr[rr, cc, 0] = val

        print "%s: %d Y blobs, %d B blobs, %d L blobs, %d targets" % (pgm_path, len(yellow_blobs), len(blue_blobs), len(L_blobs), len(target_blobs))

        # If blobs were found, write an image out showing where
        if len(target_blobs):
            out_path = pgm_path.rpartition(".")[0] + "-blobs.png"
            skimage.io.imsave(out_path, img_arr)


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        try:
            detect(arg)
        except Exception:
            print "Couldn't process %s" % arg
