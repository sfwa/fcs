# coding=utf-8
import sys
import time
import socket


FIELDS = (
    "t", "lat", "lon", "alt", "vn", "ve", "vd", "q0", "q1", "q2", "q3", "yaw",
    "pitch", "roll", "vroll", "vpitch", "vyaw", "wn", "we", "wd", "mode"
)


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('127.0.0.1', 51000))
sock.sendall("sub sim/operation/override/override_planepath\n")
sock.sendall("set sim/operation/override/override_planepath [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\n")
sock.sendall("sub sim/flightmodel/position/psi\n")  # yaw
sock.sendall("sub sim/flightmodel/position/theta\n")  # pitch
sock.sendall("sub sim/flightmodel/position/phi\n")  # roll
sock.sendall("sub sim/flightmodel/position/local_x\n")
sock.sendall("sub sim/flightmodel/position/local_y\n")
sock.sendall("sub sim/flightmodel/position/local_z\n")
sock.sendall("extplane-set update_interval 0.01\n")


t = 0.0
for line in sys.stdin:
    data = dict(zip(FIELDS, line.strip("\n").split(",")))
    if data["t"] == "t":
        continue
    elif data["mode"] not in ("R", "A"):
        t = float(data["t"])
        continue
    elif float(data["t"]) < t:
        continue

    update = "world-set %.9f %.9f %.3f\n" % (float(data["lat"]),
                                             float(data["lon"]),
                                             float(data["alt"]))

    update += "set sim/flightmodel/position/psi %.2f\n" % float(data["yaw"])
    update += "set sim/flightmodel/position/theta %.2f\n" % float(data["pitch"])
    update += "set sim/flightmodel/position/phi %.2f\n" % float(data["roll"])

    print line.strip("\n")
    sock.sendall(update)

    t += 0.02
    time.sleep(0.02)
