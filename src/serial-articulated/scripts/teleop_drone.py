#!/usr/bin/env python3
# Dragon drone keyboard teleop -- flight + arm control
#
# W/S  forward/backward   I/K  ascend/descend
# A/D  strafe             J/L  yaw
# Z/X  joint1   C/V  joint2   B/N  joint3   R  reset arm   Q  quit

import curses
import math
import time

import gz.transport13 as transport
from gz.msgs10 import boolean_pb2, double_pb2, pose_pb2

WORLD = 'dragon_world'
MODEL = 'dragon'
SVC   = f'/world/{WORLD}/set_pose'

FRAME_OFFSET = math.radians(107.3)

SPEED    = 0.1
TURN     = 0.2
TILT_MAX = 0.01
RATE     = 0.05

JOINT_STEP  = 0.05   # rad per keypress (~3°)
JOINT_LIMIT = 1.5708  # ±90°

BINDINGS = {
    ord('w'): ('vx', +1),
    ord('s'): ('vx', -1),
    ord('a'): ('vy', +1),
    ord('d'): ('vy', -1),
    ord('i'): ('vz', +1),
    ord('k'): ('vz', -1),
    ord('j'): ('wz', +1),
    ord('l'): ('wz', -1),
}

HELP = [
    "=== Dragon Teleop ===",
    "  W/S   : forward / backward",
    "  A/D   : strafe left / right",
    "  I/K   : ascend / descend",
    "  J/L   : yaw left / right",
    "  Space : stop",
    "--- Arm joints ---",
    "  Z/X   : joint1 (base)   +/-",
    "  C/V   : joint2 (middle) +/-",
    "  B/N   : joint3 (tip)    +/-",
    "  R     : reset arm",
    "  Q     : quit",
    "==================",
]


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    w  =  cr * cp * cy + sr * sp * sy
    qx =  sr * cp * cy - cr * sp * sy
    qy =  cr * sp * cy + sr * cp * sy
    qz =  cr * cp * sy - sr * sp * cy
    return w, qx, qy, qz


def send_pose(node, x, y, z, roll, pitch, yaw):
    req = pose_pb2.Pose()
    req.name = MODEL
    req.position.x = x
    req.position.y = y
    req.position.z = z
    w, qx, qy, qz = rpy_to_quat(roll, pitch, yaw)
    req.orientation.w = w
    req.orientation.x = qx
    req.orientation.y = qy
    req.orientation.z = qz
    try:
        node.request(SVC, req, pose_pb2.Pose, boolean_pb2.Boolean, 30)
    except Exception:
        pass


def send_joint(pub, angle):
    msg = double_pb2.Double()
    msg.data = angle
    pub.publish(msg)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(stdscr):
    node = transport.Node()

    pub1 = node.advertise('/arm/joint1/cmd_pos', double_pb2.Double)
    pub2 = node.advertise('/arm/joint2/cmd_pos', double_pb2.Double)
    pub3 = node.advertise('/arm/joint3/cmd_pos', double_pb2.Double)

    curses.curs_set(0)
    stdscr.nodelay(True)

    for i, line in enumerate(HELP):
        try:
            stdscr.addstr(i, 0, line)
        except curses.error:
            pass
    status_row = len(HELP) + 1
    arm_row    = status_row + 1

    px, py, pz = 0.0, 0.0, 1.0
    yaw = 0.0
    cmd = {'vx': 0, 'vy': 0, 'vz': 0, 'wz': 0}

    j = [0.0, 0.0, 0.0]   # joint angles in radians

    last = time.time()

    while True:
        key = stdscr.getch()

        if key == ord('q'):
            break
        elif key == ord(' '):
            cmd = {'vx': 0, 'vy': 0, 'vz': 0, 'wz': 0}
        elif key in BINDINGS:
            axis, sign = BINDINGS[key]
            cmd[axis] = sign

        # Arm joint keys
        elif key == ord('z'):
            j[0] = clamp(j[0] + JOINT_STEP, -JOINT_LIMIT, JOINT_LIMIT)
            send_joint(pub1, j[0])
        elif key == ord('x'):
            j[0] = clamp(j[0] - JOINT_STEP, -JOINT_LIMIT, JOINT_LIMIT)
            send_joint(pub1, j[0])
        elif key == ord('c'):
            j[1] = clamp(j[1] + JOINT_STEP, -JOINT_LIMIT, JOINT_LIMIT)
            send_joint(pub2, j[1])
        elif key == ord('v'):
            j[1] = clamp(j[1] - JOINT_STEP, -JOINT_LIMIT, JOINT_LIMIT)
            send_joint(pub2, j[1])
        elif key == ord('b'):
            j[2] = clamp(j[2] + JOINT_STEP, -JOINT_LIMIT, JOINT_LIMIT)
            send_joint(pub3, j[2])
        elif key == ord('n'):
            j[2] = clamp(j[2] - JOINT_STEP, -JOINT_LIMIT, JOINT_LIMIT)
            send_joint(pub3, j[2])
        elif key == ord('r'):
            j = [0.0, 0.0, 0.0]
            send_joint(pub1, 0.0)
            send_joint(pub2, 0.0)
            send_joint(pub3, 0.0)

        now = time.time()
        dt  = now - last
        last = now

        bvx = cmd['vx'] * SPEED
        bvy = cmd['vy'] * SPEED
        bvz = cmd['vz'] * SPEED
        wz  = cmd['wz'] * TURN

        heading = yaw + FRAME_OFFSET
        px  += (math.cos(heading) * bvx - math.sin(heading) * bvy) * dt
        py  += (math.sin(heading) * bvx + math.cos(heading) * bvy) * dt
        pz  += bvz * dt
        yaw += wz * dt

        fo = FRAME_OFFSET
        model_tx = cmd['vx'] * math.cos(fo) - cmd['vy'] * math.sin(fo)
        model_ty = cmd['vx'] * math.sin(fo) + cmd['vy'] * math.cos(fo)
        pitch_tilt = -TILT_MAX * model_tx
        roll_tilt  =  TILT_MAX * model_ty

        send_pose(node, px, py, pz, roll_tilt, pitch_tilt, yaw)

        try:
            stdscr.addstr(status_row, 0,
                f"  pos ({px:+.2f},{py:+.2f},{pz:+.2f})"
                f"  yaw {math.degrees(yaw):+.1f} deg   ")
            stdscr.addstr(arm_row, 0,
                f"  arm  j1={math.degrees(j[0]):+.1f}"
                f"  j2={math.degrees(j[1]):+.1f}"
                f"  j3={math.degrees(j[2]):+.1f} deg   ")
        except curses.error:
            pass
        stdscr.refresh()

        time.sleep(RATE)

    send_pose(node, px, py, pz, 0.0, 0.0, yaw)


if __name__ == '__main__':
    curses.wrapper(main)
