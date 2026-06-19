#!/usr/bin/env python3
"""
welding.py  —  Dragon drone wall-mounted welding

The drone presses its body against the wall (fixed side).
The 3-joint connector arm acts as a robotic manipulator:
  - Base end  (base_link / drone body) : FIXED on wall surface at Y=1.3995
  - Tip end   (joint3_yaw)             : also on wall, sweeps a straight line in X

Kinematics
----------
Drone body pressed against wall at (0, 1.3995, 1.5).
Wall at Y = 1.3995 m (face).
The arm bends back from the wall, arcs in front, and the tip
touches the wall again at different X positions.
IK solutions sweep X from -0.700 m to +0.080 m (0.780 m weld line).

Usage
-----
  # Terminal 1
  ros2 launch serial-articulated display.launch.py

  # Terminal 2
  python3 src/serial-articulated/scripts/welding.py
"""

import math, time
import numpy as np
from scipy.optimize import fsolve

import subprocess

import gz.transport13 as transport
from gz.msgs10 import double_pb2

# ── Gazebo interface ──────────────────────────────────────────────────────
WORLD = 'dragon_world'
MODEL = 'dragon'
SVC   = f'/world/{WORLD}/set_pose'

# ── Welding parameters ────────────────────────────────────────────────────
WALL_Y     = 1.3995            # wall face Y in world frame
WELD_Z     = 1.50              # welding height
DRONE_POS  = (0.0, WALL_Y, WELD_Z)  # drone body PRESSED against wall (fixed side)
WELD_SPEED = 0.04              # tip speed along wall (m/s)
N_WAYPTS   = 60                # IK waypoints along the line

TICK = 0.05   # control loop period (s)

# ── Forward kinematics (connector chain, drone at DRONE_POS) ──────────────

def _R(r, p, y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p)
    cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp,   cp*sr,           cp*cr]])

def fk(q1, q2, q3):
    """Return world-frame tip position for joint angles q1,q2,q3."""
    dx, dy, dz = DRONE_POS
    p = np.array([dx, dy, dz], float)
    R = np.eye(3)
    # joint1_dummy (rpy 0 0.0012 -π) then revolute q1 around local Z
    R = R @ _R(0, 0.0012441, -math.pi) @ _R(0, 0, q1)
    p += np.array([0.41105, 0.751, -0.042412])
    # tube 1 (fixed)
    p += R @ np.array([-0.0425, 0.071, 0.0425])
    R  = R @ _R(1.5696, 0, 1.5708)
    # joint2_dummy
    p += R @ np.array([0.68, -0.042553, 0.042447])
    R  = R @ _R(1.5708, 0, 0) @ _R(0, 0, q2)
    # tube 2 (fixed)
    p += R @ np.array([0.0425, 0.071053, -0.042412])
    R  = R @ _R(-1.5708, 0, 1.5708)
    # joint3_dummy
    p += R @ np.array([0.68, -0.0425, 0.0425])
    R  = R @ _R(1.5708, 0, math.pi) @ _R(0, 0, q3)
    # tube 3 + half of arm-4
    p += R @ np.array([-0.0425, 0.071, 0.0425])
    R  = R @ _R(1.5708, 0, 1.5708)
    p += R @ np.array([0.34, 0, 0])
    return p

# ── Inverse kinematics ────────────────────────────────────────────────────
JOINT_LIM = 1.5708   # ±90°

def _ik_residual(q, x_target):
    p = fk(*q)
    return [p[0] - x_target, p[1] - WALL_Y, p[2] - WELD_Z]

def solve_ik(x_target, q_guess):
    """Return joint angles for tip at (x_target, WALL_Y, WELD_Z), or None."""
    sol, _, ier, _ = fsolve(_ik_residual, q_guess, args=(x_target,), full_output=True)
    err = max(abs(v) for v in _ik_residual(sol, x_target))
    if ier == 1 and err < 1e-5 and all(abs(sol[i]) <= JOINT_LIM for i in range(3)):
        return sol
    return None

def build_trajectory():
    """
    Walk outward from the known seed configuration, collecting all
    reachable (x_target, [q1,q2,q3]) pairs.  Returns list sorted by x.
    Drone body is pressed against wall (DRONE_POS[1] == WALL_Y); the arm
    bends back from the wall and the tip sweeps the same wall surface.
    """
    # Seed: drone at wall, tip at (-0.5, WALL_Y, WELD_Z), joints folded back
    q_seed = np.array([0.0531, -0.3793, 0.1734])
    waypoints = {}

    # Walk toward smaller X
    q = q_seed.copy()
    for xt in np.arange(-0.5, -1.10, -0.005):
        sol = solve_ik(xt, q)
        if sol is not None:
            waypoints[round(xt, 4)] = sol.tolist()
            q = sol
        else:
            break

    # Walk toward larger X
    q = q_seed.copy()
    for xt in np.arange(-0.5, +0.30, +0.005):
        sol = solve_ik(xt, q)
        if sol is not None:
            waypoints[round(xt, 4)] = sol.tolist()
            q = sol
        else:
            break

    traj = sorted(waypoints.items())   # [(x, [q1,q2,q3]), ...]
    return traj

# ── Gazebo helpers ────────────────────────────────────────────────────────

def lock_drone():
    """
    Snap drone to DRONE_POS using gz service CLI.
    Called only ONCE at start — with gravity=0 the drone stays put afterwards.
    Returns True on success.
    """
    req = (
        f'name: "{MODEL}" '
        f'position: {{x: {DRONE_POS[0]:.4f}, y: {DRONE_POS[1]:.4f}, z: {DRONE_POS[2]:.4f}}} '
        f'orientation: {{w: 1.0, x: 0.0, y: 0.0, z: 0.0}}'
    )
    result = subprocess.run([
        'gz', 'service', '-s', SVC,
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req,
    ], capture_output=True, text=True, timeout=5)
    ok = 'true' in result.stdout.lower()
    status = 'OK' if ok else f'FAILED (stdout={result.stdout.strip()!r})'
    print(f"    gz set_pose -> {status}")
    return ok

def send_joint(pub, angle):
    msg = double_pb2.Double()
    msg.data = float(angle)
    pub.publish(msg)

# ── Motion primitives ─────────────────────────────────────────────────────

def _smooth(t):
    t = max(0.0, min(1.0, t))
    return t*t*(3.0 - 2.0*t)

def _lerp(a, b, t):
    return a + (b - a) * t

def move_joints(pubs, q_start, q_end, duration):
    """Interpolate joint angles smoothly over `duration` seconds."""
    steps = max(1, round(duration / TICK))
    for i in range(steps + 1):
        t = _smooth(i / steps)
        for pub, a0, a1 in zip(pubs, q_start, q_end):
            send_joint(pub, _lerp(a0, a1, t))
        time.sleep(TICK)

def execute_weld(pubs, traj, speed):
    """
    Execute IK trajectory — drone locked by gravity=0, tip traces wall.
    `traj`: list of (x_tip, [q1,q2,q3]) sorted by x_tip.
    """
    xs  = [pt[0] for pt in traj]
    qs  = [pt[1] for pt in traj]
    total_dist = abs(xs[-1] - xs[0])

    print(f"  Weld line: tip X {xs[0]:.3f} -> {xs[-1]:.3f}  "
          f"({total_dist:.3f} m at {speed} m/s ~ {total_dist/speed:.0f} s)")

    for i in range(1, len(traj)):
        seg_dist = abs(xs[i] - xs[i-1])
        seg_time = max(seg_dist / speed, TICK)
        steps = max(1, round(seg_time / TICK))
        q0 = qs[i-1];  q1_v = qs[i]
        for step in range(steps + 1):
            t = step / steps
            for pub, a0, a1 in zip(pubs, q0, q1_v):
                send_joint(pub, _lerp(a0, a1, t))
            time.sleep(TICK)

        if i % 10 == 0 or i == len(traj)-1:
            pct = (i / (len(traj) - 1)) * 100
            cur_tip = fk(*qs[i])
            print(f"\r  {pct:5.1f}%  tip=({cur_tip[0]:+.3f}, {cur_tip[1]:.3f}, {cur_tip[2]:.3f})"
                  f"  q=({qs[i][0]:+.3f},{qs[i][1]:+.3f},{qs[i][2]:+.3f})",
                  end='', flush=True)
    print()

# ── Main ──────────────────────────────────────────────────────────────────

def main():
    # ── Pre-compute IK trajectory ─────────────────────────────────────
    print("Computing IK trajectory … ", end='', flush=True)
    traj = build_trajectory()
    if len(traj) < 5:
        print("ERROR: not enough IK solutions found. Abort.")
        return
    weld_len = abs(traj[-1][0] - traj[0][0])
    print(f"{len(traj)} waypoints,  line = {weld_len:.3f} m")

    q_home  = [0.0, 0.0, 0.0]
    q_start = traj[0][1]    # joints at weld line start
    q_end   = traj[-1][1]   # joints at weld line end

    # ── Init node ─────────────────────────────────────────────────────
    node = transport.Node()
    pub1 = node.advertise('/arm/joint1/cmd_pos', double_pb2.Double)
    pub2 = node.advertise('/arm/joint2/cmd_pos', double_pb2.Double)
    pub3 = node.advertise('/arm/joint3/cmd_pos', double_pb2.Double)
    pubs = [pub1, pub2, pub3]
    time.sleep(0.4)

    print("\n=== Dragon Wall-Mounted Welding ===")
    print(f"  Drone body PRESSED against wall at Y={DRONE_POS[1]:.4f} m  (fixed side)")
    print(f"  Arm tip SWEEPS same wall at Y={WALL_Y:.4f} m  (moving side)")
    print(f"  Weld Z={WELD_Z:.2f} m  |  line X {traj[0][0]:+.3f} -> {traj[-1][0]:+.3f}  ({weld_len:.3f} m)")
    print(f"  Speed: {WELD_SPEED} m/s   approx {weld_len/WELD_SPEED:.0f} s\n")

    # ── Press drone against wall via CLI (bypasses gz.transport timeout bug) ─
    print(f"[ ] Pressing drone against wall at Y={DRONE_POS[1]:.4f} via gz service ...")
    ok = lock_drone()
    time.sleep(1.5)
    if not ok:
        # Retry once
        lock_drone()
        time.sleep(1.0)

    # ── 1. Arm moves to weld-line start (drone stays still) ───────────
    print("[1/4] Positioning arm to weld start")
    move_joints(pubs, q_home, q_start, duration=3.0)
    tip = fk(*q_start)
    print(f"      Tip touching wall at: ({tip[0]:+.3f}, {tip[1]:.4f}, {tip[2]:.3f})")

    # ── 2. Weld — only joints move, drone locked ───────────────────────
    print("[2/4] Welding  (drone fixed, arm sweeps wall)")
    execute_weld(pubs, traj, speed=WELD_SPEED)

    # ── 3. Arm retracts (drone still) ────────────────────────────────
    print("[3/4] Retracting arm")
    move_joints(pubs, q_end, q_home, duration=3.0)

    # ── 4. Hold ───────────────────────────────────────────────────────
    print("[4/4] Complete — drone holding position")
    lock_drone()

    print("\nWelding complete.")
    print(f"  Weld line: Y={WALL_Y:.4f}  Z={WELD_Z:.2f}  "
          f"X={traj[0][0]:.3f}->{traj[-1][0]:.3f}")


if __name__ == '__main__':
    main()
