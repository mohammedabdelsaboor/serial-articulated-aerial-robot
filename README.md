# Serial Articulated Aerial Robot (SAAR)

A reconfigurable, thrust-vectoring aerial robot for contact-based industrial tasks — inspection, welding, and manipulation in confined or high-altitude spaces that conventional drones can't safely reach.

Mechanical design · kinematic & dynamic modeling · control system development · simulation · experimental validation
EJUST Mechatronics Graduation Project

---

## 1. Problem

Conventional multi-rotor drones are underactuated: their rotors all point one way, so the airframe can't resist the reaction forces and moments generated during physical contact — pushing, holding, fixing, welding. That same fixed geometry forces drones into ballistic, high-speed passes through narrow openings (long runway in, long runway out), which doesn't work in cluttered or hazardous industrial environments.

SAAR addresses both problems at once with two coupled capabilities:

- **Shape morphing** — a serial chain of articulated links lets the body fold down to a slender profile to pass through manholes, ducts, and tight structural gaps.
- **Thrust vectoring** — independently tiltable rotor pairs on every link let the robot redirect thrust to counteract contact forces, so it can stay stable while pushing against a surface instead of just hovering near it.

### Target applications
1. **Storage tank maintenance** — internal/external inspection and NDT in petroleum facilities, entering through narrow manholes.
2. **Aerial welding & structural fixation** — precision contact tasks (arc welding, assembly) while compensating for the induced reaction wrench.
3. **Offshore oil & gas operations** — maintenance and repair on rigs and platforms under environmental disturbance.
4. **Civil infrastructure inspection** — structural assessments on bridges and suspension cables.
5. **Telecom tower servicing** — inspection and alignment of antenna arrays at height, against wind gusts.

---

## 2. System architecture

The robot follows the **DRAGON** (Dual-rotor embedded multilink) architecture introduced by Zhao et al., JSK Lab, University of Tokyo (*"Design, Modeling, and Control of an Aerial Robot DRAGON,"* IEEE RA-L, 2018) — a serial chain of bicopter modules connected by single-axis joints, with consecutive joint axes rotated 90° from each other so the chain can curl in any plane, not just a single one.

```
base_link ── joint1 ── joint2 ── joint3
   │            │          │         │
 (rigid)    [link1]    [link2]   [link3+4]
            bicopter   bicopter   bicopter
```

| Component | DOF | Description |
|---|---|---|
| Connector chain | 3 revolute joints | `joint1_dummy`, `joint2_dummy`, `Joint3_dummy`, each ±90°. Fixed 90° twists between them give full out-of-plane bending with single-axis actuators. |
| Bicopter modules | 4 links × 2 rotors | Each link carries two independently tiltable rotor units (servo + propeller) — **8 thrusters total**, each able to redirect its own thrust vector. |
| Body | 1 rigid base | `base_link`, the root of the connector chain. |

This gives the robot two simultaneous control handles: it can change its *shape* (joint angles) and its *thrust distribution* (rotor tilt + speed) to hold a pose under contact load.

---

## 3. Repository structure

This repo covers the full project, not only the simulation. Mechanical, modeling, and validation work live alongside the ROS 2 package:

```
serial-articulated-aerial-robot/
├── README.md
├── docs/                          # ⏳ project report, problem statement, figures/diagrams
├── cad/                           # ⏳ SolidWorks assemblies/parts, drawings, BOM
├── modeling/                      # ⏳ kinematic & dynamic modeling, control system design
│   ├── dynamics/                  #     ADAMS models, equations of motion
│   ├── control/                   #     Simulink control system models
│   └── matlab/                    #     MATLAB scripts (PID tuning, analysis, plots)
├── experiments/                   # ⏳ hardware test data, validation logs, results
└── src/serial-articulated/        # ✅ ROS 2 (ament_cmake) simulation package
    ├── package.xml / CMakeLists.txt
    ├── urdf/
    │   ├── dragon.xacro            # robot description (active)
    │   ├── dragon_arm.xacro        # bicopter module macro (active)
    │   ├── materials.xacro
    │   ├── dragon.urdf             # flattened reference export
    │   ├── bicopter.xacro          # early draft, superseded
    │   ├── connectors.xacro        # early draft, superseded
    │   └── thrusters.xacro         # single-arm standalone test
    ├── meshes/                    # 18 STL files (SolidWorks export)
    ├── launch/display.launch.py   # builds world + spawns robot in gz sim
    ├── worlds/dragon.sdf           # alternate static-model world (unused by launch file)
    ├── model.sdf / model.config    # Gazebo model package (alternate path)
    ├── rviz/arm.rviz
    └── scripts/
        ├── teleop_drone.py         # keyboard teleop: body pose + arm joints
        └── welding.py              # FK/IK wall-welding demo
```

`✅` = present now. `⏳` = planned — create the folder when you add the first file of that kind; `src/serial-articulated/` stays as-is so colcon still finds it as a ROS 2 package (colcon only looks inside `src/`, so the sibling folders above won't interfere with the build).

> **Note on the simulation package:** `bicopter.xacro`, `connectors.xacro`, `thrusters.xacro`, `dragon.urdf`, and `model.sdf`/`worlds/dragon.sdf` are earlier drafts or standalone test files kept for reference — the live simulation is built entirely from `dragon.xacro` + `dragon_arm.xacro` via `display.launch.py`.

---

## 4. Getting started (simulation)

### Prerequisites
- ROS 2 (Jazzy or newer)
- Gazebo Harmonic (`gz sim`)
- `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `ros_gz_sim`
- Python 3 with `numpy`, `scipy` (for the welding demo)

### Build
```bash
colcon build
source install/setup.bash
```

### Run the simulation
```bash
ros2 launch serial-articulated display.launch.py
```
This spawns the robot in Gazebo (gravity disabled for this kinematic demo), opens RViz, and starts joint-position controllers on:
- `/arm/joint1/cmd_pos`
- `/arm/joint2/cmd_pos`
- `/arm/joint3/cmd_pos`

### Keyboard teleop
```bash
python3 src/serial-articulated/scripts/teleop_drone.py
```
| Key | Action | Key | Action |
|---|---|---|---|
| `W` / `S` | forward / back | `I` / `K` | ascend / descend |
| `A` / `D` | strafe | `J` / `L` | yaw |
| `Z` / `X` | joint 1 ± | `C` / `V` | joint 2 ± |
| `B` / `N` | joint 3 ± | `R` | reset arm |
| `Space` | stop | `Q` | quit |

### Wall-welding demo
```bash
# terminal 1
ros2 launch serial-articulated display.launch.py
# terminal 2
python3 src/serial-articulated/scripts/welding.py
```
The body presses against and locks onto a wall; the script solves inverse kinematics for the 3-joint arm and sweeps the tip along a 0.78 m line on the same wall — a proof-of-concept for application #2 above (contact-based welding while compensating body position).

---

## 5. Project status

| Workstream | Status |
|---|---|
| Mechanical design (CAD) | ⏳ To be added (`cad/`) |
| Kinematic & dynamic modeling | ⏳ To be added (`modeling/dynamics/`) |
| Control system development | ⏳ To be added (`modeling/control/`) |
| Simulation — shape-morphing kinematics & visualization | ✅ Implemented |
| Simulation — contact-task IK (welding demo) | ✅ Implemented |
| Simulation — thrust allocation for force compensation | ⏳ Not yet — propellers currently spin at a fixed visual rate |
| Simulation — flight dynamics (gravity-on, closed-loop control) | ⏳ Not yet — current demos run with gravity disabled |
| Experimental validation | ⏳ To be added (`experiments/`) |
| Project report / documentation | ⏳ To be added (`docs/`) |

The simulation package currently demonstrates the **kinematic feasibility** of the shape-morphing + contact-manipulation concept. Next milestones: thrust-allocation control tying rotor tilt/speed to a desired body wrench, closed-loop flight control with gravity enabled, and the mechanical/dynamics/control deliverables above.

---

## 6. Acknowledgment

The link/joint architecture is based on the published DRAGON design:
M. Zhao, T. Anzai, F. Shi, X. Chen, K. Okada, and M. Inaba, "Design, Modeling, and Control of an Aerial Robot DRAGON: A Dual-Rotor Embedded Multilink Robot with the Ability of Multi-Degree-of-Freedom Aerial Transformation," *IEEE Robotics and Automation Letters*, vol. 3, no. 2, pp. 1176–1183, 2018.

