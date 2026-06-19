# GEAR Sonic + Inspire RH56 Hand — Integration & Runbook

This documents the Inspire RH56 (FTP series) 6-DOF dexterous hand support for the
GEAR Sonic stack (Pico teleop + `gear_sonic_deploy` controller + VLA data
collection / inference), replacing the default Unitree dex3 three-finger hand.

> **Scope:** real-robot deploy + Pico teleop + VLA data collection/inference.
> MuJoCo sim of the Inspire hand is **not** supported (the sim bridge only speaks
> the dex3 `rt/dex3/*` topics). Use the real-robot path.

---

## 1. dex3 vs Inspire — the one thing to understand first

| | Unitree **dex3** | **Inspire RH56** |
|---|---|---|
| Who drives the motors | The **G1's own firmware** | A separate **userspace driver process** |
| Extra process to run? | **No** | **Yes** — `Headless_driver_l.py` / `_r.py` |
| Transport to hardware | Internal to the robot | **Modbus** (TCP `192.168.11.210:6000` or USB-serial) |
| DOF per hand | 7 | 6 |
| Command topic | `rt/dex3/{left,right}/cmd` | `rt/inspire_hand/ctrl/{l,r}` |
| State topic | `rt/dex3/{left,right}/state` | `rt/inspire_hand/state/{l,r}` |
| Command units | radians + kp/kd (position) | integer drive units `[0,1000]`, angle mode |

**Key point:** with dex3 you publish a DDS command and the robot firmware moves the
fingers — nothing else to launch. The Inspire hand is invisible to the G1 firmware;
it is a Modbus device, so you **must run the Inspire driver process**, which
subscribes to `rt/inspire_hand/ctrl/{l,r}` and writes Modbus registers on the hand.
Forgetting the driver is the #1 reason "nothing moves".

---

## 2. Architecture / data flow

```
Pico controller (XRoboToolkit: body + trigger/grip)
  → pico_manager_thread_server.py
      (G1GripperInverseKinematicsSolver: pinch → 6 Inspire joint angles, radians)
  → ZMQ pose/planner stream  (port 5556)
  → gear_sonic_deploy (C++ controller, --input-type zmq_manager)
      InspireHands: radians → angle_set[0..1000], mode=angle
  → publishes inspire_hand_ctrl on  rt/inspire_hand/ctrl/{l,r}
  → [Inspire driver process: Headless_driver_l/r.py]  → Modbus → hardware
  ← reads inspire_hand_state from rt/inspire_hand/state/{l,r} (angle_act → radians, logging)
```

The WBC policy controls the 29 body DOF; the hand is teleop/VLA passthrough and is
**not** part of the policy observation.

---

## 3. Key files

### Operator / teleop / VLA (Python, `gear_sonic/`)

| File | Role |
|---|---|
| `gear_sonic/utils/inspire_hand_spec.py` | Single source of truth: 6-DOF order, limits, radian↔drive mapping |
| `gear_sonic/utils/teleop/solver/hand/g1_gripper_ik_solver.py` | `G1GripperInverseKinematicsSolver` — pinch → 6-DOF progressive grasp |
| `gear_sonic/data/robot_model/supplemental_info/g1/g1_supplemental_info.py` | Inspire 6-DOF actuated joints/limits/groups |
| `gear_sonic/data/robot_model/instantiation/g1.py` | Loads `g1_29dof_with_inspire_hand.urdf` |
| `gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_inspire_hand.urdf` | Deploy URDF (6 revolute joints/hand) |
| `gear_sonic/envs/env_utils/joint_utils.py` | `G1_HAND_JOINTS` (12 = 6/hand) |
| `gear_sonic/scripts/run_vla_inference.py`, `run_data_exporter.py`, `data/features_sonic_vla.py` | 6-DOF hand through inference + dataset export |

### Robot controller (C++, `gear_sonic_deploy/`)

| File | Role |
|---|---|
| `.../include/inspire_hand_spec.hpp` | C++ mirror of the Python spec (`rad_to_drive` / `drive_to_rad`) |
| `.../include/inspire_hands.hpp` | `InspireHands` — publishes `inspire_hand_ctrl`, subscribes state |
| `.../include/inspire_idl/` , `.../src/inspire_idl/` | Vendored Cyclone-DDS Inspire message types (ctrl/state) |
| `.../src/g1_deploy_onnx_ref.cpp` | Uses `InspireHands` in place of `Dex3Hands` |
| `.../include/input_interface/zmq_endpoint_interface.hpp` | ZMQ parser accepts 6-DOF (and legacy 7) hand joints |

The old `.../include/dex3_hands.hpp` is now unused.

---

## 4. Joint specification (from the Inspire FTP user manual)

6 drives, in `angle_set` / `angle_act` index order (also the actuator order in the
robot model). Drive units are integers `[0, 1000]`, **`1000` = fully open,
`0` = fully closed**.

| idx | joint suffix | finger | modeled limit (rad) |
|---|---|---|---|
| 0 | `pinky`      | little finger        | `[0, 2.72]` |
| 1 | `ring`       | ring finger          | `[0, 2.72]` |
| 2 | `middle`     | middle finger        | `[0, 2.72]` |
| 3 | `index`      | index finger         | `[0, 2.72]` |
| 4 | `thumb_bend` | thumb proximal pitch | `[0, 1.45]` |
| 5 | `thumb_rot`  | thumb proximal yaw   | `[0, 1.31]` |

Mapping (open = 0 rad): `drive = 1000 · (Q_CLOSED − q) / Q_CLOSED`, clipped to
`[0, 1000]`. Kept numerically identical between `inspire_hand_spec.py` and
`inspire_hand_spec.hpp`. Command mode bit `0b0001` = angle mode.

---

## 5. Prerequisites

1. **Build `gear_sonic_deploy`** on the robot/onboard computer (needs CUDA / TensorRT /
   onnxruntime — a Jetson/GPU target, not a generic dev box):
   ```bash
   cd gear_sonic_deploy && ./deploy.sh real      # runs cmake/make, then launches
   ```
   The vendored Inspire IDL is compiled automatically (globbed into the build).

2. **Inspire SDK present** (`external_dependencies/inspire_hand_ws/`, a submodule):
   ```bash
   git submodule update --init external_dependencies/inspire_hand_ws
   cd external_dependencies/inspire_hand_ws/inspire_hand_sdk && pip install -e .
   ```

3. **Run the Inspire driver** on the **hand host** — the computer physically wired to
   the hands (the machine that can reach the Modbus IP / has the serial cable). On a
   typical G1 this is the **onboard computer that also runs `gear_sonic_deploy`**, and
   the two talk over local DDS. From `inspire_hand_sdk/example/`:
   ```bash
   python Headless_driver_l.py       # left  hand → rt/inspire_hand/{ctrl,state}/l
   python Headless_driver_r.py       # right hand → rt/inspire_hand/{ctrl,state}/r
   # serial wiring instead of Modbus-TCP: use Headless_driver_485_l.py / _r.py
   ```
   Default hand connection: Modbus TCP `192.168.11.210:6000`.

4. Hands powered (24 V) and reachable on the configured interface. If the deploy and
   the driver run on **different** machines, they must share the same DDS domain/network
   so `rt/inspire_hand/ctrl/*` crosses between them.

---

## 6. Running

The Pico teleop and VLA launchers need **no flag changes** — `--input-type zmq_manager`
already routes Pico/VLA → deploy, and the deploy now drives the Inspire hand.

### Teleop data collection
```bash
# On the hand host, first:
python .../inspire_hand_sdk/example/Headless_driver_l.py &
python .../inspire_hand_sdk/example/Headless_driver_r.py &

# Then the stack (launches deploy + Pico teleop + exporter + viewer):
python gear_sonic/scripts/launch_data_collection.py \
    --camera-host 192.168.123.164 \
    --task-prompt "pick up the cup"
```
Squeeze the Pico trigger → fingers close; release → open. The exporter records
6-wide `teleop.{left,right}_hand_joints`.

### VLA inference
```bash
# Inspire driver running as above, then:
python gear_sonic/scripts/launch_inference.py
```

---

## 7. Verifying without moving the robot

1. **Drive-unit parity** (host): the C++ and Python specs produce identical drive
   values (verified: `rad_to_drive(Q_OPEN)=1000…`, `rad_to_drive(Q_CLOSED)=0…`).
2. **DDS smoke test**: with `gear_sonic_deploy` + Inspire driver running, confirm the
   ctrl topic carries commands — e.g. the SDK's `example/dds_subscribe.py`, and check
   `angle_set ∈ [0,1000]`, `mode == 1`.
3. **End-to-end**: trigger on the Pico → fingers actuate.

---

## 8. Known limitations / notes

- **Sim unsupported** for the Inspire hand (deploy-only).
- **Grasp is coarse**: trigger → progressive close (index → pinky), not a
  fingertip-accurate per-finger retarget. Tune amplitudes in
  `g1_gripper_ik_solver.py` (`FINGER_FLEX`, `THUMB_BEND`, `THUMB_ROT`).
- Inspire state has no joint velocity, so `*_hand_dq` is logged as zeros.
- Topic case (`/l`, `/r`) and angle mode `0b0001` follow the Inspire SDK; if a hand's
  firmware differs it is a one-line change in `inspire_hands.hpp`.
- X/C keys still adjust the max-close-ratio (applied in radian space before the
  drive conversion).
